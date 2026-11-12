#include "device_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace device_log {
namespace {

constexpr size_t ENTRY_COUNT = 24;
constexpr size_t TEXT_SIZE = 96;

struct Entry {
  uint32_t ms;
  char text[TEXT_SIZE];
};

Entry entries[ENTRY_COUNT];
size_t nextEntry = 0;
size_t entryCount = 0;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

bool append(char* out, size_t capacity, size_t& used, const char* text, size_t length) {
  if (length >= capacity - used) return false;
  memcpy(out + used, text, length);
  used += length;
  out[used] = '\0';
  return true;
}

bool appendJsonString(char* out, size_t capacity, size_t& used, const char* text) {
  if (!append(out, capacity, used, "\"", 1)) return false;
  for (const unsigned char* p = reinterpret_cast<const unsigned char*>(text); *p; ++p) {
    char escaped[7];
    size_t length = 1;
    escaped[0] = static_cast<char>(*p);
    if (*p == '"' || *p == '\\') {
      escaped[0] = '\\';
      escaped[1] = static_cast<char>(*p);
      length = 2;
    } else if (*p < 0x20) {
      snprintf(escaped, sizeof(escaped), "\\u%04x", *p);
      length = 6;
    }
    if (!append(out, capacity, used, escaped, length)) return false;
  }
  return append(out, capacity, used, "\"", 1);
}

}  // namespace

void addf(const char* format, ...) {
  Entry entry = {};
  entry.ms = millis();
  va_list args;
  va_start(args, format);
  vsnprintf(entry.text, sizeof(entry.text), format, args);
  va_end(args);
  portENTER_CRITICAL(&logMux);
  entries[nextEntry] = entry;
  nextEntry = (nextEntry + 1) % ENTRY_COUNT;
  if (entryCount < ENTRY_COUNT) ++entryCount;
  portEXIT_CRITICAL(&logMux);
}

size_t appendJson(char* out, size_t capacity) {
  Entry snapshot[ENTRY_COUNT];
  size_t count;
  portENTER_CRITICAL(&logMux);
  count = entryCount;
  const size_t first = (nextEntry + ENTRY_COUNT - count) % ENTRY_COUNT;
  for (size_t i = 0; i < count; ++i) snapshot[i] = entries[(first + i) % ENTRY_COUNT];
  portEXIT_CRITICAL(&logMux);

  if (!capacity) return 0;
  size_t used = 0;
  out[0] = '\0';
  if (!append(out, capacity, used, "[", 1)) return 0;
  for (size_t i = 0; i < count; ++i) {
    char prefix[40];
    const int prefixLength = snprintf(prefix, sizeof(prefix), "%s{\"ms\":%lu,\"text\":",
                                      i ? "," : "", static_cast<unsigned long>(snapshot[i].ms));
    if (prefixLength < 0 || !append(out, capacity, used, prefix, static_cast<size_t>(prefixLength)) ||
        !appendJsonString(out, capacity, used, snapshot[i].text) ||
        !append(out, capacity, used, "}", 1)) {
      return 0;
    }
  }
  if (!append(out, capacity, used, "]", 1)) return 0;
  return used;
}

}  // namespace device_log
