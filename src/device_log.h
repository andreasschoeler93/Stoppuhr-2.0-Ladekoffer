#pragma once

#include <Arduino.h>

namespace device_log {

void addf(const char* format, ...);
size_t appendJson(char* out, size_t capacity);

}  // namespace device_log
