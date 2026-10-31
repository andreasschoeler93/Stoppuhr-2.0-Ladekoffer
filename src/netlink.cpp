#include "netlink.h"

#include <HTTPClient.h>
#include <WiFi.h>

namespace netlink {
namespace {

SemaphoreHandle_t mutex = nullptr;
Status shared;
bool haveStatus = false;
uint32_t signature = 0;
volatile int httpCode = 0;
String deviceMac;

const char* lightName(Light l) {
  switch (l) {
    case Light::GREEN: return "green";
    case Light::YELLOW: return "yellow";
    case Light::YELLOW_BLINK: return "yellow_blink";
    case Light::RED: return "red";
    case Light::RED_BLINK: return "red_blink";
    default: return "off";
  }
}

// Fasst alles zusammen, was eine sofortige Meldung rechtfertigt (ohne Messwerte wie vbat)
uint32_t makeSignature(const Status& s) {
  uint32_t h = (uint32_t)s.light;
  h = h * 31 + s.muted + 2 * s.m.tempAlarm + 4 * s.m.ahtOk + 8 * s.m.i2cOk;
  for (const SlotState& x : s.m.slots) h = h * 31 + (x.present | (x.charging << 1) | (x.full << 2) | (x.fault << 3));
  return h;
}

void appendNum(char* out, size_t n, float v, int dec) {
  if (isnan(v)) snprintf(out, n, "null");
  else snprintf(out, n, "%.*f", dec, v);
}

// Feldnamen siehe README (Schnittstelle zum Pi)
size_t buildBody(const Status& s, char* buf, size_t cap) {
  char t[16], h[16];
  appendNum(t, sizeof(t), s.m.tempC, 1);
  appendNum(h, sizeof(h), s.m.humidity, 0);
  uint8_t present = 0, charging = 0, full = 0;
  for (const SlotState& x : s.m.slots) {
    present += x.present;
    charging += x.charging;
    full += x.full;
  }
  size_t n = snprintf(buf, cap,
                      "{\"mac\":\"%s\",\"fw\":\"%s\",\"uptime_s\":%lu,\"rssi\":%d,"
                      "\"temp\":%s,\"humidity\":%s,\"aht_ok\":%s,\"temp_alarm\":%s,\"i2c_ok\":%s,"
                      "\"light\":\"%s\",\"fault\":%s,\"muted\":%s,"
                      "\"present\":%u,\"charging\":%u,\"full\":%u,\"slots\":[",
                      deviceMac.c_str(), FW_VERSION, (unsigned long)(millis() / 1000), WiFi.RSSI(), t, h,
                      s.m.ahtOk ? "true" : "false", s.m.tempAlarm ? "true" : "false",
                      s.m.i2cOk ? "true" : "false", lightName(s.light), s.anyFault ? "true" : "false",
                      s.muted ? "true" : "false", present, charging, full);
  for (uint8_t i = 0; i < cfg::SLOT_COUNT && n < cap; i++) {
    const SlotState& x = s.m.slots[i];
    n += snprintf(buf + n, cap - n, "%s{\"n\":%u,\"present\":%s,\"charging\":%s,\"full\":%s,\"fault\":%s,\"vbat\":%.2f}",
                  i ? "," : "", i + 1, x.present ? "true" : "false", x.charging ? "true" : "false",
                  x.full ? "true" : "false", x.fault ? "true" : "false", x.vbat);
  }
  if (n < cap) n += snprintf(buf + n, cap - n, "]}");
  return n;
}

// Liest "wake":{"pins":[..],"ms":..,"gap_ms":..} aus der Antwort des Pi
bool parseWake(const String& body, wake::Config& out) {
  int w = body.indexOf("\"wake\"");
  if (w < 0) return false;
  int p = body.indexOf("\"pins\"", w);
  int open = p < 0 ? -1 : body.indexOf('[', p);
  if (open < 0) return false;
  const char* s = body.c_str() + open + 1;
  wake::Config c = wake::defaults();
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    char* end = nullptr;
    long v = strtol(s, &end, 10);
    if (end == s || v < 0 || v > 15) return false;
    c.pins[i] = (uint8_t)v;
    s = end;
    while (*s == ' ' || *s == ',') s++;
  }
  auto num = [&](const char* key, long lo, long hi, uint16_t& dst) {
    int k = body.indexOf(key, w);
    if (k < 0) return;
    int colon = body.indexOf(':', k);
    if (colon < 0) return;
    long v = strtol(body.c_str() + colon + 1, nullptr, 10);
    if (v >= lo && v <= hi) dst = (uint16_t)v;
  };
  num("\"ms\"", 100, 30000, c.pulseMs);
  num("\"gap_ms\"", 100, 30000, c.gapMs);
  out = c;
  return true;
}

wake::Config pendingWake;
bool wakePending = false;

int post(const Status& s) {
  static char body[1400];
  if (buildBody(s, body, sizeof(body)) >= sizeof(body)) return -1;
  String url = String("http://") + cfg::Net::PI_HOST + ":" + cfg::Net::PI_PORT + "/api/charger";
  HTTPClient http;
  http.setConnectTimeout(cfg::Net::HTTP_TIMEOUT_MS);
  http.setTimeout(cfg::Net::HTTP_TIMEOUT_MS);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  if (!http.begin(url)) return -1;
  http.addHeader("Content-Type", "application/json");
  if (cfg::Net::API_KEY[0]) http.addHeader("X-Api-Key", cfg::Net::API_KEY);
  int code = http.POST((uint8_t*)body, strlen(body));
  if (code == 200) {
    wake::Config c;
    if (parseWake(http.getString(), c) && xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      pendingWake = c;
      wakePending = true;
      xSemaphoreGive(mutex);
    }
  }
  http.end();
  return code;
}

void netTask(void*) {
  uint32_t lastTryMs = 0;
  uint32_t lastSendMs = 0;
  uint32_t sentSignature = 0;
  bool sentOnce = false;
  bool wasConnected = false;
  for (;;) {
    uint32_t now = millis();
    if (WiFi.status() != WL_CONNECTED) {
      if (wasConnected) {
        Serial.printf("WLAN getrennt (Statuscode %d)\n", (int)WiFi.status());
        wasConnected = false;
      }
      if (now - lastTryMs >= cfg::Net::WIFI_RETRY_MS || lastTryMs == 0) {
        // Der Statuscode vor dem Neustart zeigt, woran der letzte Versuch scheiterte
        Serial.printf("WLAN: Verbindungsversuch zu \"%s\" (vorheriger Statuscode %d)\n", cfg::Net::WIFI_SSID,
                      (int)WiFi.status());
        lastTryMs = now;
        WiFi.disconnect();
        WiFi.begin(cfg::Net::WIFI_SSID, cfg::Net::WIFI_PASS);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (!wasConnected) {
      Serial.printf("WLAN verbunden, IP: %s\n", WiFi.localIP().toString().c_str());
      wasConnected = true;
    }

    Status copy;
    uint32_t sig = 0;
    bool have = false;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      copy = shared;
      sig = signature;
      have = haveStatus;
      xSemaphoreGive(mutex);
    }
    const uint32_t since = now - lastSendMs;
    const bool changed = sig != sentSignature && httpCode == 200;  // nach Fehlern nur im Normaltakt
    const bool due = !sentOnce || since >= cfg::Net::SEND_INTERVAL_MS ||
                     (changed && since >= cfg::Net::MIN_GAP_MS);
    if (have && due) {
      int code = post(copy);
      httpCode = code;
      lastSendMs = millis();
      // Auch bei Fehlschlag erst nach dem normalen Abstand wiederholen (Pi nicht ueberlasten)
      if (code == 200) sentSignature = sig;
      sentOnce = true;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

}  // namespace

void begin() {
  mutex = xSemaphoreCreateMutex();
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);  // das WLAN ist versteckt
  WiFi.setHostname("ladekoffer");
  WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);  // Der Pi-Zugangspunkt wird sonst bei aelterem WPA-Modus abgelehnt (Grund 211)
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t info) {
    Serial.printf("WLAN getrennt, Grund %d\n", (int)info.wifi_sta_disconnected.reason);
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  deviceMac = WiFi.macAddress();
  xTaskCreate(netTask, "netlink", 8192, nullptr, 1, nullptr);
}

void publish(const Status& s) {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (!mutex || now - lastMs < 100) return;
  if (xSemaphoreTake(mutex, 0) != pdTRUE) return;  // nie warten, der naechste Durchlauf folgt
  lastMs = now;
  shared = s;
  signature = makeSignature(s);
  haveStatus = true;
  xSemaphoreGive(mutex);
}

bool takeWakeConfig(wake::Config& out) {
  if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return false;
  const bool have = wakePending;
  if (have) {
    out = pendingWake;
    wakePending = false;
  }
  xSemaphoreGive(mutex);
  return have;
}

bool connected() { return WiFi.status() == WL_CONNECTED; }
int rssi() { return connected() ? WiFi.RSSI() : 0; }
int lastHttpCode() { return httpCode; }

}  // namespace netlink
