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
  http.end();
  return code;
}

void netTask(void*) {
  uint32_t lastTryMs = 0;
  uint32_t lastSendMs = 0;
  uint32_t sentSignature = 0;
  bool sentOnce = false;
  for (;;) {
    uint32_t now = millis();
    if (WiFi.status() != WL_CONNECTED) {
      if (now - lastTryMs >= cfg::Net::WIFI_RETRY_MS || lastTryMs == 0) {
        lastTryMs = now;
        WiFi.disconnect();
        WiFi.begin(cfg::Net::WIFI_SSID, cfg::Net::WIFI_PASS);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
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

bool connected() { return WiFi.status() == WL_CONNECTED; }
int rssi() { return connected() ? WiFi.RSSI() : 0; }
int lastHttpCode() { return httpCode; }

}  // namespace netlink
