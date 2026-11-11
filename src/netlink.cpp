#include "netlink.h"

#include <ctype.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Update.h>
#include <WiFi.h>

namespace netlink {
namespace {

SemaphoreHandle_t mutex = nullptr;
Status shared;
bool haveStatus = false;
uint32_t signature = 0;
bool sharedWakeActive = false;
volatile int httpCode = 0;
String deviceMac;
struct ManualWake {
  char id[33] = {};
  uint8_t slot = 0;
};
ManualWake pendingManualWake;
bool manualWakePending = false;
String lastManualWakeId;
String wakeAck;

// Zugangsdaten stehen im Flash (NVS). Eine per USB geflashte Version mit secrets.h schreibt sie dorthin;
// die fertige Release-Datei von GitHub enthaelt keine und nutzt die gespeicherten Werte.
String wifiSsid = cfg::Net::WIFI_SSID;
String wifiPass = cfg::Net::WIFI_PASS;
String piHost = cfg::Net::PI_HOST;
String apiKey = cfg::Net::API_KEY;

void loadCredentials() {
  Preferences p;
  if (!p.begin("net", false)) return;
  if (cfg::Net::WIFI_PASS[0]) {
    p.putString("ssid", cfg::Net::WIFI_SSID);
    p.putString("pass", cfg::Net::WIFI_PASS);
    p.putString("pi", cfg::Net::PI_HOST);
    p.putString("key", cfg::Net::API_KEY);
  } else if (p.isKey("ssid")) {
    wifiSsid = p.getString("ssid", wifiSsid);
    wifiPass = p.getString("pass", wifiPass);
    piHost = p.getString("pi", piHost);
    apiKey = p.getString("key", apiKey);
  } else {
    Serial.println("Keine WLAN-Daten gespeichert: einmal per USB mit src/secrets.h flashen");
  }
  p.end();
}

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
uint32_t makeSignature(const Status& s, bool wakeActive) {
  uint32_t h = (uint32_t)s.light;
  h = h * 31 + s.muted + 2 * s.m.tempAlarm + 4 * s.m.ahtOk + 8 * s.m.i2cOk;
  h = h * 31 + wakeActive;
  for (const SlotState& x : s.m.slots) h = h * 31 + (x.charging | (x.fault << 1));
  return h;
}

void appendNum(char* out, size_t n, float v, int dec) {
  if (isnan(v)) snprintf(out, n, "null");
  else snprintf(out, n, "%.*f", dec, v);
}

// Feldnamen siehe README (Schnittstelle zum Pi)
size_t buildBody(const Status& s, bool wakeActive, char* buf, size_t cap) {
  char t[16], h[16];
  String ack;
  if (mutex && xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    ack = wakeAck;
    xSemaphoreGive(mutex);
  }
  appendNum(t, sizeof(t), s.m.tempC, 1);
  appendNum(h, sizeof(h), s.m.humidity, 0);
  uint8_t charging = 0;
  for (const SlotState& x : s.m.slots) charging += x.charging;
  size_t n = snprintf(buf, cap,
                      "{\"mac\":\"%s\",\"fw\":\"%s\",\"uptime_s\":%lu,\"rssi\":%d,"
                      "\"temp\":%s,\"humidity\":%s,\"aht_ok\":%s,\"temp_alarm\":%s,\"i2c_ok\":%s,"
                      "\"light\":\"%s\",\"fault\":%s,\"muted\":%s,"
                      "\"charging\":%u,\"slots\":[",
                      deviceMac.c_str(), FW_VERSION, (unsigned long)(millis() / 1000), WiFi.RSSI(), t, h,
                      s.m.ahtOk ? "true" : "false", s.m.tempAlarm ? "true" : "false",
                      s.m.i2cOk ? "true" : "false", lightName(s.light), s.anyFault ? "true" : "false",
                      s.muted ? "true" : "false", charging);
  for (uint8_t i = 0; i < cfg::SLOT_COUNT && n < cap; i++) {
    const SlotState& x = s.m.slots[i];
    char since[16] = "null";
    if (x.changed) snprintf(since, sizeof(since), "%lu", (unsigned long)x.sinceS);
    n += snprintf(buf + n, cap - n, "%s{\"n\":%u,\"charging\":%s,\"fault\":%s,\"vbat\":%.2f,\"since_s\":%s}",
                  i ? "," : "", i + 1, x.charging ? "true" : "false", x.fault ? "true" : "false", x.vbat, since);
  }
  if (n < cap) {
    n += snprintf(buf + n, cap - n, "],\"wake_active\":%s", wakeActive ? "true" : "false");
  }
  if (ack.length() && n < cap) n += snprintf(buf + n, cap - n, ",\"wake_ack\":\"%s\"", ack.c_str());
  if (n < cap) n += snprintf(buf + n, cap - n, "}");
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
remotebeep::Config pendingBeep;
bool beepPending = false;
// Restzeit fuer die Startkontakte beim Taster-Update (vom Pi); 0 = aus
volatile uint32_t otaHoldUntilMs = 0;
volatile bool otaHoldOn = false;
long numAfter(const String& body, const char* key, int from, long dflt) {
  int k = body.indexOf(key, from);
  if (k < 0) return dflt;
  int colon = body.indexOf(':', k);
  if (colon < 0) return dflt;
  return strtol(body.c_str() + colon + 1, nullptr, 10);
}

// Liest "beep":{"yellow_on":..,"yellow_seq":..,"red_on":..,"red_ms":..,"red_every_ms":..}
bool parseBeep(const String& body, remotebeep::Config& out) {
  int b = body.indexOf("\"beep\"");
  if (b < 0) return false;
  out.yellowOn = numAfter(body, "\"yellow_on\"", b, 0) != 0;
  out.yellowSeq = (uint32_t)numAfter(body, "\"yellow_seq\"", b, 0);
  out.redOn = numAfter(body, "\"red_on\"", b, 0) != 0;
  out.redMs = (uint16_t)constrain(numAfter(body, "\"red_ms\"", b, 2000), 100, 30000);
  out.redEveryMs = (uint32_t)constrain(numAfter(body, "\"red_every_ms\"", b, 10000), 1000, 3600000);
  return true;
}

// Kennung in der Firmware-Datei: daran liest der Pi die Version der hochgeladenen Datei
extern const char kFwMarker[] __attribute__((used));
const char kFwMarker[] __attribute__((used)) = "KOFFER-FW:" FW_VERSION;
// Der volatile-Zeiger, der beim Start gelesen wird, haelt die Kennung gegen den Linker im Image
const char* volatile kFwMarkerRef = kFwMarker;

struct OtaOffer {
  char version[16];
  char md5[40];
  uint32_t size;
};

String jsonString(const String& body, const char* key, int from) {
  int k = body.indexOf(key, from);
  if (k < 0) return "";
  int q1 = body.indexOf('"', body.indexOf(':', k) + 1);
  if (q1 < 0) return "";
  int q2 = body.indexOf('"', q1 + 1);
  return q2 < 0 ? String("") : body.substring(q1 + 1, q2);
}

bool parseManualWake(const String& body, ManualWake& out, bool& fieldPresent) {
  const int w = body.indexOf("\"manual_wake\"");
  if (w < 0) {
    fieldPresent = false;
    return false;
  }
  fieldPresent = true;
  const int open = body.indexOf('{', w);
  if (open < 0) return false;
  const int close = body.indexOf('}', open);
  if (close < 0) return false;
  const String item = body.substring(open, close + 1);
  const String id = jsonString(item, "\"id\"", 0);
  const long slot = numAfter(item, "\"slot\"", 0, 0);
  if (id.length() != 32 || slot < 1 || slot > cfg::SLOT_COUNT) return false;
  for (unsigned int i = 0; i < id.length(); i++) {
    if (!isxdigit((unsigned char)id[i])) return false;
  }
  strlcpy(out.id, id.c_str(), sizeof(out.id));
  out.slot = (uint8_t)slot;
  return true;
}

// Liest "ota":{"md5":"..","size":N,"version":".."}; false = kein Update angeboten
bool parseOta(const String& body, OtaOffer& out) {
  int o = body.indexOf("\"ota\"");
  if (o < 0) return false;
  String v = jsonString(body, "\"version\"", o), m = jsonString(body, "\"md5\"", o);
  long sz = numAfter(body, "\"size\"", o, 0);
  if (!v.length() || m.length() != 32 || sz < 100000 || sz > 1966080) return false;
  if (v == FW_VERSION) return false;
  // Nur neuere Versionen einspielen (kein Downgrade, z. B. nach dem Flashen per USB)
  int ma = 0, mi = 0, pa = 0;
  if (sscanf(v.c_str(), "v%d.%d.%d", &ma, &mi, &pa) == 3) {
    const long offered = ma * 1000000L + mi * 1000L + pa;
    const long mine = FW_VERSION_MAJOR * 1000000L + FW_VERSION_MINOR * 1000L + FW_VERSION_PATCH;
    if (offered <= mine) return false;
  }
  strlcpy(out.version, v.c_str(), sizeof(out.version));
  strlcpy(out.md5, m.c_str(), sizeof(out.md5));
  out.size = (uint32_t)sz;
  return true;
}

// Laedt die Firmware vom Pi und startet neu. Kehrt nur bei einem Fehler zurueck.
bool runOta(const OtaOffer& o) {
  Serial.printf("Update auf %s (%lu Bytes) ...\n", o.version, (unsigned long)o.size);
  String url = String("http://") + piHost + ":" + cfg::Net::PI_PORT + "/api/charger/firmware";
  HTTPClient http;
  http.setConnectTimeout(cfg::Net::HTTP_TIMEOUT_MS);
  http.setTimeout(15000);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  if (!http.begin(url)) return false;
  if (apiKey.length()) http.addHeader("X-Api-Key", apiKey);
  bool ok = false;
  if (http.GET() == 200 && (uint32_t)http.getSize() == o.size && Update.begin(o.size)) {
    Update.setMD5(o.md5);
    ok = Update.writeStream(*http.getStreamPtr()) == o.size && Update.end(true);
    if (!ok) {
      Serial.printf("Update fehlgeschlagen: %s\n", Update.errorString());
      Update.abort();
    }
  } else {
    Serial.println("Update: Download nicht moeglich");
  }
  http.end();
  return ok;
}

int post(const Status& s, bool wakeActive) {
  static char body[1400];
  if (buildBody(s, wakeActive, body, sizeof(body)) >= sizeof(body)) return -1;
  String url = String("http://") + piHost + ":" + cfg::Net::PI_PORT + "/api/charger";
  HTTPClient http;
  http.setConnectTimeout(cfg::Net::HTTP_TIMEOUT_MS);
  http.setTimeout(cfg::Net::HTTP_TIMEOUT_MS);
  http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
  if (!http.begin(url)) return -1;
  http.addHeader("Content-Type", "application/json");
  if (apiKey.length()) http.addHeader("X-Api-Key", apiKey);
  int code = http.POST((uint8_t*)body, strlen(body));
  OtaOffer offer = {"", "", 0};
  if (code == 200) {
    const String reply = http.getString();
    if (reply.indexOf("\"ota_hold_ms\"") >= 0) {
      const long hold = constrain(numAfter(reply, "\"ota_hold_ms\"", 0, 0), 0, 3600000);
      otaHoldUntilMs = millis() + (uint32_t)hold;
      otaHoldOn = hold > 0;
    }    if (!parseOta(reply, offer)) offer.size = 0;
    wake::Config w;
    remotebeep::Config bp;
    ManualWake manual = {};
    bool manualFieldPresent = false;
    const bool haveManual = parseManualWake(reply, manual, manualFieldPresent);
    const String confirmedWakeAck = jsonString(reply, "\"wake_ack\"", 0);
    const bool haveWake = parseWake(reply, w);
    const bool haveBeep = parseBeep(reply, bp);
    if ((haveWake || haveBeep || manualFieldPresent || confirmedWakeAck.length()) &&
        xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (haveWake) {
        pendingWake = w;
        wakePending = true;
      }
      if (haveBeep) {
        pendingBeep = bp;
        beepPending = true;
      }
      if (haveManual && lastManualWakeId != manual.id) {
        pendingManualWake = manual;
        manualWakePending = true;
        lastManualWakeId = manual.id;
      } else if (manualFieldPresent && !haveManual && !confirmedWakeAck.length()) {
        pendingManualWake = {};
        manualWakePending = false;
        lastManualWakeId = "";
      }
      if (confirmedWakeAck.length() && confirmedWakeAck == lastManualWakeId) {
        pendingManualWake = {};
        manualWakePending = false;
        lastManualWakeId = "";
        if (wakeAck == confirmedWakeAck) wakeAck = "";
      }
      xSemaphoreGive(mutex);
    }
  }
  http.end();
  if (offer.size) {
    static uint32_t lastFailMs = 0;
    // Nach einem Fehlschlag 5 Minuten warten, damit der Koffer nicht dauernd laedt
    if (lastFailMs == 0 || millis() - lastFailMs > 300000) {
      if (runOta(offer)) {
        Serial.println("Update fertig, Neustart");
        delay(300);
        ESP.restart();
      }
      lastFailMs = millis();
    }
  }
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
        Serial.printf("WLAN: Verbindungsversuch zu \"%s\" (vorheriger Statuscode %d)\n", wifiSsid.c_str(),
                      (int)WiFi.status());
        lastTryMs = now;
        WiFi.disconnect();
        WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
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
    bool wakeActive = false;
    bool have = false;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      copy = shared;
      sig = signature;
      wakeActive = sharedWakeActive;
      have = haveStatus;
      xSemaphoreGive(mutex);
    }
    const uint32_t since = now - lastSendMs;
    const bool changed = sig != sentSignature && httpCode == 200;  // nach Fehlern nur im Normaltakt
    const bool due = !sentOnce || since >= cfg::Net::SEND_INTERVAL_MS ||
                     (changed && since >= cfg::Net::MIN_GAP_MS);
    if (have && due) {
      int code = post(copy, wakeActive);
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
  loadCredentials();
  deviceMac = WiFi.macAddress();
  Serial.printf("Firmware %s\n", kFwMarkerRef + 10);
  xTaskCreate(netTask, "netlink", 10240, nullptr, 1, nullptr);
}

void publish(const Status& s, bool wakeActive) {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (!mutex || now - lastMs < 100) return;
  if (xSemaphoreTake(mutex, 0) != pdTRUE) return;  // nie warten, der naechste Durchlauf folgt
  lastMs = now;
  shared = s;
  signature = makeSignature(s, wakeActive);
  sharedWakeActive = wakeActive;
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

bool takeManualWakeRequest(String& id, uint8_t& slot) {
  if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return false;
  const bool have = manualWakePending;
  if (have) {
    id = pendingManualWake.id;
    slot = pendingManualWake.slot;
    pendingManualWake = {};
    manualWakePending = false;
  }
  xSemaphoreGive(mutex);
  return have;
}

void retryManualWakeRequest(const String& id, uint8_t slot) {
  if (!mutex || xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  strlcpy(pendingManualWake.id, id.c_str(), sizeof(pendingManualWake.id));
  pendingManualWake.slot = slot;
  manualWakePending = true;
  lastManualWakeId = id;
  xSemaphoreGive(mutex);
}

void acknowledgeManualWake(const String& id) {
  if (!mutex || xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  wakeAck = id;
  xSemaphoreGive(mutex);
}

bool takeBeepConfig(remotebeep::Config& out) {
  if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return false;
  const bool have = beepPending;
  if (have) {
    out = pendingBeep;
    beepPending = false;
  }
  xSemaphoreGive(mutex);
  return have;
}

bool otaHoldActive() { return otaHoldOn && (int32_t)(otaHoldUntilMs - millis()) > 0; }

bool connected() { return WiFi.status() == WL_CONNECTED; }
int rssi() { return connected() ? WiFi.RSSI() : 0; }
int lastHttpCode() { return httpCode; }

}  // namespace netlink
