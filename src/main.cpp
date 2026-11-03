#include <Arduino.h>
#include <WiFi.h>

#include "board.h"
#include "config.h"
#include "netlink.h"
#include "sensors.h"
#include "signals.h"

static const Status* current = nullptr;

static void printStatus() {
  const Status& s = *current;
  Serial.printf("\n=== Ladekoffer %s ===\n", FW_VERSION);
  Serial.printf("Temp: %.1f C (aht=%s, alarm=%s)  I2C: %s\n", s.m.tempC, s.m.ahtOk ? "ok" : "FEHLT",
                s.m.tempAlarm ? "JA" : "nein", s.m.i2cOk ? "ok" : "AUSFALL");
  Serial.printf("Alle da=%s alle voll=%s laedt=%s Fehler=%s quittiert=%s\n", s.allPresent ? "ja" : "nein",
                s.allFull ? "ja" : "nein", s.anyCharging ? "ja" : "nein", s.anyFault ? "ja" : "nein",
                s.muted ? "ja" : "nein");
  Serial.printf("WLAN: %s (%d dBm, Statuscode %d), letzter HTTP-Code: %d\n",
                netlink::connected() ? "verbunden" : "getrennt", netlink::rssi(),
                (int)WiFi.status(), netlink::lastHttpCode());
  Serial.println("Slot | da | laedt | voll | Fehler | Vbat");
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    const SlotState& x = s.m.slots[i];
    Serial.printf("%4u | %2s | %5s | %4s | %6s | %.3f V\n", i + 1, x.present ? "ja" : "-", x.charging ? "ja" : "-",
                  x.full ? "ja" : "-", x.fault ? "JA" : "-", x.vbat);
  }
}

// Befehle: status | mute | unmute (nicht blockierend, nur verfuegbare Zeichen)
static void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      if (line.length() < 32) line += c;
      continue;
    }
    line.trim();
    line.toLowerCase();
    if (line == "status" || line == "s") printStatus();
    else if (line == "mute") signals::mute(true);
    else if (line == "unmute") signals::mute(false);
    else if (line == "wake") wake::start();
    else if (line.length()) Serial.println("Befehle: status | mute | unmute | wake");
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  board::begin();
  sensors::begin();
  signals::begin();
  wake::begin();
  current = &signals::update(sensors::get());
  netlink::begin();
  Serial.printf("Ladekoffer-Controller %s bereit.\n", FW_VERSION);
}

void loop() {
  static uint32_t manualWakeRetryAtMs = 0;
  board::service();
  sensors::update();
  current = &signals::update(sensors::get());
  wake::Config wakeCfg;
  if (netlink::takeWakeConfig(wakeCfg)) wake::configure(wakeCfg);
  remotebeep::Config beepCfg;
  if (netlink::takeBeepConfig(beepCfg)) remotebeep::configure(beepCfg);
  if (!wake::active() && millis() >= manualWakeRetryAtMs) {
    String requestId;
    uint8_t slot = 0;
    if (netlink::takeManualWakeRequest(requestId, slot)) {
      if (wake::startSlot(slot, requestId)) {
        manualWakeRetryAtMs = 0;
      } else {
        netlink::retryManualWakeRequest(requestId, slot);
        manualWakeRetryAtMs = millis() + cfg::MCP_RETRY_MS;
      }
    }
  }
  wake::update();
  const String completedWakeId = wake::takeCompletedRequestId();
  if (completedWakeId.length()) netlink::acknowledgeManualWake(completedWakeId);
  netlink::publish(*current, wake::active());
  handleSerial();
}
