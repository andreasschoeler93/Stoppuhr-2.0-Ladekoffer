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
  Serial.printf("Laedt=%s Fehler=%s quittiert=%s\n", s.anyCharging ? "ja" : "nein", s.anyFault ? "ja" : "nein",
                s.muted ? "ja" : "nein");
  Serial.printf("WLAN: %s (%d dBm, Statuscode %d), letzter HTTP-Code: %d\n",
                netlink::connected() ? "verbunden" : "getrennt", netlink::rssi(),
                (int)WiFi.status(), netlink::lastHttpCode());
  Serial.println("Slot | laedt | Fehler | Vbat");
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    const SlotState& x = s.m.slots[i];
    Serial.printf("%4u | %5s | %6s | %.3f V\n", i + 1, x.charging ? "ja" : "-", x.fault ? "JA" : "-", x.vbat);
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
    if (line.length()) Serial.printf("> Befehl empfangen: '%s'\n", line.c_str());
    if (line == "status" || line == "s") printStatus();
    else if (line == "mute") signals::mute(true);
    else if (line == "unmute") signals::mute(false);
    else if (line == "wake") wake::start();
    else if (line.startsWith("wake ")) {
      int n = line.substring(5).toInt();
      if (n >= 1 && n <= cfg::SLOT_COUNT) wake::start(n - 1);
      else Serial.println("wake <1..10>");
    }
    else if (line == "wakeall on") wake::setAll(true);
    else if (line == "wakeall off") wake::setAll(false);
    else if (line == "pintest") wake::startPinTest();
    else if (line == "wakepol low") wake::setActiveLow(true);
    else if (line == "wakepol high") wake::setActiveLow(false);
    else if (line == "stat") board::statDump();
    else if (line == "i2c") board::i2cScan();
    else if (line == "debug on") { board::debugOn = true; Serial.println("Debug an"); }
    else if (line == "debug off") { board::debugOn = false; Serial.println("Debug aus"); }
    else if (line.length()) Serial.println("Befehle: status | mute | unmute | wake [1..10] | wakepol low|high | wakeall on|off | pintest | stat | i2c | debug on|off");
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  board::begin();
  board::ledTest();
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
  if (netlink::takeWakeConfig(wakeCfg)) {
    wake::configure(wakeCfg);
    DBG("Wake-Konfig vom Pi: %u ms, Abstand %u ms, Pins %u,%u,%u,%u,%u,%u,%u,%u,%u,%u", wakeCfg.pulseMs, wakeCfg.gapMs,
        wakeCfg.pins[0], wakeCfg.pins[1], wakeCfg.pins[2], wakeCfg.pins[3], wakeCfg.pins[4], wakeCfg.pins[5],
        wakeCfg.pins[6], wakeCfg.pins[7], wakeCfg.pins[8], wakeCfg.pins[9]);
  }
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
  wake::setHold(netlink::otaHoldActive());
  wake::update();
  const String completedWakeId = wake::takeCompletedRequestId();
  if (completedWakeId.length()) netlink::acknowledgeManualWake(completedWakeId);
  netlink::publish(*current, wake::active());
  handleSerial();
}
