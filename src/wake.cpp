#include "wake.h"

#include <Adafruit_MCP23X17.h>
#include <Preferences.h>
#include <Wire.h>

#include "board.h"

namespace wake {
namespace {

Adafruit_MCP23X17 mcp;
bool mcpUp = false;
uint32_t lastTryMs = 0;

Config cfgNow = defaults();
Config run = defaults();
bool running = false;
// Pin-Test: jeden der 16 Pins von U5 einzeln ansteuern, um die Zuordnung zu den Slots zu finden
bool pinTest = false;
uint8_t pinTestPin = 0;
uint32_t pinTestMs = 0;
constexpr uint32_t PINTEST_PULSE_MS = 3000;
constexpr uint32_t PINTEST_GAP_MS = 2000;
int8_t only = -1;
uint32_t startMs = 0;
bool pinOn[cfg::SLOT_COUNT] = {};
String requestId;
String completedRequestId;
bool activeLow = cfg::Wake::ACTIVE_LOW;
bool holdWanted = false;
bool holdApplied = false;

void drivePin(uint8_t slot, bool on) {
  if (!mcpUp) return;
  const bool level = activeLow ? !on : on;
  mcp.digitalWrite(run.pins[slot], level ? HIGH : LOW);
  DBG("Wake Slot %u: MCP-Pin %u -> %s (Ausgang %s, Readback %s)", slot + 1, run.pins[slot], on ? "AKTIV" : "ruhe",
      level ? "HIGH" : "LOW", mcp.digitalRead(run.pins[slot]) ? "HIGH" : "LOW");
}

bool initMcp() {
  if (!mcp.begin_I2C(cfg::Wake::MCP_ADDR, &Wire)) {
    DBG("MCP U5 (0x%02X) Init fehlgeschlagen", cfg::Wake::MCP_ADDR);
    return false;
  }
  DBG("MCP U5 (0x%02X) Init ok, alle 16 Pins auf Ruhepegel", cfg::Wake::MCP_ADDR);
  // Erst Ruhepegel schreiben, dann auf Ausgang schalten (kein Impuls beim Start)
  for (uint8_t p = 0; p < 16; p++) {
    mcp.digitalWrite(p, activeLow ? HIGH : LOW);
    mcp.pinMode(p, OUTPUT);
  }
  return true;
}

void allOff() {
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    if (pinOn[i]) drivePin(i, false);
    pinOn[i] = false;
  }
}

}  // namespace

Config defaults() {
  Config c;
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) c.pins[i] = cfg::Wake::DEFAULT_PINS[i];
  c.pulseMs = cfg::Wake::DEFAULT_PULSE_MS;
  c.gapMs = cfg::Wake::DEFAULT_GAP_MS;
  return c;
}

// Die Pins sind durch die Platine fest vorgegeben (DEFAULT_PINS); Pins vom Pi werden ignoriert
void configure(const Config& c) {
  cfgNow = c;
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) cfgNow.pins[i] = cfg::Wake::DEFAULT_PINS[i];
}

void setActiveLow(bool low) {
  activeLow = low;
  Preferences p;
  if (p.begin("wake", false)) {
    p.putBool("pol2", low);
    p.end();
  }
  if (mcpUp) {
    for (uint8_t i = 0; i < 16; i++) mcp.digitalWrite(i, activeLow ? HIGH : LOW);
  }
  Serial.printf("Wake-Polaritaet (gespeichert): Trigger aktiv bei %s, Ruhepegel %s\n", activeLow ? "LOW" : "HIGH",
                activeLow ? "HIGH" : "LOW");
}

void setAll(bool on) {
  if (running || pinTest) {
    Serial.println("Aufwecken laeuft noch, bitte warten");
    return;
  }
  if (!mcpUp) mcpUp = initMcp();
  if (!mcpUp) {
    Serial.println("MCP23017 U5 (0x21) antwortet nicht");
    return;
  }
  const bool level = activeLow ? !on : on;
  for (uint8_t i = 0; i < 16; i++) mcp.digitalWrite(i, level ? HIGH : LOW);
  Serial.printf("Alle 16 Trigger-Ausgaenge: %s (Pegel %s)\n", on ? "AKTIV" : "ruhe", level ? "HIGH" : "LOW");
}

void begin() {
  Preferences p;
  if (p.begin("wake", true)) {
    activeLow = p.getBool("pol2", cfg::Wake::ACTIVE_LOW);
    p.end();
  }
  DBG("Wake-Polaritaet: aktiv bei %s", activeLow ? "LOW" : "HIGH");
  mcpUp = initMcp();
}

void start(int8_t onlySlot) {
  if (running) return;
  if (!mcpUp) mcpUp = initMcp();
  if (!mcpUp) {
    Serial.println("Aufwecken: MCP23017 U5 (0x21) antwortet nicht");
    return;
  }
  run = cfgNow;
  only = onlySlot;
  requestId = "";
  startMs = millis();
  running = true;
  if (only >= 0) Serial.printf("Aufwecken: nur Slot %d (MCP-Pin %u, %u ms)\n", only + 1, run.pins[only], run.pulseMs);
  else Serial.println("Aufwecken: Taster werden der Reihe nach geweckt");
}

bool startSlot(uint8_t slot, const String& id) {
  if (running || slot < 1 || slot > cfg::SLOT_COUNT || !id.length()) return false;
  if (!mcpUp) mcpUp = initMcp();
  if (!mcpUp) {
    Serial.println("Einzel-Aufwecken: MCP23017 U5 (0x21) antwortet nicht");
    return false;
  }
  run = cfgNow;
  only = slot - 1;
  requestId = id;
  startMs = millis();
  running = true;
  Serial.printf("Aufwecken: einzelner Taster in Slot %u\n", slot);
  return true;
}

bool active() { return running || pinTest; }

void setHold(bool on) { holdWanted = on; }

void startPinTest() {
  if (running || pinTest) return;
  if (!mcpUp) mcpUp = initMcp();
  if (!mcpUp) {
    Serial.println("Pin-Test: MCP23017 U5 (0x21) antwortet nicht");
    return;
  }
  pinTest = true;
  pinTestPin = 0;
  pinTestMs = millis();
  Serial.println("Pin-Test: jeder Pin 3 s aktiv, danach 2 s Pause. Notieren Sie, welcher Taster bei welchem Pin startet.");
}

String takeCompletedRequestId() {
  String id = completedRequestId;
  completedRequestId = "";
  return id;
}

void update() {
  const uint32_t now = millis();
  if (running || pinTest) {
    holdApplied = false;  // Pins werden vom Ablauf veraendert, danach neu setzen
  } else if (mcpUp && holdWanted != holdApplied) {
    const bool level = activeLow ? !holdWanted : holdWanted;
    for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) mcp.digitalWrite(cfg::Wake::DEFAULT_PINS[i], level ? HIGH : LOW);
    holdApplied = holdWanted;
    Serial.printf("Taster-Update: Startkontakte %s\n", holdWanted ? "EIN" : "aus");
  }
  if (pinTest) {
    const uint32_t t = now - pinTestMs;
    static bool on = false;
    const bool want = t < PINTEST_PULSE_MS;
    if (want != on) {
      on = want;
      mcp.digitalWrite(pinTestPin, (activeLow ? !on : on) ? HIGH : LOW);
      if (on) Serial.printf("Pin-Test: MCP-Pin %u AKTIV (%s)\n", pinTestPin, pinTestPin < 8 ? "GPA" : "GPB");
      else DBG("Pin-Test: MCP-Pin %u aus", pinTestPin);
    }
    if (t >= PINTEST_PULSE_MS + PINTEST_GAP_MS) {
      pinTestPin++;
      pinTestMs = now;
      if (pinTestPin >= 16) {
        pinTest = false;
        Serial.println("Pin-Test: fertig");
      }
    }
    return;
  }
  if (!running) {
    if (!mcpUp && now - lastTryMs >= cfg::MCP_RETRY_MS) {
      lastTryMs = now;
      mcpUp = initMcp();
    }
    return;
  }
  const uint32_t t = now - startMs;
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    if (only >= 0 && i != only) continue;
    const uint32_t on = (only >= 0 ? 0 : (uint32_t)i) * run.gapMs;
    const bool want = t >= on && t < on + run.pulseMs;
    if (want != pinOn[i]) {
      pinOn[i] = want;
      drivePin(i, want);
    }
  }
  const uint32_t total = (only >= 0 ? 0 : (uint32_t)(cfg::SLOT_COUNT - 1) * run.gapMs) + run.pulseMs;
  if (t >= total) {
    allOff();
    running = false;
    only = -1;
    if (requestId.length()) {
      completedRequestId = requestId;
      requestId = "";
    }
    Serial.println("Aufwecken: fertig");
  }
}

}  // namespace wake
