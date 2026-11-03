#include "wake.h"

#include <Adafruit_MCP23X17.h>
#include <Wire.h>

namespace wake {
namespace {

Adafruit_MCP23X17 mcp;
bool mcpUp = false;
uint32_t lastTryMs = 0;

Config cfgNow = defaults();
Config run = defaults();
bool running = false;
uint32_t startMs = 0;
bool pinOn[cfg::SLOT_COUNT] = {};
int8_t onlySlot = -1;
String requestId;
String completedRequestId;

void drivePin(uint8_t slot, bool on) {
  if (!mcpUp) return;
  const bool level = cfg::Wake::ACTIVE_LOW ? !on : on;
  mcp.digitalWrite(run.pins[slot], level ? HIGH : LOW);
}

bool initMcp() {
  if (!mcp.begin_I2C(cfg::Wake::MCP_ADDR, &Wire)) return false;
  // Erst Ruhepegel schreiben, dann auf Ausgang schalten (kein Impuls beim Start)
  for (uint8_t p = 0; p < 16; p++) {
    mcp.digitalWrite(p, cfg::Wake::ACTIVE_LOW ? HIGH : LOW);
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

void configure(const Config& c) { cfgNow = c; }

void begin() { mcpUp = initMcp(); }

void start() {
  if (running) return;
  if (!mcpUp) mcpUp = initMcp();
  if (!mcpUp) {
    Serial.println("Aufwecken: MCP23017 U5 (0x21) antwortet nicht");
    return;
  }
  run = cfgNow;
  onlySlot = -1;
  requestId = "";
  startMs = millis();
  running = true;
  Serial.println("Aufwecken: Taster werden der Reihe nach geweckt");
}

bool startSlot(uint8_t slot, const String& id) {
  if (running || slot < 1 || slot > cfg::SLOT_COUNT || !id.length()) return false;
  if (!mcpUp) mcpUp = initMcp();
  if (!mcpUp) {
    Serial.println("Einzel-Aufwecken: MCP23017 U5 (0x21) antwortet nicht");
    return false;
  }
  run = cfgNow;
  onlySlot = slot - 1;
  requestId = id;
  startMs = millis();
  running = true;
  Serial.printf("Aufwecken: einzelner Taster in Slot %u\n", slot);
  return true;
}

bool active() { return running; }

String takeCompletedRequestId() {
  String id = completedRequestId;
  completedRequestId = "";
  return id;
}

void update() {
  const uint32_t now = millis();
  if (!running) {
    if (!mcpUp && now - lastTryMs >= cfg::MCP_RETRY_MS) {
      lastTryMs = now;
      mcpUp = initMcp();
    }
    return;
  }
  const uint32_t t = now - startMs;
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    const uint32_t on = (uint32_t)i * run.gapMs;
    const bool want = onlySlot >= 0
      ? i == onlySlot && t < run.pulseMs
      : t >= on && t < on + run.pulseMs;
    if (want != pinOn[i]) {
      pinOn[i] = want;
      drivePin(i, want);
    }
  }
  const uint32_t duration = onlySlot >= 0
    ? run.pulseMs
    : (uint32_t)(cfg::SLOT_COUNT - 1) * run.gapMs + run.pulseMs;
  if (t >= duration) {
    allOff();
    running = false;
    onlySlot = -1;
    if (requestId.length()) {
      completedRequestId = requestId;
      requestId = "";
    }
    Serial.println("Aufwecken: fertig");
  }
}

}  // namespace wake
