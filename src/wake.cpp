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
  startMs = millis();
  running = true;
  Serial.println("Aufwecken: Taster werden der Reihe nach geweckt");
}

bool active() { return running; }

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
    const bool want = t >= on && t < on + run.pulseMs;
    if (want != pinOn[i]) {
      pinOn[i] = want;
      drivePin(i, want);
    }
  }
  if (t >= (uint32_t)(cfg::SLOT_COUNT - 1) * run.gapMs + run.pulseMs) {
    allOff();
    running = false;
    Serial.println("Aufwecken: fertig");
  }
}

}  // namespace wake
