#include "board.h"

#include <Adafruit_MCP23X17.h>
#include <Wire.h>

#include "config.h"

namespace board {
namespace {

Adafruit_MCP23X17 mcp;
bool mcpUp = false;
uint32_t lastTryMs = 0;

bool probeMcp() {
  Wire.beginTransmission(cfg::Mcp::ADDR);
  return Wire.endTransmission() == 0;
}

bool initMcp() {
  if (!mcp.begin_I2C(cfg::Mcp::ADDR, &Wire)) return false;
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    mcp.pinMode(cfg::Mcp::STAT[i], INPUT_PULLUP);
  }
  const uint8_t outs[] = {cfg::Mcp::BUZZ_EN, cfg::Mcp::LED_RED_DRV, cfg::Mcp::LED_YEL_DRV,
                          cfg::Mcp::LED_GRN_DRV};
  for (uint8_t p : outs) {
    mcp.pinMode(p, OUTPUT);
    mcp.digitalWrite(p, LOW);
  }
  return true;
}

void setMuxChannel(uint8_t ch) {
  digitalWrite(cfg::Pin::MUX_A, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(cfg::Pin::MUX_B, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(cfg::Pin::MUX_C, (ch & 0x04) ? HIGH : LOW);
  delayMicroseconds(60);  // Einschwingzeit des Multiplexers
}

}  // namespace

void begin() {
  pinMode(cfg::Pin::LED_RED, OUTPUT);
  pinMode(cfg::Pin::LED_YEL, OUTPUT);
  pinMode(cfg::Pin::LED_GRN, OUTPUT);
  setLeds(false, false, false);

  pinMode(cfg::Pin::MUX_A, OUTPUT);
  pinMode(cfg::Pin::MUX_B, OUTPUT);
  pinMode(cfg::Pin::MUX_C, OUTPUT);
  setMuxChannel(0);
  analogReadResolution(12);

  pinMode(cfg::Pin::TRIG_ALL, INPUT_PULLUP);

  Wire.begin(cfg::Pin::SDA, cfg::Pin::SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  mcpUp = initMcp();
}

void service() {
  uint32_t now = millis();
  if (mcpUp) {
    if (!probeMcp()) mcpUp = false;
    else return;
  }
  if (now - lastTryMs < cfg::MCP_RETRY_MS) return;
  lastTryMs = now;
  mcpUp = initMcp();
}

bool i2cOk() { return mcpUp; }

float readVbat(uint8_t slot) {
  const bool second = slot >= 8;
  const int pin = second ? cfg::Pin::ADC_MUX_B : cfg::Pin::ADC_MUX_A;
  setMuxChannel(second ? slot - 8 : slot);
  analogReadMilliVolts(pin);  // erster Wert nach dem Umschalten wird verworfen
  uint32_t sum = 0;
  for (uint8_t i = 0; i < cfg::ADC_SAMPLES; i++) sum += analogReadMilliVolts(pin);
  return (float)sum / cfg::ADC_SAMPLES / 1000.0f * cfg::VBAT_DIV_RATIO;
}

bool readCharging(uint8_t slot) {
  if (!mcpUp || slot >= cfg::SLOT_COUNT) return false;
  bool low = mcp.digitalRead(cfg::Mcp::STAT[slot]) == LOW;
  return cfg::STAT_ACTIVE_LOW ? low : !low;
}

void setLeds(bool red, bool yellow, bool green) {
  digitalWrite(cfg::Pin::LED_RED, red ? HIGH : LOW);
  digitalWrite(cfg::Pin::LED_YEL, yellow ? HIGH : LOW);
  digitalWrite(cfg::Pin::LED_GRN, green ? HIGH : LOW);
  if (cfg::Mcp::MIRROR_LEDS && mcpUp) {
    mcp.digitalWrite(cfg::Mcp::LED_RED_DRV, red ? HIGH : LOW);
    mcp.digitalWrite(cfg::Mcp::LED_YEL_DRV, yellow ? HIGH : LOW);
    mcp.digitalWrite(cfg::Mcp::LED_GRN_DRV, green ? HIGH : LOW);
  }
}

void setBuzzer(bool on) {
  if (mcpUp) mcp.digitalWrite(cfg::Mcp::BUZZ_EN, on ? HIGH : LOW);
}

bool ackPressedRaw() {
  int v = digitalRead(cfg::Pin::TRIG_ALL);
  return cfg::BTN_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

}  // namespace board
