#include "board.h"

#include <Adafruit_MCP23X17.h>
#include <Wire.h>

#include "config.h"

namespace board {

bool debugOn = true;

namespace {

Adafruit_MCP23X17 mcp;
bool mcpUp = false;
uint32_t lastTryMs = 0;

bool probeMcp() {
  Wire.beginTransmission(cfg::Mcp::ADDR);
  return Wire.endTransmission() == 0;
}

bool initMcp() {
  if (!mcp.begin_I2C(cfg::Mcp::ADDR, &Wire)) {
    DBG("MCP U2 (0x%02X) Init fehlgeschlagen", cfg::Mcp::ADDR);
    return false;
  }
  DBG("MCP U2 (0x%02X) Init ok", cfg::Mcp::ADDR);
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    mcp.pinMode(cfg::Mcp::STAT[i], INPUT_PULLUP);
  }
  const uint8_t outs[] = {cfg::Mcp::BUZZ_EN, cfg::Mcp::LED_RED_DRV, cfg::Mcp::LED_YEL_DRV,
                          cfg::Mcp::LED_GRN_DRV};
  for (uint8_t p : outs) {
    mcp.pinMode(p, OUTPUT);
    mcp.digitalWrite(p, cfg::Mcp::LED_ACTIVE_LOW && p != cfg::Mcp::BUZZ_EN ? HIGH : LOW);
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
    if (!probeMcp()) {
      mcpUp = false;
      DBG("MCP U2 antwortet nicht mehr");
    } else {
      return;
    }
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

void statDump() {
  if (!mcpUp) {
    Serial.println("MCP U2 antwortet nicht");
    return;
  }
  const uint16_t v = mcp.readGPIOAB();
  Serial.printf("U2 Eingaenge roh: 0x%04X\n", v);
  for (uint8_t p = 0; p < 16; p++) Serial.printf("  Pin %2u (%s%u): %s\n", p, p < 8 ? "GPA" : "GPB", p & 7, (v >> p) & 1 ? "HIGH" : "LOW");
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++)
    Serial.printf("  Slot %2u = Pin %2u: %s -> %s\n", i + 1, cfg::Mcp::STAT[i],
                  (v >> cfg::Mcp::STAT[i]) & 1 ? "HIGH" : "LOW",
                  ((((v >> cfg::Mcp::STAT[i]) & 1) != 0) == !cfg::STAT_ACTIVE_LOW) ? "laedt" : "laedt nicht");
}

bool readStatRaw(uint8_t slot) {
  if (!mcpUp || slot >= cfg::SLOT_COUNT) return false;
  return mcp.digitalRead(cfg::Mcp::STAT[slot]) == HIGH;
}

void i2cScan() {
  Serial.println("I2C-Scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      const char* name = a == cfg::Mcp::ADDR ? " (MCP U2)" : a == 0x21 ? " (MCP U5)" : a == cfg::AHT_ADDR ? " (AHT20)" : "";
      Serial.printf("  0x%02X%s\n", a, name);
      found++;
    }
  }
  Serial.printf("  %u Geraet(e) gefunden\n", found);
}

bool readCharging(uint8_t slot) {
  if (!mcpUp || slot >= cfg::SLOT_COUNT) return false;
  bool low = mcp.digitalRead(cfg::Mcp::STAT[slot]) == LOW;
  return cfg::STAT_ACTIVE_LOW ? low : !low;
}

void setLeds(bool red, bool yellow, bool green) {
  const auto level = [](bool on) { return (on != cfg::Mcp::LED_ACTIVE_LOW) ? HIGH : LOW; };
  digitalWrite(cfg::Pin::LED_RED, level(red));
  digitalWrite(cfg::Pin::LED_YEL, level(yellow));
  digitalWrite(cfg::Pin::LED_GRN, level(green));
  if (cfg::Mcp::MIRROR_LEDS && mcpUp) {
    mcp.digitalWrite(cfg::Mcp::LED_RED_DRV, level(red));
    mcp.digitalWrite(cfg::Mcp::LED_YEL_DRV, level(yellow));
    mcp.digitalWrite(cfg::Mcp::LED_GRN_DRV, level(green));
  }
}

void ledTest() {
  DBG("LED-Test");
  const bool seq[4][3] = {{true, false, false}, {false, true, false}, {false, false, true}, {true, true, true}};
  for (const auto& s : seq) {
    setLeds(s[0], s[1], s[2]);
    delay(500);
  }
  setLeds(false, false, false);
}

void setBuzzer(bool on) {
  if (mcpUp) mcp.digitalWrite(cfg::Mcp::BUZZ_EN, on ? HIGH : LOW);
}

bool ackPressedRaw() {
  int v = digitalRead(cfg::Pin::TRIG_ALL);
  return cfg::BTN_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

}  // namespace board
