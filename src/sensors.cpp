#include "sensors.h"

#include <Wire.h>

#include "board.h"

namespace sensors {
namespace {

Measurements data;
uint32_t lastScanMs = 0;

// ---- AHT20 als Zustandsautomat: Messung anstossen, 90 ms spaeter lesen ----
enum class Aht : uint8_t { IDLE, CONVERTING };
Aht ahtState = Aht::IDLE;
uint32_t ahtStartMs = 0;
uint32_t ahtLastGoodMs = 0;
bool ahtInitDone = false;
float lastTemp = NAN;
float lastHum = NAN;

bool ahtInit() {
  Wire.beginTransmission(cfg::AHT_ADDR);
  Wire.write(0x71);  // Status lesen
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((int)cfg::AHT_ADDR, 1) != 1) return false;
  uint8_t status = Wire.read();
  if (status & 0x08) return true;  // bereits kalibriert
  Wire.beginTransmission(cfg::AHT_ADDR);
  Wire.write(0xBE);
  Wire.write(0x08);
  Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

bool ahtTrigger() {
  Wire.beginTransmission(cfg::AHT_ADDR);
  Wire.write(0xAC);
  Wire.write(0x33);
  Wire.write(0x00);
  return Wire.endTransmission() == 0;
}

bool ahtRead() {
  if (Wire.requestFrom((int)cfg::AHT_ADDR, 6) != 6) return false;
  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();
  if (b[0] & 0x80) return false;  // noch beschaeftigt
  uint32_t rawH = ((uint32_t)b[1] << 12) | ((uint32_t)b[2] << 4) | (b[3] >> 4);
  uint32_t rawT = ((uint32_t)(b[3] & 0x0F) << 16) | ((uint32_t)b[4] << 8) | b[5];
  lastHum = (float)rawH * 100.0f / 1048576.0f;
  lastTemp = (float)rawT * 200.0f / 1048576.0f - 50.0f;
  return true;
}

void updateTemperature(uint32_t now) {
  static uint32_t lastStartMs = 0;
  if (ahtState == Aht::IDLE) {
    if (now - lastStartMs < cfg::TEMP_INTERVAL_MS) return;
    lastStartMs = now;
    if (!ahtInitDone) ahtInitDone = ahtInit();
    if (ahtInitDone && ahtTrigger()) {
      ahtState = Aht::CONVERTING;
      ahtStartMs = now;
    } else {
      ahtInitDone = false;
    }
  } else if (now - ahtStartMs >= cfg::AHT_CONVERT_MS) {
    ahtState = Aht::IDLE;
    if (ahtRead()) {
      ahtLastGoodMs = now;
    } else {
      ahtInitDone = false;
      DBG("AHT20: Lesen fehlgeschlagen (Busy oder keine Antwort), neuer Versuch in %lu s",
          (unsigned long)(cfg::TEMP_INTERVAL_MS / 1000));
    }
  }

  const bool wasOk = data.ahtOk;
  data.ahtOk = ahtLastGoodMs != 0 && (now - ahtLastGoodMs) <= cfg::AHT_VALID_MS;
  if (wasOk != data.ahtOk) DBG("AHT20: %s", data.ahtOk ? "ok" : "ausgefallen (keine gueltige Messung)");
  data.tempC = data.ahtOk ? lastTemp : NAN;
  data.humidity = data.ahtOk ? lastHum : NAN;
  if (data.ahtOk) {
    if (!data.tempAlarm && data.tempC >= cfg::TEMP_ALARM_ON_C) data.tempAlarm = true;
    if (data.tempAlarm && data.tempC <= cfg::TEMP_ALARM_OFF_C) data.tempAlarm = false;
  }
}

void scanSlots() {
  static bool prevCharging[cfg::SLOT_COUNT] = {};
  static bool seen[cfg::SLOT_COUNT] = {};
  static uint32_t changedAt[cfg::SLOT_COUNT] = {};
  const uint32_t now = millis();
  for (uint8_t i = 0; i < cfg::SLOT_COUNT; i++) {
    SlotState& s = data.slots[i];
    s.vbat = board::readVbat(i);
    // Sicher messbar: rote LED des Lademoduls (laedt) und die Spannung am Ausgang
    s.charging = board::readCharging(i);
    s.fault = false;
    if (!seen[i]) {
      seen[i] = true;
      prevCharging[i] = s.charging;
    } else if (s.charging != prevCharging[i]) {
      DBG("Slot %u: laedt=%d Vbat=%.3f V", i + 1, s.charging, s.vbat);
      prevCharging[i] = s.charging;
      changedAt[i] = now;
      s.changed = true;
    }
    s.sinceS = s.changed ? (now - changedAt[i]) / 1000 : 0;
  }
}

}  // namespace

void begin() { update(); }

void update() {
  uint32_t now = millis();
  if (now - lastScanMs >= cfg::SCAN_SLOTS_MS || lastScanMs == 0) {
    lastScanMs = now ? now : 1;
    scanSlots();
  }
  updateTemperature(now);
  data.i2cOk = board::i2cOk();
}

const Measurements& get() { return data; }

}  // namespace sensors
