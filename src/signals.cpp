#include "signals.h"

#include "board.h"
#include "wake.h"

namespace signals {
namespace {

Status st;
bool lastAnyFault = false;

// Entprellter Taster: kurzer Druck (Loslassen) quittiert den Alarm, langes Halten weckt die Taster
bool rawLast = false;
bool stable = false;
bool stableLast = false;
bool longFired = false;
uint32_t lastChangeMs = 0;
uint32_t pressedSinceMs = 0;

void updateButton(uint32_t now) {
  bool raw = board::ackPressedRaw();
  if (raw != rawLast) {
    rawLast = raw;
    lastChangeMs = now;
  }
  if (now - lastChangeMs >= cfg::DEBOUNCE_MS) stable = raw;
  if (stable && !stableLast) {
    pressedSinceMs = now;
    longFired = false;
  }
  if (stable && !longFired && now - pressedSinceMs >= cfg::LONG_PRESS_MS) {
    longFired = true;
    wake::start();
  }
  if (!stable && stableLast && !longFired && st.anyFault) st.muted = true;
  stableLast = stable;
}

void aggregate(const Measurements& m) {
  st.m = m;
  st.anyCharging = false;
  st.allPresent = true;
  st.allFull = true;
  bool slotFault = false;
  for (const SlotState& s : m.slots) {
    st.anyCharging |= s.charging;
    st.allPresent &= s.present;
    st.allFull &= s.full;
    slotFault |= s.fault;
  }
  st.sensorFault = !m.ahtOk;
  // Roter Dauerfehler: Temperatur, Slot oder ausgefallener I2C-Baustein
  st.hardFault = m.tempAlarm || slotFault || !m.i2cOk;
  st.anyFault = st.sensorFault || st.hardFault;

  if (!st.anyFault) st.muted = false;
  if (st.anyFault && !lastAnyFault) st.muted = false;  // neuer Fehler -> Ton wieder aktiv
  lastAnyFault = st.anyFault;
}

void drive(uint32_t now) {
  const bool blink = ((now / cfg::BLINK_MS) % 2) == 0;

  if (st.hardFault) {
    st.light = Light::RED;
    board::setLeds(true, false, false);
  } else if (st.sensorFault) {
    st.light = Light::RED_BLINK;
    board::setLeds(blink, false, false);
  } else if (st.anyCharging) {
    st.light = st.allPresent ? Light::YELLOW : Light::YELLOW_BLINK;
    board::setLeds(false, st.allPresent ? true : blink, false);
  } else if (st.allPresent && st.allFull) {
    st.light = Light::GREEN;
    board::setLeds(false, false, true);
  } else {
    st.light = Light::OFF;
    board::setLeds(false, false, false);
  }

  static uint32_t lastToggle = 0;
  static bool buzzer = false;
  if (st.anyFault && !st.muted) {
    uint32_t onMs, offMs;
    if (st.m.tempAlarm) { onMs = 200; offMs = 800; }
    else if (st.sensorFault) { onMs = 80; offMs = 920; }
    else { onMs = 100; offMs = 1900; }
    if (now - lastToggle >= (buzzer ? onMs : offMs)) {
      lastToggle = now;
      buzzer = !buzzer;
      board::setBuzzer(buzzer);
    }
  } else if (buzzer) {
    buzzer = false;
    board::setBuzzer(false);
  }
}

}  // namespace

void begin() {}

const Status& update(const Measurements& m) {
  uint32_t now = millis();
  aggregate(m);
  updateButton(now);
  drive(now);
  return st;
}

void mute(bool on) { st.muted = on && st.anyFault; }

}  // namespace signals
