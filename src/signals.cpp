#include "signals.h"

#include "board.h"
#include "netlink.h"
#include "remotebeep.h"
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
bool sleepUnassignedFired = false;
uint32_t lastChangeMs = 0;
uint32_t pressedSinceMs = 0;

void updateButton(uint32_t now) {
  bool raw = board::ackPressedRaw();
  if (raw != rawLast) {
    rawLast = raw;
    lastChangeMs = now;
    DBG("TRIG_ALL roh: %s", raw ? "gedrueckt (LOW)" : "losgelassen (HIGH)");
  }
  if (now - lastChangeMs >= cfg::DEBOUNCE_MS) stable = raw;
  if (stable && !stableLast) {
    pressedSinceMs = now;
    longFired = false;
    sleepUnassignedFired = false;
    DBG("TRIG_ALL entprellt: gedrueckt");
  }
  if (stable && !sleepUnassignedFired && now - pressedSinceMs >= cfg::SLEEP_UNASSIGNED_PRESS_MS) {
    sleepUnassignedFired = true;
    longFired = true;
    DBG("TRIG_ALL 15 s gehalten -> nicht zugeordnete Taster ausschalten");
    netlink::requestSleepUnassigned();
  }
  if (!stable && stableLast) DBG("TRIG_ALL entprellt: losgelassen (lang=%d)", longFired);
  if (!stable && stableLast && !longFired) {
    if (now - pressedSinceMs >= cfg::LONG_PRESS_MS) {
      DBG("TRIG_ALL lang gehalten -> Aufwecken");
      wake::start();
      longFired = true;
    } else {
      if (st.anyFault) st.muted = true;
      if (remotebeep::active(now) || st.anyFault) remotebeep::mute();
      DBG("Quittiert: Pieper stumm bis zur naechsten Meldung");
    }
  }
  stableLast = stable;
}

void aggregate(const Measurements& m) {
  st.m = m;
  st.anyCharging = false;
  bool slotFault = false;
  for (const SlotState& s : m.slots) {
    st.anyCharging |= s.charging;
    slotFault |= s.fault;
  }
  st.sensorFault = !m.ahtOk;
  // Roter Dauerfehler: Temperatur, Slot oder ausgefallener I2C-Baustein
  st.hardFault = m.tempAlarm || slotFault || !m.i2cOk;
  st.anyFault = st.sensorFault || st.hardFault;

  static bool pT = false, pS = false, pI = false, pA = false, pM = false;
  if (m.tempAlarm != pT || slotFault != pS || m.i2cOk != pI || m.ahtOk != pA || st.muted != pM) {
    DBG("Fehler: Temp-Alarm=%d Slot-Ueberspannung=%d I2C-Ausfall=%d AHT-ok=%d quittiert=%d", m.tempAlarm, slotFault,
        !m.i2cOk, m.ahtOk, st.muted);
    pT = m.tempAlarm; pS = slotFault; pI = m.i2cOk; pA = m.ahtOk; pM = st.muted;
  }
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
    st.light = Light::YELLOW;
    board::setLeds(false, true, false);
  } else if (st.m.i2cOk) {
    st.light = Light::GREEN;
    board::setLeds(false, false, true);
  } else {
    st.light = Light::OFF;
    board::setLeds(false, false, false);
  }

  static uint32_t lastToggle = 0;
  static bool buzzer = false;
  static bool buzzerOut = false;
  if (st.anyFault && !st.muted) {
    uint32_t onMs, offMs;
    if (st.m.tempAlarm) { onMs = 200; offMs = 800; }
    else if (st.sensorFault) { onMs = 80; offMs = 920; }
    else { onMs = 100; offMs = 1900; }
    if (now - lastToggle >= (buzzer ? onMs : offMs)) {
      lastToggle = now;
      buzzer = !buzzer;
    }
  } else {
    buzzer = false;
  }
  // Eigener Fehlerton oder Ton fuer eine Dashboard-Meldung
  const bool wantBuzzer = buzzer || remotebeep::active(now);
  if (wantBuzzer != buzzerOut) {
    buzzerOut = wantBuzzer;
    board::setBuzzer(wantBuzzer);
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
