#include "signals.h"

#include "board.h"
#include "netlink.h"
#include "remotebeep.h"
#include "wake.h"

namespace signals {
namespace {

Status st;
uint8_t acknowledgedFaults = 0;
uint8_t remoteMask = 0;
uint8_t activeFaults = 0;

// Entprellter Taster: kurzer Druck (Loslassen) quittiert den Alarm, langes Halten weckt die Taster
bool rawLast = false;
bool stable = false;
bool stableLast = false;
bool longFired = false;
bool sleepUnassignedFired = false;
uint32_t lastChangeMs = 0;
uint32_t pressedSinceMs = 0;
uint32_t sleepFeedbackStartedMs = 0;
bool sleepFeedbackActive = false;
uint32_t startupStartedMs = 0;
bool startupPending = false;
bool startupActive = false;

constexpr uint32_t STARTUP_WAIT_MS = 1000;
constexpr uint32_t STARTUP_BEEP_MS = 100;
constexpr uint32_t SLEEP_WARNING_BEEP_MS = 100;
constexpr uint32_t SLEEP_FEEDBACK_MS = 300;

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
  if (raw && stable && !sleepUnassignedFired &&
      now - pressedSinceMs >= cfg::SLEEP_UNASSIGNED_PRESS_MS) {
    sleepUnassignedFired = true;
    longFired = true;
    sleepFeedbackStartedMs = now;
    sleepFeedbackActive = true;
    DBG("TRIG_ALL 5 s gehalten -> nicht zugeordnete Taster ausschalten");
    netlink::requestSleepUnassigned();
  }
  if (!stable && stableLast) DBG("TRIG_ALL entprellt: losgelassen (lang=%d)", longFired);
  if (!stable && stableLast && !longFired) {
    if (now - pressedSinceMs >= cfg::LONG_PRESS_MS) {
      DBG("TRIG_ALL lang gehalten -> Aufwecken");
      wake::start();
      longFired = true;
    } else {
      acknowledgedFaults |= activeFaults;
      remotebeep::mute();
      DBG("Quittiert: Pieper stumm bis zur naechsten Meldung");
    }
  }
  stableLast = stable;
}

void aggregate(const Measurements& m) {
  st.m = m;
  st.anyCharging = false;
  for (const SlotState& s : m.slots) {
    st.anyCharging |= s.charging;
  }
  st.sensorFault = !m.ahtOk;
  st.hardFault = m.tempAlarm || !m.i2cOk;
  st.anyFault = st.sensorFault || st.hardFault;
  activeFaults = (m.tempAlarm ? 1 : 0) | (!m.i2cOk ? 2 : 0) | (!m.ahtOk ? 4 : 0);
  acknowledgedFaults &= activeFaults;
  st.muted = st.anyFault && (activeFaults & ~acknowledgedFaults) == 0;

  static bool pT = false, pI = false, pA = false, pM = false;
  if (m.tempAlarm != pT || m.i2cOk != pI || m.ahtOk != pA || st.muted != pM) {
    DBG("Fehler: Temp-Alarm=%d I2C-Ausfall=%d AHT-ok=%d quittiert=%d", m.tempAlarm,
        !m.i2cOk, m.ahtOk, st.muted);
    pT = m.tempAlarm; pI = m.i2cOk; pA = m.ahtOk; pM = st.muted;
  }
}

void drive(uint32_t now) {
  const bool blink = ((now / cfg::BLINK_MS) % 2) == 0;
  const uint32_t heldMs = now - pressedSinceMs;

  if (st.hardFault) {
    st.light = Light::RED;
    board::setLeds(true, false, false);
  } else if (st.sensorFault) {
    st.light = Light::RED_BLINK;
    board::setLeds(blink, false, false);
  } else if (stable && !sleepUnassignedFired && heldMs >= cfg::SLEEP_WARNING_START_MS) {
    st.light = Light::YELLOW_BLINK;
    board::setLeds(false, blink, false);
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

  if (startupPending) {
    if (st.hardFault || now - startupStartedMs >= STARTUP_WAIT_MS) {
      startupPending = false;
    } else if (!st.anyFault) {
      startupPending = false;
      startupActive = true;
      startupStartedMs = now;
    }
  }
  if (startupActive && (st.anyFault || now - startupStartedMs >= 3 * STARTUP_BEEP_MS)) {
    startupActive = false;
  }
  if (sleepFeedbackActive && now - sleepFeedbackStartedMs >= SLEEP_FEEDBACK_MS) {
    sleepFeedbackActive = false;
  }
  const bool startupBuzzer = startupActive &&
      ((now - startupStartedMs) / STARTUP_BEEP_MS) % 2 == 0;
  const bool sleepWarningBuzzer = stable && !sleepUnassignedFired &&
      ((heldMs >= cfg::SLEEP_WARNING_START_MS &&
        heldMs - cfg::SLEEP_WARNING_START_MS < SLEEP_WARNING_BEEP_MS) ||
       (heldMs >= cfg::SLEEP_WARNING_START_MS + 1000 &&
        heldMs - cfg::SLEEP_WARNING_START_MS - 1000 < SLEEP_WARNING_BEEP_MS));

  static uint32_t lastToggle = 0;
  static bool buzzer = false;
  static bool buzzerOut = false;
  static uint8_t lastAudibleFaults = 0;
  // Die erste AHT-Messung darf anlaufen, bevor ein fehlender Messwert piept.
  const uint8_t audibleFaults = activeFaults & ~acknowledgedFaults & (startupPending ? ~4 : ~0);
  if (audibleFaults) {
    uint32_t onMs, offMs;
    if (audibleFaults & 1) { onMs = 200; offMs = 800; }
    else if (audibleFaults & 4) { onMs = 80; offMs = 920; }
    else { onMs = 100; offMs = 1900; }
    if (audibleFaults != lastAudibleFaults) {
      lastToggle = now;
      buzzer = true;
    } else if (now - lastToggle >= (buzzer ? onMs : offMs)) {
      lastToggle = now;
      buzzer = !buzzer;
    }
  } else {
    buzzer = false;
  }
  lastAudibleFaults = audibleFaults;
  const bool remoteBuzzer = remotebeep::active(now);
  const bool wantBuzzer = audibleFaults ? buzzer :
      (remoteBuzzer || sleepWarningBuzzer || sleepFeedbackActive || startupBuzzer);
  if (!st.m.i2cOk) buzzerOut = false;
  if (wantBuzzer != buzzerOut) {
    buzzerOut = wantBuzzer;
    board::setBuzzer(wantBuzzer);
  }
}

}  // namespace

void begin() {
  startupStartedMs = millis();
  startupPending = true;
}

const Status& update(const Measurements& m) {
  uint32_t now = millis();
  aggregate(m);
  updateButton(now);
  drive(now);
  return st;
}

void mute(bool on) { acknowledgedFaults = on ? activeFaults : 0; }

void setRemoteMute(uint8_t mask) {
  acknowledgedFaults |= (mask & ~remoteMask) & activeFaults;
  remoteMask = mask;
}

}  // namespace signals
