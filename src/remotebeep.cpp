#include "remotebeep.h"

namespace remotebeep {
namespace {

constexpr uint32_t YELLOW_BEEP_MS = 300;
// Ohne frische Antwort vom Pi (WLAN weg) verstummt der rote Dauerton
constexpr uint32_t STALE_MS = 60000;

Config cur = {false, 0, false, 2000, 10000, 0, false, 0, false, false, false, false};
uint32_t lastConfigMs = 0;
bool haveSeq = false;
uint32_t lastSeq = 0;
uint32_t yellowUntil = 0;
bool yellowActive = false;
bool redWasOn = false;
bool haveRedSeq = false;
uint32_t lastRedSeq = 0;
uint32_t redNextAt = 0;
uint32_t redUntil = 0;
bool redActive = false;
bool redSilenced = false;
bool haveVisualSeq = false;
uint32_t visualYellowSeq = 0;
uint32_t visualRedSeq = 0;
bool visualYellowWasOn = false;
bool visualRedWasOn = false;
uint8_t visualPending = 0;

// Zeitvergleich, der den Ueberlauf von millis() uebersteht
bool reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

}  // namespace

void configure(const Config& c) {
  const uint32_t now = millis();
  const bool freshYellowEvent = haveVisualSeq && c.hasVisualFresh && c.visualFresh &&
      c.yellowSeq != visualYellowSeq;
  const bool freshRedEvent = haveVisualSeq && c.hasVisualFresh && c.visualFresh &&
      c.hasRedSeq && c.redSeq != visualRedSeq;
  if (freshYellowEvent || (c.visualYellowOn &&
      (!haveVisualSeq || !visualYellowWasOn || c.yellowSeq != visualYellowSeq))) {
    visualPending |= YELLOW_ALERT;
  }
  if (freshRedEvent || (c.visualRedOn &&
      (!haveVisualSeq || !visualRedWasOn || (c.hasRedSeq && c.redSeq != visualRedSeq)))) {
    visualPending |= RED_ALERT;
  }
  haveVisualSeq = true;
  visualYellowSeq = c.yellowSeq;
  visualRedSeq = c.redSeq;
  visualYellowWasOn = c.visualYellowOn;
  visualRedWasOn = c.visualRedOn;
  if (!haveSeq) {
    haveSeq = true;  // der erste Stand nach dem Start loest keinen Ton aus
  } else if (c.yellowSeq != lastSeq && c.yellowOn) {
    yellowUntil = now + YELLOW_BEEP_MS;
    yellowActive = true;
  }
  lastSeq = c.yellowSeq;
  const bool newRed = c.redOn && (c.hasRedSeq
      ? (!haveRedSeq || c.redSeq != lastRedSeq)
      : !redWasOn);
  if (newRed) {
    redNextAt = now;  // sofort beginnen
    redSilenced = false;
  }
  if (c.hasRedSeq && c.redOn) {
    haveRedSeq = true;
    lastRedSeq = c.redSeq;
  }
  redWasOn = c.redOn;
  cur = c;
  lastConfigMs = now;
}

void mute() {
  if (cur.redOn) redSilenced = true;
  yellowActive = false;
  redActive = false;
}

bool muted() { return redSilenced; }

uint8_t takeVisualAlerts() {
  const uint8_t pending = visualPending;
  visualPending = 0;
  return pending;
}

bool active(uint32_t now) {
  if (yellowActive && reached(now, yellowUntil)) yellowActive = false;
  if (cur.redOn && !redSilenced && now - lastConfigMs <= STALE_MS) {
    if (reached(now, redNextAt)) {
      redUntil = now + cur.redMs;
      redActive = true;
      redNextAt = now + (cur.redEveryMs > cur.redMs ? cur.redEveryMs : cur.redMs + 1000);
    }
    if (redActive && reached(now, redUntil)) redActive = false;
  } else {
    redActive = false;
  }
  return yellowActive || redActive;
}

}  // namespace remotebeep
