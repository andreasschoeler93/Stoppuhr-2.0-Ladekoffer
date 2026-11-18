#include "remotebeep.h"

namespace remotebeep {
namespace {

constexpr uint32_t YELLOW_BEEP_MS = 300;
// Ohne frische Antwort vom Pi (WLAN weg) verstummt der rote Dauerton
constexpr uint32_t STALE_MS = 60000;

Config cur = {false, 0, false, 2000, 10000, 0, false, 0};
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

// Zeitvergleich, der den Ueberlauf von millis() uebersteht
bool reached(uint32_t now, uint32_t t) { return (int32_t)(now - t) >= 0; }

}  // namespace

void configure(const Config& c) {
  const uint32_t now = millis();
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
