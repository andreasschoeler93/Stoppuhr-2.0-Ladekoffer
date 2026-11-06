#include "remotebeep.h"

namespace remotebeep {
namespace {

constexpr uint32_t YELLOW_BEEP_MS = 300;
// Ohne frische Antwort vom Pi (WLAN weg) verstummt der rote Dauerton
constexpr uint32_t STALE_MS = 60000;

Config cur = {false, 0, false, 2000, 10000};
uint32_t lastConfigMs = 0;
bool haveSeq = false;
uint32_t lastSeq = 0;
uint32_t yellowUntil = 0;
bool yellowActive = false;
bool redWasOn = false;
uint32_t redNextAt = 0;
uint32_t redUntil = 0;
bool redActive = false;
bool silenced = false;

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
    silenced = false;
  }
  lastSeq = c.yellowSeq;
  if (c.redOn && !redWasOn) {
    redNextAt = now;  // sofort beginnen
    silenced = false;
  }
  if (!c.redOn && !yellowActive) silenced = false;
  redWasOn = c.redOn;
  cur = c;
  lastConfigMs = now;
}

void mute() {
  silenced = true;
  yellowActive = false;
  redActive = false;
}

bool muted() { return silenced; }

bool active(uint32_t now) {
  if (silenced) return false;
  if (yellowActive && reached(now, yellowUntil)) yellowActive = false;
  if (cur.redOn && now - lastConfigMs <= STALE_MS) {
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
