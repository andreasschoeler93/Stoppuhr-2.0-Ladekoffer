#pragma once

#include <Arduino.h>

#include "config.h"

// Weckt die Taster im Koffer der Reihe nach: Slot 1 zuerst, jeder Magnet-Trigger
// bleibt pulseMs aktiv und startet gapMs nach dem vorherigen (die Impulse ueberlappen).
namespace wake {

struct Config {
  uint8_t pins[cfg::SLOT_COUNT];  // MCP23017-Pin (0..15) je Slot
  uint16_t pulseMs;
  uint16_t gapMs;
};

Config defaults();
// Neue Werte vom Pi; wirkt ab dem naechsten Start, ein laufender Ablauf bleibt unveraendert
void configure(const Config& c);
void begin();
// Startet die Sequenz (ignoriert, solange eine laeuft)
void start();
bool active();
// Nicht blockierend im loop() aufrufen
void update();

}  // namespace wake
