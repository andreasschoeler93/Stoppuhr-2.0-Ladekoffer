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
// Polaritaet der Trigger-Ausgaenge umschalten und im Flash speichern (gilt auch nach dem Neustart)
void setActiveLow(bool low);
// Alle 16 Ausgaenge von U5 gleichzeitig auf aktiv oder Ruhe setzen (bleibt, bis zurueckgesetzt)
void setAll(bool on);
// Schaltet U5-Pin 0..15 nacheinander je 3 s ein, um die Zuordnung zu den Slots zu finden
void startPinTest();
void begin();
// Startet die Sequenz (ignoriert, solange eine laeuft); onlySlot 0..9 weckt nur diesen Slot
void start(int8_t onlySlot = -1);
// Weckt einen einzelnen 1-basierten Slot mit derselben Pin- und Impuls-Konfiguration.
bool startSlot(uint8_t slot, const String& requestId);
bool active();
// Haelt alle Startkontakte aktiv (Taster-Update im Gange); danach zurueck auf Ruhe. Ein Weckablauf hat Vorrang.
void setHold(bool on);
String takeCompletedRequestId();
// Nicht blockierend im loop() aufrufen
void update();

}  // namespace wake
