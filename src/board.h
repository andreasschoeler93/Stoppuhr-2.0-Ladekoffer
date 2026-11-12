#pragma once

#include <Arduino.h>

#include "device_log.h"

// Zugriff auf die Platine: I2C, MCP23017, Multiplexer, Ampel, Pieper, Taster.
namespace board {

// Debug-Ausgaben ueber die serielle Schnittstelle (Befehl "debug on|off")
extern bool debugOn;
#define DBG(...) \
  do { if (board::debugOn) { device_log::addf(__VA_ARGS__); Serial.printf("[%lu] ", (unsigned long)millis()); Serial.printf(__VA_ARGS__); Serial.println(); } } while (0)

// Sucht Geraete am I2C-Bus und gibt die Adressen aus
void i2cScan();
// Gibt alle 16 Eingaenge von U2 roh aus (zum Pruefen der STAT-Zuordnung)
void statDump();
// Rohpegel des STAT-Pins (true = HIGH); false, solange der MCP23017 nicht antwortet
bool readStatRaw(uint8_t slot);

void begin();
// Prueft den MCP23017 und startet ihn nach einem Ausfall neu (nicht blockierend).
void service();
bool i2cOk();

// Spannung am Akku des Slots (0..9) in Volt, aus gemittelten ADC-Werten
float readVbat(uint8_t slot);
// TP4056 laedt (STAT aktiv); false, solange der MCP23017 nicht antwortet
bool readCharging(uint8_t slot);

void setLeds(bool red, bool yellow, bool green);
void setBuzzer(bool on);
// LED-Test beim Start: Rot, Gelb, Gruen nacheinander, dann alle zusammen (blockiert ca. 2 s)
void ledTest();
// Taster TRIG_ALL gedrueckt (roh, ohne Entprellung)
bool ackPressedRaw();

}  // namespace board
