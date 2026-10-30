#pragma once

#include <Arduino.h>

// Zugriff auf die Platine: I2C, MCP23017, Multiplexer, Ampel, Pieper, Taster.
namespace board {

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
// Taster TRIG_ALL gedrueckt (roh, ohne Entprellung)
bool ackPressedRaw();

}  // namespace board
