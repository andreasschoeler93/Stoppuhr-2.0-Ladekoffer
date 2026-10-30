#pragma once

#include <Arduino.h>

#include "config.h"

// Gemeinsame Datenstrukturen von Messung, Ampel und Netzwerk.
struct SlotState {
  bool present = false;
  bool charging = false;
  bool full = false;
  bool fault = false;
  float vbat = 0.0f;
};

// Rohmessung (sensors)
struct Measurements {
  SlotState slots[cfg::SLOT_COUNT];
  float tempC = NAN;
  float humidity = NAN;
  bool ahtOk = false;
  bool tempAlarm = false;
  bool i2cOk = false;  // MCP23017 antwortet
};

enum class Light : uint8_t { OFF = 0, GREEN, YELLOW, YELLOW_BLINK, RED, RED_BLINK };

// Messung plus abgeleitete Zustaende (signals); wird an den Pi gemeldet
struct Status {
  Measurements m;
  bool anyCharging = false;
  bool allPresent = false;
  bool allFull = false;
  bool anyFault = false;
  bool hardFault = false;    // Uebertemperatur, Slot-Fehler oder I2C-Ausfall (rot dauerhaft)
  bool sensorFault = false;  // AHT20 fehlt oder liefert nichts (rot blinkend)
  bool muted = false;        // Alarm quittiert
  Light light = Light::OFF;
};
