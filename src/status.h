#pragma once

#include <Arduino.h>

#include "config.h"

// Gemeinsame Datenstrukturen von Messung, Ampel und Netzwerk.
struct SlotState {
  bool charging = false;  // rote LED des Lademoduls
  bool fault = false;
  float vbat = 0.0f;
  bool changed = false;     // Ladezustand hat sich seit dem Start geaendert
  uint32_t sinceS = 0;      // Sekunden seit dieser Aenderung (nur mit changed)
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
  bool anyFault = false;
  bool hardFault = false;    // Uebertemperatur, Slot-Fehler oder I2C-Ausfall (rot dauerhaft)
  bool sensorFault = false;  // AHT20 fehlt oder liefert nichts (rot blinkend)
  bool muted = false;        // Alarm quittiert
  Light light = Light::OFF;
};
