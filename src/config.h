#pragma once

#include <Arduino.h>

// Zugangsdaten stehen in src/secrets.h (nicht im Repository, siehe src/secrets.example.h).
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef SECRET_WIFI_SSID
#define SECRET_WIFI_SSID "Stoppuhr-Koffer"
#endif
#ifndef SECRET_WIFI_PASS
#define SECRET_WIFI_PASS ""
#endif
#ifndef SECRET_PI_IP
#define SECRET_PI_IP "192.168.50.10"
#endif
#ifndef SECRET_API_KEY
#define SECRET_API_KEY ""
#endif

// Versionsschema: MAJOR.MINOR.PATCH (siehe README). Bei jeder Aenderung erhoehen.
#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 7
#define FW_VERSION_PATCH 25
#define FW_STR_(x) #x
#define FW_STR(x) FW_STR_(x)
#define FW_VERSION "v" FW_STR(FW_VERSION_MAJOR) "." FW_STR(FW_VERSION_MINOR) "." FW_STR(FW_VERSION_PATCH)

namespace cfg {

static constexpr uint8_t SLOT_COUNT = 10;

// ---- Pins (XIAO ESP32-C3, laut Schaltplan V3) ----
struct Pin {
  static constexpr int ADC_MUX_A = 2;  // Slots 1..8
  static constexpr int ADC_MUX_B = 3;  // Slots 9..10
  static constexpr int LED_RED = 4;
  static constexpr int TRIG_ALL = 5;  // Taster: Alarm quittieren
  static constexpr int SDA = 6;
  static constexpr int SCL = 7;
  static constexpr int LED_GRN = 20;
  static constexpr int LED_YEL = 21;
  static constexpr int MUX_A = 8;
  static constexpr int MUX_B = 9;
  static constexpr int MUX_C = 10;
};

// ---- MCP23017 U2 (Adresse 0x20): 0..7 = GPA0..7, 8..15 = GPB0..7 ----
struct Mcp {
  static constexpr uint8_t ADDR = 0x20;
  // STAT (TP4056 CHRG) je Slot 1..10
  static constexpr uint8_t STAT[SLOT_COUNT] = {8, 9, 10, 11, 12, 13, 14, 15, 0, 1};
  static constexpr uint8_t BUZZ_EN = 2;
  static constexpr uint8_t LED_RED_DRV = 3;
  static constexpr uint8_t LED_YEL_DRV = 4;
  static constexpr uint8_t LED_GRN_DRV = 5;
  // Ampel zusaetzlich auf die Treiberausgaenge fuer externe LEDs (J2) spiegeln
  static constexpr bool MIRROR_LEDS = true;
  static constexpr bool LED_ACTIVE_LOW = true;
};

// AHT20 (feste I2C-Adresse)
static constexpr uint8_t AHT_ADDR = 0x38;
static constexpr uint8_t INA226_ADDR = 0x40;

// ---- Messung ----
// STAT ist an der roten LED des TP4056-Moduls angelötet: LED an (laedt) = HIGH, LED aus = LOW
static constexpr bool STAT_ACTIVE_LOW = false;
// Spannungsteiler je Slot: 200 kOhm oben, 100 kOhm unten -> Vbat = Vadc * 3
static constexpr float VBAT_DIV_RATIO = 3.0f;
static constexpr uint8_t ADC_SAMPLES = 4;

static constexpr float TEMP_ALARM_ON_C = 60.0f;
static constexpr float TEMP_ALARM_OFF_C = 57.0f;

// ---- Zeiten ----
static constexpr uint32_t SCAN_SLOTS_MS = 250;
static constexpr uint32_t TEMP_INTERVAL_MS = 10000;
static constexpr uint32_t AHT_CONVERT_MS = 90;  // Messdauer des AHT20 (mind. 80 ms)
static constexpr uint32_t AHT_VALID_MS = 90000;  // danach gilt der Sensor als ausgefallen
static constexpr uint32_t MCP_RETRY_MS = 2000;
static constexpr uint32_t BLINK_MS = 500;
static constexpr uint32_t DEBOUNCE_MS = 40;
static constexpr bool BTN_ACTIVE_LOW = true;
// Taster halten: ab 1,5 s beim Loslassen aufwecken; ab 15 s Taster ohne Rolle ausschalten
static constexpr uint32_t LONG_PRESS_MS = 1500;
static constexpr uint32_t SLEEP_UNASSIGNED_PRESS_MS = 15000;

// ---- Aufwecken der Taster ueber die Magnet-Trigger (MCP23017 U5) ----
struct Wake {
  static constexpr uint8_t MCP_ADDR = 0x21;
  // Ausgang HIGH = Gate des 2N7000 an, MAG_TRIG gegen GND = Trigger an; im Ruhezustand LOW
  static constexpr bool ACTIVE_LOW = false;
  // Fest verdrahtet (Schaltplan V3, Start_A..J an U5): Slot 1..10 -> GPB3, GPA4, GPB4, GPA3, GPB5, GPA2, GPB6, GPA1, GPB7, GPA0
  static constexpr uint8_t DEFAULT_PINS[SLOT_COUNT] = {11, 4, 12, 3, 13, 2, 14, 1, 15, 0};
  static constexpr uint16_t DEFAULT_PULSE_MS = 3000;
  static constexpr uint16_t DEFAULT_GAP_MS = 1000;
};

// ---- Netz (Koffer-WLAN, Koffer-Pi) ----
struct Net {
  static constexpr const char* WIFI_SSID = SECRET_WIFI_SSID;
  static constexpr const char* WIFI_PASS = SECRET_WIFI_PASS;
  static constexpr const char* PI_HOST = SECRET_PI_IP;
  static constexpr uint16_t PI_PORT = 1999;
  static constexpr const char* API_KEY = SECRET_API_KEY;
  static constexpr uint32_t HTTP_TIMEOUT_MS = 1500;
  static constexpr uint32_t WIFI_RETRY_MS = 5000;
  // Der Pi wertet den Koffer als offline, wenn laenger nichts kommt (dort 30 s)
  static constexpr uint32_t SEND_INTERVAL_MS = 5000;
  // Aenderungen werden frueher gesendet, aber nie dichter als dieser Abstand
  static constexpr uint32_t MIN_GAP_MS = 1000;
};

}  // namespace cfg
