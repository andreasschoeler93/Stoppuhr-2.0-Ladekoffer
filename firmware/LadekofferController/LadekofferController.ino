/*
  Ladekoffer Controller – Stoppuhr 2.0
  Target: Seeed Studio XIAO ESP32-C3 (Arduino-ESP32 Core)

  - 10 Slots
  - TP4056 STAT (CHRG) über MCP23017 (I2C 0x20) -> active LOW
  - VBAT über 2× 74HC4051 auf 2 ADCs
  - Ampel-LEDs: ROT/GELB/GRÜN direkt am ESP
  - Pieper über MCP23017: BUZZ_EN
  - TRIG_ALL: Taster (Quittieren des Piepser bei Fehler)

  Ampel (dein Konzept):
  - GRÜN: alle Taster da UND voll (kein Laden)
  - GELB: es wird geladen
      * GELB dauerhaft: alle da und mind. einer lädt
      * GELB blinkend: mind. einer lädt, aber nicht alle da
  - ROT: Fehler
      * ROT dauerhaft: Fehler (Temperatur, Slot-Fault)
      * ROT blinkend: Sensorfehler (AHT20 fehlt/ungültig)

  Pieper:
  - nur bei Fehler
  - Quittierung per kurzem Tastendruck am TRIG_ALL
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_AHTX0.h>

// ---------------- Pins ----------------
static constexpr int PIN_ADC_MUX_A = 2;
static constexpr int PIN_ADC_MUX_B = 3;

static constexpr int PIN_LED_RED   = 4;
static constexpr int PIN_TRIG_ALL  = 5;   // Button (Quittierung)

static constexpr int PIN_SDA       = 6;
static constexpr int PIN_SCL       = 7;

static constexpr int PIN_MUX_A     = 8;
static constexpr int PIN_MUX_B     = 9;
static constexpr int PIN_MUX_C     = 10;

static constexpr int PIN_LED_GRN   = 20;
static constexpr int PIN_LED_YEL   = 21;

// ---------------- MCP23017 ----------------
Adafruit_MCP23X17 mcp;
static constexpr uint8_t MCP_ADDR = 0x20;

static constexpr uint8_t MCP_STAT_1  = 8;   // GPB0
static constexpr uint8_t MCP_STAT_2  = 9;   // GPB1
static constexpr uint8_t MCP_STAT_3  = 10;  // GPB2
static constexpr uint8_t MCP_STAT_4  = 11;  // GPB3
static constexpr uint8_t MCP_STAT_5  = 12;  // GPB4
static constexpr uint8_t MCP_STAT_6  = 13;  // GPB5
static constexpr uint8_t MCP_STAT_7  = 14;  // GPB6
static constexpr uint8_t MCP_STAT_8  = 15;  // GPB7
static constexpr uint8_t MCP_STAT_9  = 0;   // GPA0
static constexpr uint8_t MCP_STAT_10 = 1;   // GPA1
static constexpr uint8_t MCP_BUZZ_EN = 2;   // GPA2

// ---------------- AHT20 ----------------
Adafruit_AHTX0 aht;
static bool ahtOk = false;

// ---------------- TP4056 / VBAT ----------------
static constexpr bool  STAT_ACTIVE_LOW = true;
static constexpr float VBAT_DIV_RATIO  = 2.0f;   // <-- ggf. anpassen
static constexpr float VBAT_PRESENT_V  = 2.0f;
static constexpr float VBAT_FULL_V     = 4.10f;
static constexpr float VBAT_ABS_MAX_V  = 4.30f;

// change thresholds for event printing
static constexpr float VBAT_DELTA_REPORT_V = 0.05f;
static constexpr float TEMP_DELTA_REPORT_C = 0.5f;

// ---------------- Temperatur ----------------
static constexpr float TEMP_ALARM_ON_C  = 60.0f;
static constexpr float TEMP_ALARM_OFF_C = 57.0f;

// ---------------- Timing ----------------
static constexpr uint32_t SCAN_SLOTS_MS = 250;
static constexpr uint32_t READ_TEMP_MS  = 1000;
static constexpr uint32_t BLINK_MS      = 500;

// ---------------- Button Debounce ----------------
static constexpr bool BTN_ACTIVE_LOW    = true;
static constexpr uint32_t DEBOUNCE_MS   = 40;

// ---------------- State ----------------
struct SlotState {
  bool present = false;
  bool charging = false;
  bool full = false;
  bool fault = false;
  float vbat = 0.0f;
};
static SlotState slots[10];
static SlotState prevSlots[10];

static float tempC = NAN;
static float prevTempC = NAN;
static bool tempAlarm = false;

static bool alarmMuted = false;
static bool lastAnyFault = false;
static bool prevAnyFault = false;

static bool prevAhtOk = false;

static uint32_t lastScanSlotsMs = 0;
static uint32_t lastTempMs = 0;

// ---------------- Helpers ----------------
static inline void setLeds(bool red, bool yel, bool grn) {
  digitalWrite(PIN_LED_RED, red ? HIGH : LOW);
  digitalWrite(PIN_LED_YEL, yel ? HIGH : LOW);
  digitalWrite(PIN_LED_GRN, grn ? HIGH : LOW);
}

static inline void buzzerOn(bool on) {
  mcp.digitalWrite(MCP_BUZZ_EN, on ? HIGH : LOW);
}

static inline void setMuxChannel(uint8_t ch) {
  digitalWrite(PIN_MUX_A, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_MUX_B, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_MUX_C, (ch & 0x04) ? HIGH : LOW);
  // allow settling after switching multiplexer
  delayMicroseconds(200);
}

static inline float readAdcVolts(int adcPin) {
  uint32_t mv = analogReadMilliVolts(adcPin);
  return (float)mv / 1000.0f;
}

static bool readStatActive(uint8_t slotIndex0) {
  uint8_t pin;
  switch (slotIndex0) {
    case 0:  pin = MCP_STAT_1;  break;
    case 1:  pin = MCP_STAT_2;  break;
    case 2:  pin = MCP_STAT_3;  break;
    case 3:  pin = MCP_STAT_4;  break;
    case 4:  pin = MCP_STAT_5;  break;
    case 5:  pin = MCP_STAT_6;  break;
    case 6:  pin = MCP_STAT_7;  break;
    case 7:  pin = MCP_STAT_8;  break;
    case 8:  pin = MCP_STAT_9;  break;
    default: pin = MCP_STAT_10; break;
  }
  int raw = mcp.digitalRead(pin);
  bool activeLow = (raw == LOW);
  return STAT_ACTIVE_LOW ? activeLow : !activeLow;
}

static int readStatRawPin(uint8_t slotIndex0) {
  uint8_t pin;
  switch (slotIndex0) {
    case 0:  pin = MCP_STAT_1;  break;
    case 1:  pin = MCP_STAT_2;  break;
    case 2:  pin = MCP_STAT_3;  break;
    case 3:  pin = MCP_STAT_4;  break;
    case 4:  pin = MCP_STAT_5;  break;
    case 5:  pin = MCP_STAT_6;  break;
    case 6:  pin = MCP_STAT_7;  break;
    case 7:  pin = MCP_STAT_8;  break;
    case 8:  pin = MCP_STAT_9;  break;
    default: pin = MCP_STAT_10; break;
  }
  return mcp.digitalRead(pin);
}

static float readSlotVbat(uint8_t slotIndex0) {
  if (slotIndex0 <= 7) {
    setMuxChannel(slotIndex0);
    // discard first reading after switching, then average few readings
    const int N = 3;
    uint32_t sumMv = 0;
    for (int i = 0; i < N; ++i) {
      sumMv += analogReadMilliVolts(PIN_ADC_MUX_A);
      delayMicroseconds(50);
    }
    float mv = (float)sumMv / N;
    return (mv / 1000.0f) * VBAT_DIV_RATIO;
  } else {
    uint8_t ch = slotIndex0 - 8; // Slot9->0, Slot10->1
    setMuxChannel(ch);
    const int N = 3;
    uint32_t sumMv = 0;
    for (int i = 0; i < N; ++i) {
      sumMv += analogReadMilliVolts(PIN_ADC_MUX_B);
      delayMicroseconds(50);
    }
    float mv = (float)sumMv / N;
    return (mv / 1000.0f) * VBAT_DIV_RATIO;
  }
}

// ---------------- Button (Quittierung) ----------------
static bool btnStable = false;
static bool btnLastStable = false;
static uint32_t btnLastChangeMs = 0;

static bool readButtonPressedRaw() {
  int v = digitalRead(PIN_TRIG_ALL);
  return BTN_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

static void updateAckButton(bool anyFault) {
  bool raw = readButtonPressedRaw();
  uint32_t now = millis();

  static bool rawLast = false;
  if (raw != rawLast) {
    rawLast = raw;
    btnLastChangeMs = now;
  }
  if ((now - btnLastChangeMs) >= DEBOUNCE_MS) {
    btnStable = raw;
  }

  // kurzer Druck -> bei Fehler quittieren (Ton aus), LED bleibt wie gehabt
  if (!btnStable && btnLastStable) {
    Serial.println(F("Button: short press detected (ack)."));
    if (anyFault) {
      alarmMuted = true;
      Serial.println(F("Alarm muted by button."));
    }
  }

  // long press / hold could be observed here in the future

  btnLastStable = btnStable;
}

// ---------------- Temp ----------------
static void readTemperature() {
  if (!ahtOk) { tempC = NAN; return; }

  sensors_event_t hum, temp;
  // some AHT libs return void for getEvent; check behavior - original code used boolean
  // We'll call it and trust the library fills the structs.
  aht.getEvent(&hum, &temp);
  float newTempC = temp.temperature;
  tempC = newTempC;

  if (!tempAlarm && !isnan(tempC) && tempC >= TEMP_ALARM_ON_C) {
    tempAlarm = true;
    alarmMuted = false; // neuer Alarm -> Ton aktiv
  }
  if (tempAlarm && !isnan(tempC) && tempC <= TEMP_ALARM_OFF_C) {
    tempAlarm = false;
  }

  // report temperature changes
  if (isnan(prevTempC) || fabs(tempC - prevTempC) >= TEMP_DELTA_REPORT_C) {
    if (isnan(tempC)) {
      Serial.println(F("Temperature: N/A"));
    } else {
      Serial.printf("Temperature: %0.1f C (alarm=%s)\n", tempC, tempAlarm ? "YES" : "no");
    }
    prevTempC = tempC;
  }
}

// ---------------- Slot Scan ----------------
static void printSlotChange(int idx, const SlotState &oldS, const SlotState &s) {
  bool any = false;
  String msg;
  msg.reserve(128);
  msg += "Slot ";
  msg += String(idx + 1);
  msg += ": ";

  if (oldS.present != s.present) {
    msg += s.present ? "present " : "removed ";
    any = true;
  }
  if (oldS.charging != s.charging) {
    msg += s.charging ? "charging " : "not-charging ";
    any = true;
  }
  if (oldS.full != s.full) {
    msg += s.full ? "full " : "not-full ";
    any = true;
  }
  if (oldS.fault != s.fault) {
    msg += s.fault ? "FAULT " : "fault-cleared ";
    any = true;
  }
  if (fabs(oldS.vbat - s.vbat) >= VBAT_DELTA_REPORT_V) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%0.3fV", s.vbat);
    msg += buf;
    any = true;
  }
  if (any) {
    Serial.println(msg);
  }
}

static void scanSlotsOnce() {
  for (int i = 0; i < 10; i++) {
    SlotState old = slots[i];
    SlotState &s = slots[i];
    float v = readSlotVbat(i);
    bool statActive = readStatActive(i);

    s.vbat = v;
    s.present  = (v >= VBAT_PRESENT_V);
    s.charging = s.present && statActive;
    s.full     = s.present && !s.charging && (v >= VBAT_FULL_V);
    s.fault    = s.present && (v > VBAT_ABS_MAX_V);

    // print per-slot changes compared to previous
    printSlotChange(i, prevSlots[i], s);

    // persist for next comparison
    prevSlots[i] = s;
  }
}

// ---------------- Outputs ----------------
static void updateOutputs() {
  bool anyCharging = false;
  bool allPresent  = true;
  bool allFull     = true;
  bool anySlotFault = false;

  for (int i = 0; i < 10; i++) {
    anyCharging  |= slots[i].charging;
    allPresent   &= slots[i].present;
    allFull      &= slots[i].full;
    anySlotFault |= slots[i].fault;
  }

  bool faultSensor = (!ahtOk) || isnan(tempC);
  bool faultTemp   = tempAlarm;
  bool faultOther  = anySlotFault;

  bool anyFault = faultSensor || faultTemp || faultOther;

  // Quittierung zurücksetzen wenn Fehler weg ist
  if (!anyFault) alarmMuted = false;
  // Neuer Fehler -> Quittierung aufheben
  if (anyFault && !lastAnyFault) alarmMuted = false;

  // Quittierung per Button (kurz)
  updateAckButton(anyFault);

  // report anyFault change
  if (anyFault != prevAnyFault) {
    Serial.printf("AnyFault changed: %s -> %s\n", prevAnyFault ? "YES" : "no", anyFault ? "YES" : "no");
    prevAnyFault = anyFault;
  }

  bool blink = ((millis() / BLINK_MS) % 2) == 0;

  // Ampel-Priorität: ROT > GELB > GRÜN
  if (faultSensor) {
    setLeds(blink, false, false);                 // rot blinkend
  } else if (anyFault) {
    setLeds(true, false, false);                  // rot dauerhaft
  } else if (anyCharging) {
    setLeds(false, allPresent ? true : blink, false); // gelb dauer/blink
  } else if (allPresent && allFull) {
    setLeds(false, false, true);                  // grün
  } else {
    setLeds(false, false, false);
  }

  // Buzzer nur bei Fehler und nicht quittiert
  static uint32_t lastToggle = 0;
  static bool buzState = false;

  if (anyFault && !alarmMuted) {
    uint32_t now = millis();
    uint32_t onMs = 0, offMs = 0;

    if (faultTemp) { onMs = 200; offMs = 800; }         // Temp-Alarm
    else if (faultSensor) { onMs = 80; offMs = 920; }   // Sensorfehler
    else { onMs = 100; offMs = 1900; }                  // sonstiges

    uint32_t period = buzState ? onMs : offMs;
    if (now - lastToggle >= period) {
      lastToggle = now;
      buzState = !buzState;
      buzzerOn(buzState);
    }
  } else {
    buzState = false;
    buzzerOn(false);
  }

  lastAnyFault = anyFault;
}

// ---------------- Serial Debug ----------------
static void printStatus() {
  bool anyCharging = false, allPresent = true, allFull = true, anySlotFault = false;
  for (int i = 0; i < 10; i++) {
    anyCharging |= slots[i].charging;
    allPresent &= slots[i].present;
    allFull &= slots[i].full;
    anySlotFault |= slots[i].fault;
  }
  bool faultSensor = (!ahtOk) || isnan(tempC);
  bool anyFault = faultSensor || tempAlarm || anySlotFault;

  Serial.println();
  Serial.println(F("=== Ladekoffer Status ==="));
  Serial.printf("Temp: %s  (tempAlarm=%s, ahtOk=%s)\n",
                isnan(tempC) ? "N/A" : String(tempC, 1).c_str(),
                tempAlarm ? "YES" : "no",
                ahtOk ? "YES" : "no");
  Serial.printf("Aggregate: allPresent=%s allFull=%s anyCharging=%s\n",
                allPresent ? "YES" : "no",
                allFull ? "YES" : "no",
                anyCharging ? "YES" : "no");
  Serial.printf("Faults: any=%s sensor=%s slotFault=%s muted=%s\n",
                anyFault ? "YES" : "no",
                faultSensor ? "YES" : "no",
                anySlotFault ? "YES" : "no",
                alarmMuted ? "YES" : "no");

  Serial.println(F("Slot | Present | Charging | Full | Fault | Vbat"));
  Serial.println(F("-----+---------+----------+------+-------+------"));
  for (int i = 0; i < 10; i++) {
    const SlotState &s = slots[i];
    Serial.printf("%4d | %7s | %8s | %4s | %5s | %0.3f V\n",
                  i + 1,
                  s.present ? "yes" : "no",
                  s.charging ? "yes" : "no",
                  s.full ? "yes" : "no",
                  s.fault ? "yes" : "no",
                  s.vbat);
  }
  Serial.println();
}

static void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "status" || cmd == "s") {
    printStatus();
  } else if (cmd == "mute") {
    alarmMuted = true;
    Serial.println(F("Muted (if fault active)."));
  } else if (cmd == "unmute") {
    alarmMuted = false;
    Serial.println(F("Unmuted."));
  } else if (cmd == "diag" || cmd == "d") {
    Serial.println(F("Diagnostic: MCP STAT raw pins:"));
    for (int i = 0; i < 10; ++i) {
      int raw = readStatRawPin(i);
      Serial.printf(" STAT %2d: raw=%d\n", i + 1, raw);
    }
    Serial.println(F("End diagnostic."));
  } else {
    Serial.println(F("Commands: status | mute | unmute | diag"));
  }
}

// ---------------- Startup test ----------------
static void startupTest() {
  Serial.println(F("Running startup test: buzzer + LEDs"));

  // Buzzer short beep
  buzzerOn(true);
  delay(180);
  buzzerOn(false);
  delay(200);

  // LEDs sequence: red -> yellow -> green
  setLeds(true, false, false);
  delay(200);
  setLeds(false, true, false);
  delay(200);
  setLeds(false, false, true);
  delay(200);
  setLeds(false, false, false);
  delay(100);

  Serial.println(F("Startup test done."));
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(150);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YEL, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  setLeds(false, false, false);

  pinMode(PIN_MUX_A, OUTPUT);
  pinMode(PIN_MUX_B, OUTPUT);
  pinMode(PIN_MUX_C, OUTPUT);
  setMuxChannel(0);

  analogReadResolution(12);

  // TRIG_ALL als Button
  pinMode(PIN_TRIG_ALL, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!mcp.begin_I2C(MCP_ADDR, &Wire)) {
    setLeds(true, false, false);
    while (true) {
      Serial.println(F("ERROR: MCP23017 not found (0x20)."));
      delay(1000);
    }
  }

  const uint8_t statPins[] = {
    MCP_STAT_1, MCP_STAT_2, MCP_STAT_3, MCP_STAT_4, MCP_STAT_5,
    MCP_STAT_6, MCP_STAT_7, MCP_STAT_8, MCP_STAT_9, MCP_STAT_10
  };
  for (uint8_t p : statPins) {
    mcp.pinMode(p, INPUT);
    mcp.pullUp(p, HIGH);
  }

  mcp.pinMode(MCP_BUZZ_EN, OUTPUT);
  buzzerOn(false);

  ahtOk = aht.begin(&Wire);
  if (!ahtOk) Serial.println(F("WARN: AHT20 not found (sensor fault -> RED blink)."));

  // run a small hardware test so someone at device can hear/see basic functionality
  startupTest();

  // initial reads and state setup
  scanSlotsOnce();
  readTemperature();
  updateOutputs();

  // copy current state to prev state so further prints only show changes
  for (int i = 0; i < 10; ++i) prevSlots[i] = slots[i];
  prevTempC = tempC;
  prevAhtOk = ahtOk;
  prevAnyFault = lastAnyFault;

  Serial.println(F("Ladekoffer Controller ready."));
  Serial.println(F("Commands: status | mute | unmute | diag"));
  printStatus();
}

void loop() {
  handleSerial();

  uint32_t now = millis();

  if (now - lastScanSlotsMs >= SCAN_SLOTS_MS) {
    lastScanSlotsMs = now;
    scanSlotsOnce();
  }

  if (now - lastTempMs >= READ_TEMP_MS) {
    lastTempMs = now;
    readTemperature();
  }

  updateOutputs();
}
