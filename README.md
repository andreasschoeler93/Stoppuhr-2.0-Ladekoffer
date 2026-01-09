# Ladekoffer Controller (Stoppuhr 2.0)

Firmware für den Ladekoffer-Controller mit 10 Slots (TP4056), Ampel-LEDs und Temperaturüberwachung.

## Features

- 10 Slots: Präsenz/VBAT-Messung über 2× 74HC4051 (MUX) auf 2 ADC-Pins
- Ladezustand pro Slot über MCP23017 (I²C) via TP4056-STAT (CHRG, active LOW)
- Ampel-Logik (wie im Projekt definiert):
  - **Grün:** alle Taster vorhanden **und** voll
  - **Gelb:** mindestens ein Taster lädt
    - gelb **dauerhaft:** alle Taster vorhanden
    - gelb **blinkend:** nicht alle Taster vorhanden, aber es wird geladen
  - **Rot:** Fehler
    - rot **dauerhaft:** Fehler aktiv (z. B. Temperaturalarm oder Slot-Fault)
    - rot **blinkend:** Sensorfehler (AHT20 fehlt/ungültig)
- Pieper nur bei Fehler (z. B. Temperaturalarm); **quittierbar über Taster an TRIG_ALL**
- Serielle Debug-Kommandos: `status`, `mute`, `unmute`

## Hardware / Zielplattform

- MCU: **Seeed Studio XIAO ESP32-C3**
- I²C Expander: **MCP23017** (Adresse 0x20)
- Temperatur: **AHT20** (I²C)
- MUX: **2× 74HC4051**
- Ladeboards: **TP4056** (STAT = CHRG, active LOW)

## Pinmapping (Firmware)

| Signal | GPIO |
|---|---:|
| ADC_MUX_A (Slots 1..8) | GPIO2 |
| ADC_MUX_B (Slots 9..10) | GPIO3 |
| LED_RED | GPIO4 |
| TRIG_ALL (Button: Ack Pieper) | GPIO5 |
| I²C SDA | GPIO6 |
| I²C SCL | GPIO7 |
| MUX_A | GPIO8 |
| MUX_B | GPIO9 |
| MUX_C | GPIO10 |
| LED_GRN | GPIO20 |
| LED_YEL | GPIO21 |

> Hinweis: Slots 9/10 werden auf dem zweiten 4051 auf Kanal X0/X1 erwartet.

## Abhängigkeiten (Arduino Library Manager)

- **Adafruit MCP23X17**
- **Adafruit AHTX0**

## Build & Flash (Arduino IDE)

1. Board-Paket installieren: `esp32` by Espressif
2. Board auswählen: *XIAO ESP32C3*
3. Bibliotheken installieren (oben genannt)
4. Sketch öffnen: `firmware/LadekofferController/LadekofferController.ino`
5. Flashen

## Wichtige Konfiguration

Im Sketch bitte prüfen/anpassen:

- `VBAT_DIV_RATIO` (Spannungsteiler-Faktor):  
  `Vbat = Vadc * VBAT_DIV_RATIO`
- Schwellwerte:
  - `VBAT_PRESENT_V`, `VBAT_FULL_V`
  - `TEMP_ALARM_ON_C`, `TEMP_ALARM_OFF_C`

## Serielle Befehle

- `status` : zeigt Slot-Status, Aggregate, Temperatur, Faults
- `mute` : Pieper quittieren (falls Fehler aktiv)
- `unmute` : Quittierung aufheben

## Lizenz

Siehe `LICENSE`.
