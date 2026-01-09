# Verdrahtung / Hinweise

## TP4056 STAT (CHRG)

Die Firmware geht davon aus:
- STAT = TP4056 **CHRG**
- CHRG ist typischerweise **Open-Drain, active LOW**
  - LOW = lädt
  - HIGH = lädt nicht (voll oder nicht gesteckt)

## Präsenz & "Voll" Erkennung

Da CHRG allein nicht "voll" vs. "nicht gesteckt" unterscheiden kann, verwendet die Firmware:
- `present` über VBAT (ADC via MUX)
- `full` über VBAT-Schwelle + !charging

## Ampel

- Grün: alle Slots present und full
- Gelb: irgendein Slot charging
  - blinkend, wenn nicht alle present
- Rot: Fault
  - blinkend bei Sensorfault (AHT20)

## Button (TRIG_ALL)

- TRIG_ALL ist in der Firmware als `INPUT_PULLUP` konfiguriert
- Taster gegen GND
- Kurzer Druck quittiert den Pieper (nur wenn Fault aktiv)
