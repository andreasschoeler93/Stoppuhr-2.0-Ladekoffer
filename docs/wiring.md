# Verdrahtung / Hinweise

Maßgeblich ist der Schaltplan „Ladekoffer Stoppuhr-2.0 V3“.

## Pins

| Signal | XIAO | GPIO |
| --- | --- | --- |
| ADC_MUX_A (Slots 1..8) | D0 | 2 |
| ADC_MUX_B (Slots 9..10) | D1 | 3 |
| LED_RED | D2 | 4 |
| TRIG_ALL (Quittieren) | D3 | 5 |
| SDA / SCL | D4 / D5 | 6 / 7 |
| LED_YEL | D6 | 21 |
| LED_GRN | D7 | 20 |
| MUX_A / B / C | D8 / D9 / D10 | 8 / 9 / 10 |

Slot *n* liegt auf Multiplexer-Kanal *n−1* (Slots 1..8 an `U3`, Slots 9 und 10 an `U4`, Kanal X0/X1).

## MCP23017 U2 (0x20)

`STAT_1..8` = GPB0..7, `STAT_9/10` = GPA0/1, `BUZZ_EN` = GPA2, `LED_RED/YEL/GRN_DRV` = GPA3..5.

## TP4056 STAT (CHRG)

CHRG ist Open-Drain, aktiv LOW (lädt). Es unterscheidet nicht „voll“ von „nicht gesteckt“; deshalb gilt:
`present` über die Akkuspannung, `full` = vorhanden, lädt nicht und Spannung ≥ `VBAT_FULL_V`.

## Taster TRIG_ALL

`INPUT_PULLUP`, Taster gegen GND. Kurzer Druck quittiert den Pieper (nur bei aktivem Fehler); Halten ab 1,5 s weckt die Taster (siehe README).
