# Ladekoffer-Controller (v0.3.0)

Firmware für die Ladeplatine des Ladekoffers (Seeed XIAO ESP32-C3, PlatformIO, Arduino-Framework).
Sie überwacht 10 Ladeplätze (TP4056), zeigt den Zustand mit Ampel und Pieper und meldet ihn per WLAN an den Koffer-Pi
(Repository `Stoppuhr`). Die Taster bleiben unverändert; sie laden nur im Koffer und haben keine Verbindung zur Platine.

Versionsverlauf: [CHANGELOG.md](CHANGELOG.md). Verdrahtung: [docs/wiring.md](docs/wiring.md). Fehlersuche: [docs/troubleshooting.md](docs/troubleshooting.md).

## Hardware (Schaltplan V3)

- MCU: XIAO ESP32-C3
- MCP23017 `U2` (0x20): STAT der 10 TP4056, Pieper, Treiber für externe LEDs
- 2× 74HC4051 (`U3`, `U4`): Akkuspannung der Slots auf 2 ADC-Pins (Teiler 200 k / 100 k, Faktor 3)
- AHT20: Temperatur und Feuchte
- Ampel (Rot/Gelb/Grün) direkt am ESP, Taster `TRIG_ALL` quittiert den Alarm

Der U5 (0x21) schaltet über `MAG_TRIG` die Magnetkontakte der Slots und weckt damit die Taster (siehe „Taster aufwecken“).

## Taster aufwecken

Den Quittiertaster `TRIG_ALL` mindestens 1,5 s halten (Kurzdruck quittiert weiter den Pieper). Dann werden die Magnet-Trigger
von Slot 1 bis 10 nacheinander gegen GND gezogen: je 3 s lang, der nächste startet 1 s nach dem vorherigen
(die Impulse überlappen). Im Ruhezustand liegen die Ausgänge auf HIGH. Auch per seriellem Befehl `wake`.
Pin je Slot, Impulslänge und Abstand stellt man im Pi-Dashboard unter Einstellungen ein; der Koffer erhält sie in der Antwort
auf `POST /api/charger` (`{"ok":true,"wake":{"pins":[..10 Werte..],"ms":3000,"gap_ms":1000}}`). Vorgabe bis dahin:
`8,9,10,11,12,13,14,15,0,1` (0–7 = GPA0–7, 8–15 = GPB0–7); bitte mit dem Schaltplan abgleichen.

## Ampel und Pieper

| Ampel | Bedeutung |
| --- | --- |
| Grün | alle Taster da und voll |
| Gelb | mindestens ein Taster lädt (blinkt, wenn nicht alle da sind) |
| Rot dauerhaft | Übertemperatur, Slot-Fehler (Überspannung) oder MCP23017 antwortet nicht |
| Rot blinkend | AHT20 fehlt oder liefert keine Werte |

Der Pieper ertönt nur bei Fehlern; ein kurzer Druck auf `TRIG_ALL` quittiert ihn bis zum nächsten neuen Fehler.

## Bauen und flashen

```text
pio run
pio run -t upload
pio device monitor -b 115200
```

### Zugangsdaten (`src/secrets.h`)

`src/secrets.example.h` nach `src/secrets.h` kopieren (wird nicht hochgeladen) und eintragen:
`SECRET_WIFI_SSID`/`SECRET_WIFI_PASS` (Koffer-WLAN, der Zugangspunkt des Koffer-Pi, versteckt – SSID exakt;
der Controller verbindet sich ausschließlich mit diesem WLAN),
`SECRET_PI_IP` (Standard `192.168.50.10`, die gemeinsame Adresse der Pis), optional `SECRET_API_KEY`.

## Versionierung

Schema `MAJOR.MINOR.PATCH`, die Version steht in `src/config.h` (`FW_VERSION_*`) und wird beim Start und mit jeder Meldung gesendet.
Bei jeder Änderung Version erhöhen und `CHANGELOG.md` ergänzen.

## Aufbau (`src/`)

| Datei | Aufgabe |
| --- | --- |
| `config.h` | Pins, Schwellwerte, Zeiten, Netz, Version |
| `board.*` | I2C, MCP23017, Multiplexer, LEDs, Pieper, Taster |
| `sensors.*` | Slots (da/lädt/voll/Fehler) und AHT20, ohne Wartezeiten |
| `signals.*` | Gesamtstatus, Ampel, Pieper, Quittieren |
| `netlink.*` | WLAN und Senden an den Pi in einem eigenen Task |
| `main.cpp` | Ablauf und serielle Befehle (`status`, `mute`, `unmute`) |

`loop()` wartet nie. Ohne WLAN oder Pi arbeiten Ampel, Pieper und Temperaturschutz unverändert weiter.

## Schnittstelle zum Pi

`POST http://<Pi>:1999/api/charger` (JSON, optional Header `X-Api-Key`), alle 5 s und sofort (frühestens nach 1 s) bei einer Änderung
von Slot-Zustand, Ampel, Alarm oder Quittierung. Der Pi bewertet den Koffer als offline, wenn 30 s nichts ankommt.

```json
{"mac":"AA:BB:..","fw":"v0.1.0","uptime_s":120,"rssi":-55,
 "temp":24.5,"humidity":40,"aht_ok":true,"temp_alarm":false,"i2c_ok":true,
 "light":"green","fault":false,"muted":false,
 "present":10,"charging":0,"full":10,
 "slots":[{"n":1,"present":true,"charging":false,"full":true,"fault":false,"vbat":4.12}]}
```

`light`: `off`, `green`, `yellow`, `yellow_blink`, `red`, `red_blink`. Die Zeit stempelt der Pi selbst.

## Kalibrierung

- `VBAT_DIV_RATIO` (3,0 bei 200 k / 100 k): mit Multimeter gegenprüfen, Kommando `status`.
- `VBAT_PRESENT_V`, `VBAT_FULL_V`, `TEMP_ALARM_ON_C`/`_OFF_C` in `config.h`.
