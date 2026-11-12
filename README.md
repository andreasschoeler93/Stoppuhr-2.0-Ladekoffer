# Ladekoffer-Controller (v0.7.23)

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

## Hardware-Unterlagen

- [Interaktiven Bestückungsplan (iBOM) öffnen](https://andreasschoeler93.github.io/Stoppuhr-2.0-Ladekoffer/ibom.html)
- [Bestellliste herunterladen](docs/Stoppuhr_2_0_Ladeplatine_Bestellliste.xlsx)

## Taster aufwecken

Den Quittiertaster `TRIG_ALL` mindestens 1,5 s halten (Kurzdruck quittiert weiter den Pieper). Dann werden die Magnet-Trigger
in der im Pi-Dashboard konfigurierten Slot-Reihenfolge angesteuert: je 3 s lang, der nächste startet 1 s nach dem vorherigen
(die Impulse überlappen). Jeder Trigger ist ein 2N7000: Ausgang HIGH = Trigger an, im Ruhezustand liegen die Ausgänge auf LOW. Auch per seriellem Befehl `wake`.
Die Aufweckreihenfolge, Impulslänge und Abstand stellt man im Pi-Dashboard unter Einstellungen ein; der Koffer erhält sie in der Antwort
auf `POST /api/charger` (`{"ok":true,"wake":{"pins":[..10 Werte..],"order":[1,..,10],"ms":3000,"gap_ms":1000}}`).
Die Pin-Zuordnung ist durch die Platine fest vorgegeben (Schaltplan V3, Start_A..J) und wird in der aktuellen Firmware nicht aus der Pi-Einstellung übernommen.
Slot 1..10 = U5-Pin `11,4,12,3,13,2,14,1,15,0` (0–7 = GPA0–7, 8–15 = GPB0–7).
Die Weckreihenfolge enthält jede Slotnummer 1 bis 10 genau einmal (Pi ab v0.41.4, Koffer-Firmware ab v0.7.21).

Wird `TRIG_ALL` 15 s gehalten, fordert der Koffer den Pi auf, nicht zugeordnete Taster auszuschalten. Als lokale Bestätigung
blinken Rot und Grün anschließend abwechselnd dreimal (Koffer-Firmware ab v0.7.22); die eigentliche Abschaltung benötigt Pi v0.41.3 oder neuer.
Die Firmware puffert außerdem die letzten 24 Diagnoseereignisse im RAM und überträgt den Ringpuffer mit den Statusmeldungen an den Pi (Diagnose-Download ab Pi v0.41.6); ein Neustart setzt ihn zurück. WLAN-Wiederverbindungen werden alle 5 s versucht, und die Diagnose protokolliert Abbruchgrund, Signalstärke und Kanal (ab v0.7.23).

Die Spannungsmessung am Ladeausgang kann einen leeren Platz nicht sicher von einem eingesetzten Akku unterscheiden. Sie wird weiterhin für den Verlauf angezeigt, aber nicht mehr als Überspannungsfehler bewertet (ab v0.7.23).

Im Pi-Dashboard kann zusätzlich ein einzelner belegter Ladeplatz geweckt werden. Der Pi sendet
`"manual_wake":{"id":"<32 Hex-Zeichen>","slot":1}` in der Antwort; der Koffer pulst nur den Pin dieses Slots
für die eingestellte Impulslänge und bestätigt den Abschluss beim nächsten `POST /api/charger` mit `"wake_ack":"<id>"`.
`wake_active` im Status verhindert, dass ein Einzelbefehl während einer laufenden Wecksequenz angenommen wird.

Bei WLAN- oder Pi-Ausfall puffert der Koffer die Spannungsmessungen im RAM (bis zu einer Stunde, Messung alle 5 s)
und liefert sie nach Wiederverbindung in kleinen Paketen nach. Der Pi muss dafür ab v0.41.2 laufen. Der Puffer übersteht
keinen Neustart oder Stromausfall des Koffers; bei längeren Ausfällen werden die ältesten Werte verworfen.

Den Quittiertaster unter 1,5 s drücken quittiert einen Alarm; 1,5 bis unter 15 s halten und dann loslassen weckt alle Taster.
Wird er 15 s gehalten, fordert der Koffer den Pi auf, nur Taster ohne zugeordnete oder vorgemerkte Rolle auszuschalten
(Pi ab v0.41.3). Die Ampel-LEDs sind aktiv-low angesteuert.

## Ampel und Pieper

| Ampel | Bedeutung |
| --- | --- |
| Grün | kein Fehler und kein Ladevorgang (geladen oder nichts eingesetzt, nicht unterscheidbar) |
| Gelb | mindestens ein Platz lädt (rote LED des Lademoduls) |
| Rot dauerhaft | Übertemperatur oder MCP23017 antwortet nicht |
| Rot blinkend | AHT20 fehlt oder liefert keine Werte |

Das TP4056-Modul liefert nur „lädt“ (rote LED). „Taster da“ und „voll“ sind nicht erkennbar, weil der Ausgang ohne Akku bei ca. 4 V liegt.

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

## Update per WLAN (OTA)

Ab v0.4.0 aktualisiert sich der Koffer selbst. Im Dashboard unter **Einstellungen → Ladekoffer-Firmware** wählt man eine Version
aus der Liste der GitHub-Releases (wie bei den Tastern; der Pi lädt die Datei selbst). Alternativ lässt sich eine eigene
`.pio/build/seeed_xiao_esp32c3/firmware.bin` hochladen.
Zu jedem Versions-Tag baut ein GitHub-Workflow (`.github/workflows/release.yml`) die `firmware.bin` als Release.
Die Release-Datei enthält keine Zugangsdaten: ab v0.5.0 schreibt eine per USB mit `src/secrets.h` geflashte Version WLAN, Pi-Adresse
und Schlüssel in den Flash (NVS), und jede spätere Version ohne `secrets.h` nutzt diese Werte.
Der Pi liest die Version aus der Datei (Kennung `KOFFER-FW:vX.Y.Z`)
und bietet sie in der Antwort auf `POST /api/charger` an. Der Koffer lädt sie von `GET /api/charger/firmware`, prüft die MD5-Summe
und startet neu. Nach einem Fehler wartet er 5 Minuten bis zum nächsten Versuch. Mit „Bereitstellung beenden“ nimmt man das Angebot zurück.
Die erste Version mit Update muss einmal per USB geflasht werden.
Die `firmware.bin` darf höchstens 1.310.720 Bytes groß sein (eine OTA-Partition, `min_spiffs.csv`); der Pi lehnt größere Dateien ab.
Vor jedem Tag die Größe prüfen und bei Bedarf Code einsparen (z. B. kein `sscanf`/`printf` mit Fließkomma).
### Startkontakte beim Taster-Update (ab v0.7.17)

Aktualisiert der Pi (ab v0.36.0) gerade Taster, steht in seiner Antwort auf `POST /api/charger` die Restzeit `ota_hold_ms`.
Solange sie größer 0 ist, schaltet der Koffer alle 10 Startkontakte (Wake-Pins an `U5`) ein, damit sich die Taster beim Update nicht
ausschalten. Die Zeit stellt man im Dashboard ein (Standard 300 s, 0 = aus). Ein Weckablauf hat Vorrang; danach werden die Kontakte
wieder gehalten. Bleibt die Meldung aus, enden die Kontakte nach der zuletzt gemeldeten Restzeit.

## Versionierung

Schema `MAJOR.MINOR.PATCH`, die Version steht in `src/config.h` (`FW_VERSION_*`) und wird beim Start und mit jeder Meldung gesendet.
Bei jeder Änderung Version erhöhen und `CHANGELOG.md` ergänzen.

## Aufbau (`src/`)

| Datei | Aufgabe |
| --- | --- |
| `config.h` | Pins, Schwellwerte, Zeiten, Netz, Version |
| `board.*` | I2C, MCP23017, Multiplexer, LEDs, Pieper, Taster |
| `sensors.*` | Slots (lädt/Fehler/Spannung) und AHT20, ohne Wartezeiten |
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
 "light":"green","fault":false,"muted":false,"wake_active":false,
 "charging":0,
 "slots":[{"n":1,"charging":false,"fault":false,"vbat":4.12,"since_s":null}]}
```

`since_s`: Sekunden seit der letzten Änderung des Ladezustands, `null` wenn er sich seit dem Start nicht geändert hat.
`light`: `off`, `green`, `yellow`, `yellow_blink`, `red`, `red_blink`. Die Zeit stempelt der Pi selbst.
Ein optionales `wake_ack`-Feld bestätigt einen ausgeführten Einzel-Weckimpuls; die Pi-Antwort enthält bis zur Bestätigung
`manual_wake` (oder `null`) sowie die konfigurierten Wake-Pins und Impulszeiten.

## Kalibrierung

- `VBAT_DIV_RATIO` (3,0 bei 200 k / 100 k): mit Multimeter gegenprüfen, Kommando `status`.
- `VBAT_PRESENT_V`, `VBAT_FULL_V`, `TEMP_ALARM_ON_C`/`_OFF_C` in `config.h`.
