# Versionsverlauf – Ladekoffer-Controller

| Version | Datum | Änderung |
| --- | --- | --- |
| v0.2.0 | 2026-10-31 | Taster aufwecken: Quittiertaster `TRIG_ALL` lang halten (≥ 1,5 s) steuert die Magnet-Trigger an `U5` (0x21) der Reihe nach von Slot 1 bis 10 an (aktiv gegen GND, je 3 s, Abstand 1 s). Pins, Impulslänge und Abstand stellt man im Dashboard ein (Pi ab v0.17.0, Übergabe in der Antwort auf `POST /api/charger`). Kurzer Druck quittiert weiter den Pieper. Serieller Befehl `wake`. |
| v0.1.1 | 2026-10-30 | WLAN: Verbindungsversuche (mit Statuscode), Abbrüche und erfolgreiche Verbindung erscheinen im seriellen Monitor; es wird ausschließlich das Koffer-WLAN verwendet. |
| v0.1.0 | 2026-10-30 | Umstellung auf PlatformIO (XIAO ESP32-C3) und Modulaufbau. Neu: WLAN-Meldung an den Koffer-Pi (`POST /api/charger`). Korrigiert: Teilerfaktor 3,0 (200 k / 100 k) statt 2,0; AHT20 ohne Wartezeiten im `loop()`; MCP23017-Ausfall wird gemeldet und automatisch neu gestartet; Ampel wird auf die LED-Treiber des MCP23017 gespiegelt. |
