# Versionsverlauf – Ladekoffer-Controller

| Version | Datum | Änderung |
| --- | --- | --- |
| v0.1.0 | 2026-10-30 | Umstellung auf PlatformIO (XIAO ESP32-C3) und Modulaufbau. Neu: WLAN-Meldung an den Koffer-Pi (`POST /api/charger`). Korrigiert: Teilerfaktor 3,0 (200 k / 100 k) statt 2,0; AHT20 ohne Wartezeiten im `loop()`; MCP23017-Ausfall wird gemeldet und automatisch neu gestartet; Ampel wird auf die LED-Treiber des MCP23017 gespiegelt. |
