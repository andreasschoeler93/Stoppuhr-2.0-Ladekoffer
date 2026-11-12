# Fehlersuche

## Ampel rot blinkend
AHT20 nicht gefunden oder keine gültige Messung. SDA/SCL, Pull-ups und Anschluss `J3` prüfen; `status` im seriellen Monitor zeigt `aht=FEHLT`.

## Ampel rot dauerhaft
`status` zeigt die Ursache: Übertemperatur oder `I2C: AUSFALL` (MCP23017 antwortet nicht auf 0x20; die Firmware versucht alle 2 s einen Neustart).

## Slots „da“ flackern
ADC-Eingang schwebt oder Einschwingzeit zu kurz: Teiler und `C-DIV` prüfen. Bei Bedarf `ADC_SAMPLES` erhöhen.

## Spannung stimmt nicht
`VBAT_DIV_RATIO` mit dem Multimeter abgleichen (Soll 3,0 bei 200 k / 100 k).

## Kein Kontakt zum Pi
`status` zeigt WLAN und den letzten HTTP-Code (`200` = angekommen, `<=0` = Netzwerkfehler).
Ab Firmware v0.7.23 stehen WLAN-Abbruchgrund, Signalstärke und Funkkanal zusätzlich im Geräte-Log auf der Pi-Oberfläche.
SSID exakt prüfen und sicherstellen, dass der Zugangspunkt 2,4 GHz anbietet. Wenn möglich, testweise SSID sichtbar,
einen festen, wenig belegten 2,4-GHz-Kanal (1, 6 oder 11) und 20-MHz-Kanalbreite verwenden; Mesh-/Band-Steering
und automatische Kanalwechsel für dieses WLAN vermeiden. Die RSSI-Anzeige sollte am Koffer deutlich besser als −70 dBm sein.
Auch USB-3-Geräte, Metall und Netzteile direkt neben der Antenne können stören; Abstand von etwa 0,5–1 m testen.
Adresse `SECRET_PI_IP` (gemeinsame Adresse `192.168.50.10`) kontrollieren.

Die Spannungsmessung an einem leeren Ladeausgang kann hoch erscheinen und ist kein Fehlerindikator; nur die sicher messbaren Zustände
(Ladestatus, Temperatur und Gerätekommunikation) lösen Fehleranzeigen aus.
