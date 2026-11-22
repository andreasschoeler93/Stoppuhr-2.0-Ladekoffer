# Fehlersuche

## Ampel rot blinkend
AHT20 nicht gefunden oder keine gültige Messung. Während der ersten Messung nach dem Start kann Rot kurz blinken;
bleibt die Anzeige bestehen, SDA/SCL, Pull-ups und Anschluss `J3` prüfen. `status` im seriellen Monitor zeigt `aht=FEHLT`.

## Ampel rot dauerhaft
`status` zeigt die Ursache: Übertemperatur oder `I2C: AUSFALL` (MCP23017 antwortet nicht auf 0x20; die Firmware versucht alle 2 s einen Neustart).
Bei einem Ausfall von U2 ist auch der darüber angeschlossene Pieper nicht zuverlässig verfügbar.
Kurze optische Rückmeldungen dürfen die rote LED vorübergehend unterbrechen; anschließend erscheint der Fehler wieder.

## Schlafanforderung führt nicht zum Ausschalten
Mindestens 5 s halten; die Töne bei 3 und 4 s warnen vor der Anforderung. Der längere Ton bei 5 s bestätigt nur,
dass der Koffer sie vorgemerkt hat. Der Pi muss erreichbar sein und mindestens v0.41.3 verwenden; nur Taster ohne
zugeordnete oder vorgemerkte Rolle sind betroffen. Die Anforderung wird bis zur Bestätigung im RAM gehalten, geht aber
bei einem Neustart verloren. Ein nicht quittierter lokaler Fehler kann die Tastertöne überlagern; optische
Taster-Rückmeldungen unterbrechen die rote Fehleranzeige kurz.

## Dashboard-Meldung blinkt nicht am Koffer
Für Meldungen bei abgeschaltetem oder quittiertem Dashboard-Ton sind Pi v0.44.14 und Koffer v0.7.31 erforderlich.
Ein geöffnetes Dashboard muss die Meldungen an den Pi übertragen, und der Koffer muss mit dem Pi verbunden sein.
Neue Meldungen erscheinen nur kurz; mehrere neue Meldungen gleicher Farbe können zusammengefasst werden. Während
der Taster gehalten wird, wartet die Anzeige bis zum Loslassen.

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
