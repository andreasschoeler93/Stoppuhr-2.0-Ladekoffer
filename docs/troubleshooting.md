# Fehlersuche

## Ampel rot blinkend
AHT20 nicht gefunden oder keine gültige Messung. SDA/SCL, Pull-ups und Anschluss `J3` prüfen; `status` im seriellen Monitor zeigt `aht=FEHLT`.

## Ampel rot dauerhaft
`status` zeigt die Ursache: Übertemperatur, Slot mit Überspannung (Spalte „Fehler“) oder `I2C: AUSFALL`
(MCP23017 antwortet nicht auf 0x20; die Firmware versucht alle 2 s einen Neustart).

## Slots „da“ flackern
ADC-Eingang schwebt oder Einschwingzeit zu kurz: Teiler und `C-DIV` prüfen. Bei Bedarf `ADC_SAMPLES` erhöhen.

## Spannung stimmt nicht
`VBAT_DIV_RATIO` mit dem Multimeter abgleichen (Soll 3,0 bei 200 k / 100 k).

## Kein Kontakt zum Pi
`status` zeigt WLAN und den letzten HTTP-Code (`200` = angekommen, `<=0` = Netzwerkfehler).
SSID exakt prüfen (das WLAN ist versteckt) und die Adresse `SECRET_PI_IP` (gemeinsame Adresse `192.168.50.10`).
