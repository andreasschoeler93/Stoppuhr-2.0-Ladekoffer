# Troubleshooting

## Ampel zeigt rot blinkend (Sensorfault)
- AHT20 nicht gefunden / keine gültige Messung
- Prüfen: SDA/SCL, Pullups, Adresse/Modul
- Test: `status` im Serial Monitor

## Slots "present" flackern
- ADC/MUX Eingang floatet oder zu wenig Settling
- Maßnahmen:
  - Hardware: Pull-Down am ADC (z. B. 100k) oder sauberere Teiler
  - Software: Median/Moving-Average Filter (bei Bedarf nachrüstbar)

## Slots 9/10 werden falsch gelesen
- Prüfen, ob U4 wirklich Kanäle X0/X1 nutzt
- Falls andere Kanäle: Sketch anpassen (Mapping in `readSlotVbat()`)

## MCP23017 nicht gefunden
- I²C prüfen (SDA/SCL), Pullups, Adresse 0x20 (A0..A2 auf GND)
