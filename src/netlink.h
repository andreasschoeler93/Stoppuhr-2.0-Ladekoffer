#pragma once

#include "status.h"

// WLAN-Verbindung zum Koffer-Pi (POST /api/charger). Senden laeuft in einem
// eigenen Task, loop() blockiert nie. Ohne WLAN laeuft der Koffer lokal weiter.
namespace netlink {

void begin();
// Uebergibt den aktuellen Status (nicht blockierend, kopiert nur).
void publish(const Status& s);
bool connected();
int rssi();
// Letzter HTTP-Code des Sendens (<=0 = Netzwerkfehler, 0 = noch nichts gesendet)
int lastHttpCode();

}  // namespace netlink
