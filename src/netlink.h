#pragma once

#include "remotebeep.h"
#include "status.h"
#include "wake.h"

// WLAN-Verbindung zum Koffer-Pi (POST /api/charger). Senden laeuft in einem
// eigenen Task, loop() blockiert nie. Ohne WLAN laeuft der Koffer lokal weiter.
namespace netlink {

void begin();
// Uebergibt den aktuellen Status (nicht blockierend, kopiert nur).
void publish(const Status& s);
// Neue Weck-Einstellungen aus der Antwort des Pi; true nur einmal je Aenderung
bool takeWakeConfig(wake::Config& out);
// Neue Ton-Vorgaben aus der Antwort des Pi (bei jeder Antwort neu; false = nichts Neues)
bool takeBeepConfig(remotebeep::Config& out);
bool connected();
int rssi();
// Letzter HTTP-Code des Sendens (<=0 = Netzwerkfehler, 0 = noch nichts gesendet)
int lastHttpCode();

}  // namespace netlink
