#pragma once

#include "remotebeep.h"
#include "status.h"
#include "wake.h"

// WLAN-Verbindung zum Koffer-Pi (POST /api/charger). Senden laeuft in einem
// eigenen Task, loop() blockiert nie. Ohne WLAN laeuft der Koffer lokal weiter.
namespace netlink {

void begin();
// Uebergibt den aktuellen Status (nicht blockierend, kopiert nur).
void publish(const Status& s, bool wakeActive);
// Neue Weck-Einstellungen aus der Antwort des Pi; true nur einmal je Aenderung
bool takeWakeConfig(wake::Config& out);
// Einzel-Weckbefehl aus der Antwort des Pi; Slot ist 1-basiert.
bool takeManualWakeRequest(String& id, uint8_t& slot);
void retryManualWakeRequest(const String& id, uint8_t slot);
void acknowledgeManualWake(const String& id);
// Neue Ton-Vorgaben aus der Antwort des Pi (bei jeder Antwort neu; false = nichts Neues)
bool takeBeepConfig(remotebeep::Config& out);
// Pi verlangt eingeschaltete Startkontakte (Taster-Update); faellt von selbst ab, wenn der Pi nichts mehr meldet
bool otaHoldActive();
bool connected();
int rssi();
// Letzter HTTP-Code des Sendens (<=0 = Netzwerkfehler, 0 = noch nichts gesendet)
int lastHttpCode();

}  // namespace netlink
