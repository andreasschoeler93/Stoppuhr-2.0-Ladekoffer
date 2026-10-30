#pragma once

#include "status.h"

// Ampel, Pieper und Quittiertaster; leitet aus der Messung den Gesamtstatus ab.
namespace signals {

void begin();
// Nicht blockierend; gibt den aktuellen Gesamtstatus zurueck.
const Status& update(const Measurements& m);
// Alarm quittieren (z. B. ueber den seriellen Befehl)
void mute(bool on);

}  // namespace signals
