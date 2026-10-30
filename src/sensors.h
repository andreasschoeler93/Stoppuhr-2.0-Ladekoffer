#pragma once

#include "status.h"

// Slots (Anwesenheit, Laden, voll) und Temperatur/Feuchte (AHT20) erfassen.
namespace sensors {

void begin();
// Nicht blockierend: arbeitet nur, wenn ein Intervall abgelaufen ist.
void update();
const Measurements& get();

}  // namespace sensors
