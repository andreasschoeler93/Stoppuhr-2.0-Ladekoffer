#pragma once

#include <Arduino.h>

// Ton fuer Dashboard-Meldungen ueber den Pieper des Koffers (Vorgaben kommen vom Pi):
// gelb = kurzer Piepton bei neuer Meldung, rot = langer Ton im festen Abstand, solange eine Meldung ansteht.
namespace remotebeep {

struct Config {
  bool yellowOn;
  uint32_t yellowSeq;  // aendert sich bei jeder neuen gelben Meldung
  bool redOn;
  uint16_t redMs;
  uint32_t redEveryMs;
};

void configure(const Config& c);
// Soll der Pieper wegen einer Dashboard-Meldung jetzt an sein? Regelmaessig aufrufen.
bool active(uint32_t now);
// Quittiert: bleibt still, bis eine neue Meldung kommt (neue gelbe Meldung oder rot erneut an)
void mute();
bool muted();

}  // namespace remotebeep
