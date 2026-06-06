#include <Arduino.h>
#include "config.h"
#include "moteurs.h"
#include "sequence2.h"

void sequenceCercle(float rayonCm) {
  sequenceActive = false;
  stopMoteurs();
  etat = "S2_SIMULATION";

  // La sequence 2 est gardee dans un fichier separe pour le sujet.
  // Pour l'instant, le cercle est simule dans la GUI mais pas encore lance
  // sur le robot afin de ne pas casser les sequences deja stables.
  (void)rayonCm;
}
