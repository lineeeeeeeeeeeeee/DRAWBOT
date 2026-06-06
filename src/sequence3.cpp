#include <Arduino.h>
#include "config.h"
#include "deplacements.h"
#include "encodeurs.h"
#include "moteurs.h"
#include "sequence3.h"

void sequenceFleche(float longueurCm) {
  sequenceActive = true;
  if (longueurCm < 5.0f) longueurCm = 5.0f;
  if (longueurCm > 18.0f) longueurCm = 18.0f;

  float teteCm = constrain(longueurCm * 0.30f, 1.2f, 5.0f);
  float demiLargeurCm = constrain(longueurCm * 0.12f, 0.6f, 2.2f);
  float epauleY = longueurCm - teteCm;
  float gainFleche = max(kpStylo, 70.0f);

  resetOdometryStylo();

  styloGoto(0.0f, longueurCm, true, gainFleche);
  styloGoto(-demiLargeurCm, epauleY, true, gainFleche);
  styloGoto(0.0f, longueurCm, true, gainFleche);
  styloGoto(demiLargeurCm, epauleY, true, gainFleche);
  styloGoto(0.0f, longueurCm, true, gainFleche);

  sequenceActive = false;
  etat = "STOP";
  stopMoteurs();
}
