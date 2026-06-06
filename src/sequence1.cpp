#include <Arduino.h>
#include "config.h"
#include "deplacements.h"
#include "encodeurs.h"
#include "moteurs.h"
#include "sequence1.h"

void sequenceEscalier() {
  sequenceActive = true;
  resetOdometryStylo();
  styloGoto(0.0f, 20.0f);
  styloGoto(-10.0f, 20.0f);
  styloGoto(-10.0f, 60.0f);
  sequenceActive = false;
  etat = "STOP";
  stopMoteurs();
}

void sequenceMarches(float largeurCm, float hauteurCm, int marches) {
  sequenceActive = true;
  if (marches < 1) marches = 1;
  if (marches > 6) marches = 6;

  for (int i = 0; i < marches && sequenceActive; i++) {
    avancerCm(largeurCm);
    tournerDeg(-90);
    avancerCm(hauteurCm);
    tournerDeg(90);
  }

  sequenceActive = false;
  etat = "STOP";
  stopMoteurs();
}
