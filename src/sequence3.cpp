#include <Arduino.h>
#include "capteurs.h"
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

  if (!orienterVersNord()) {
    sequenceActive = false;
    stopMoteurs();
    return;
  }
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

void sequenceRoseDesVents(float rayonCm) {
  sequenceActive = true;
  if (rayonCm < 3.0f) rayonCm = 3.0f;
  if (rayonCm > 12.0f) rayonCm = 12.0f;

  const int pointsCercle = 48;
  float gainRose = max(kpStylo, 70.0f);

  if (!orienterVersNord()) {
    sequenceActive = false;
    stopMoteurs();
    return;
  }
  resetOdometryStylo();

  styloGoto(0.0f, 0.0f, true, gainRose);
  styloGoto(rayonCm, 0.0f, true, gainRose);

  for (int i = 1; i <= pointsCercle && sequenceActive; i++) {
    float angle = 2.0f * PI * i / pointsCercle;
    styloGoto(rayonCm * cos(angle), rayonCm * sin(angle), true, gainRose);
  }

  styloGoto(0.0f, 0.0f, true, gainRose);

  for (int i = 0; i < 8 && sequenceActive; i++) {
    float angle = PI / 2.0f - i * PI / 4.0f;
    float longueur = (i % 2 == 0) ? rayonCm : rayonCm * 0.78f;
    float pointeX = longueur * cos(angle);
    float pointeY = longueur * sin(angle);

    styloGoto(pointeX, pointeY, true, gainRose);
    if (i == 0) {
      float tete = max(0.8f, rayonCm * 0.18f);
      float demiLargeur = max(0.45f, rayonCm * 0.08f);
      styloGoto(-demiLargeur, rayonCm - tete, true, gainRose);
      styloGoto(pointeX, pointeY, true, gainRose);
      styloGoto(demiLargeur, rayonCm - tete, true, gainRose);
      styloGoto(pointeX, pointeY, true, gainRose);
    }
    styloGoto(0.0f, 0.0f, true, gainRose);
  }

  sequenceActive = false;
  etat = "STOP";
  stopMoteurs();
}
