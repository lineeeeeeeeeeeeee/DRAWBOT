#include <Arduino.h>
#include "config.h"
#include "encodeurs.h"
#include "moteurs.h"
#include "deplacements.h"

bool styloGoto(float cibleX, float cibleY, bool marcheArriereAutorisee, float gainStylo) {
  etat = "STYLO";
  updateOdometryStylo();
  float departX = styloX();
  float departY = styloY();
  float segX = cibleX - departX;
  float segY = cibleY - departY;
  float longueur = sqrt(segX * segX + segY * segY);
  if (longueur < 0.5f) return true;

  float ux = segX / longueur;
  float uy = segY / longueur;
  float nx = -uy;
  float ny = ux;

  unsigned long t0 = millis();
  while (sequenceActive) {
    updateOdometryStylo();

    float px = styloX();
    float py = styloY();
    float relX = px - departX;
    float relY = py - departY;
    float avance = relX * ux + relY * uy;
    float ecart = relX * nx + relY * ny;
    float restant = longueur - avance;

    if (restant < 0.8f) break;
    if (millis() - t0 > 14000) break;

    const float LOOKAHEAD_CM = 12.0f;
    float angleSegment = atan2(uy, ux);
    float angleDesire = angleSegment - atan2(ecart, LOOKAHEAD_CM);
    float erreurCap = normaliserRad(angleDesire - robotTheta);
    int sens = 1;

    if (marcheArriereAutorisee && abs(erreurCap) > PI / 2.0f) {
      sens = -1;
      erreurCap = normaliserRad(angleDesire + PI - robotTheta);
    }

    int base = restant < 5.0f ? max(50, pwmStylo - 15) : pwmStylo;
    float gainActif = gainStylo > 0.0f ? gainStylo : kpStylo;
    float correction = constrain(gainActif * erreurCap, -35.0f, 35.0f);

    int pwmG = sens * constrain((int)(base - correction), 45, 115);
    int pwmD = sens * constrain((int)(base + correction), 45, 115);

    moteurs(pwmG, pwmD);
    delay(10);
  }

  pauseFranche();
  return sequenceActive;
}

void deplacerCm(float cm, int sens) {
  etat = sens > 0 ? "AVANCER" : "RECULER";
  resetTicks();
  long cible = cmEnTicks(cm);
  unsigned long t0 = millis();
  long derniereErreur = 0;

  while (sequenceActive) {
    long g = lireTicksGauche();
    long d = lireTicksDroite();
    if ((g + d) / 2 >= cible) break;
    if (millis() - t0 > 12000) break;

    long erreur = g - d;
    long derivee = erreur - derniereErreur;
    derniereErreur = erreur;

    float correction = kpLigne * erreur + kdLigne * derivee;
    int vG = sens * constrain((int)(pwmBase - correction), PWM_MIN, PWM_MAX);
    int vD = sens * constrain((int)(pwmBase + correction), PWM_MIN, PWM_MAX);
    moteurs(vG, vD);
    delay(5);
  }

  pauseFranche();
}

void avancerCm(float cm) {
  deplacerCm(cm, 1);
}

void reculerCm(float cm) {
  deplacerCm(cm, -1);
}

void tournerDeg(float deg) {
  etat = deg > 0 ? "DROITE" : "GAUCHE";
  resetTicks();
  long cible = degEnTicks(deg);
  int sens = deg > 0 ? 1 : -1;
  unsigned long t0 = millis();

  while (sequenceActive) {
    long g = lireTicksGauche();
    long d = lireTicksDroite();
    if ((g + d) / 2 >= cible) break;
    if (millis() - t0 > 8000) break;

    moteurs(sens * pwmRot, -sens * pwmRot);
    delay(5);
  }

  pauseFranche();
}
