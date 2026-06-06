#include <Arduino.h>
#include "config.h"
#include "encodeurs.h"

volatile long ticksGauche = 0;
volatile long ticksDroite = 0;

float robotX = 0.0f;
float robotY = -DISTANCE_STYLO_CM;
float robotTheta = PI / 2.0f;
long odoLastG = 0;
long odoLastD = 0;

void IRAM_ATTR compterGauche() { ticksGauche++; }
void IRAM_ATTR compterDroite() { ticksDroite++; }

void setupEncodeurs() {
  pinMode(ENC_G_A, INPUT);
  pinMode(ENC_D_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_G_A), compterGauche, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_A), compterDroite, RISING);
}

long lireTicksGauche() {
  noInterrupts();
  long v = ticksGauche;
  interrupts();
  return v;
}

long lireTicksDroite() {
  noInterrupts();
  long v = ticksDroite;
  interrupts();
  return v;
}

void resetTicks() {
  noInterrupts();
  ticksGauche = 0;
  ticksDroite = 0;
  interrupts();
}

long cmEnTicks(float cm) {
  return (long)(abs(cm) * TICKS_PAR_CM);
}

long degEnTicks(float deg) {
  float arcCm = (abs(deg) / 360.0f) * PI * ENTRAXE_CM * correctionRot;
  return cmEnTicks(arcCm);
}

float normaliserRad(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

void resetOdometryStylo() {
  resetTicks();
  odoLastG = 0;
  odoLastD = 0;
  robotTheta = PI / 2.0f;
  robotX = 0.0f;
  robotY = -DISTANCE_STYLO_CM;
}

void updateOdometryStylo() {
  long g = lireTicksGauche();
  long d = lireTicksDroite();
  long dgTicks = g - odoLastG;
  long ddTicks = d - odoLastD;
  odoLastG = g;
  odoLastD = d;

  float distG = dgTicks / TICKS_PAR_CM;
  float distD = ddTicks / TICKS_PAR_CM;
  float distC = (distG + distD) * 0.5f;
  float dTheta = (distD - distG) / ENTRAXE_CM;

  if (abs(dTheta) < 1e-5f) {
    robotX += distC * cos(robotTheta);
    robotY += distC * sin(robotTheta);
  } else {
    float rayon = distC / dTheta;
    float nouveauTheta = robotTheta + dTheta;
    robotX += rayon * (sin(nouveauTheta) - sin(robotTheta));
    robotY -= rayon * (cos(nouveauTheta) - cos(robotTheta));
    robotTheta = nouveauTheta;
  }
}

float styloX() {
  return robotX + DISTANCE_STYLO_CM * cos(robotTheta);
}

float styloY() {
  return robotY + DISTANCE_STYLO_CM * sin(robotTheta);
}
