#include "roue.h"
#include <Arduino.h>

#define ENC_G_A 32
#define ENC_D_A 27

volatile long ticksGauche = 0;
volatile long ticksDroite = 0;

#define TICKS_PAR_CM_CALIBRE 34.5f

void IRAM_ATTR compterGauche() {
  ticksGauche++;
}

void IRAM_ATTR compterDroite() {
  ticksDroite++;
}

void initialiserRoue() {
  pinMode(ENC_G_A, INPUT);
  pinMode(ENC_D_A, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_G_A), compterGauche, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D_A), compterDroite, RISING);
}

long lireTicks() {
  noInterrupts();
  long gauche = ticksGauche;
  long droite = ticksDroite;
  interrupts();

  return (gauche + droite) / 2;
}

long lireTicksGauche() {
  noInterrupts();
  long ticks = ticksGauche;
  interrupts();

  return ticks;
}

long lireTicksDroite() {
  noInterrupts();
  long ticks = ticksDroite;
  interrupts();

  return ticks;
}

void resetTicks() {
  noInterrupts();
  ticksGauche = 0;
  ticksDroite = 0;
  interrupts();
}

float lireDistance() {
  return lireTicks() / TICKS_PAR_CM_CALIBRE;
}
