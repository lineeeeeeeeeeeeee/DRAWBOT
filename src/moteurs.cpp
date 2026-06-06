#include <Arduino.h>
#include "config.h"
#include "moteurs.h"

void setupMoteurs() {
  pinMode(EN_D, OUTPUT);
  pinMode(EN_G, OUTPUT);
  pinMode(IN_1_D, OUTPUT);
  pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT);
  pinMode(IN_2_G, OUTPUT);
  digitalWrite(EN_D, HIGH);
  digitalWrite(EN_G, HIGH);
  stopMoteurs();
}

void moteurs(int vitesseGauche, int vitesseDroite) {
  vitesseGauche = constrain(vitesseGauche, -255, 255);
  vitesseDroite = constrain(vitesseDroite, -255, 255);

  if (vitesseDroite >= 0) {
    analogWrite(IN_1_D, 0);
    analogWrite(IN_2_D, vitesseDroite);
  } else {
    analogWrite(IN_1_D, -vitesseDroite);
    analogWrite(IN_2_D, 0);
  }

  if (vitesseGauche >= 0) {
    analogWrite(IN_1_G, vitesseGauche);
    analogWrite(IN_2_G, 0);
  } else {
    analogWrite(IN_1_G, 0);
    analogWrite(IN_2_G, -vitesseGauche);
  }
}

void stopMoteurs() {
  moteurs(0, 0);
}

void pauseFranche() {
  stopMoteurs();
  delay(PAUSE_MS);
}
