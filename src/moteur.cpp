#include "moteur.h"
#include "roue.h"
#include <Arduino.h>

#define EN_D   23
#define EN_G    4
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16

#define COMPENSATION_GAUCHE 0.80f
#define VITESSE_BASE        150
#define VITESSE_ROTATION    120
#define KP                  2.0f

#define TICKS_PAR_TOUR   1035
#define DIAMETRE_ROUE_CM 9.0f
#define ENTRAXE_CM       14.0f
#define TICKS_PAR_CM 34.5f

void initialiserMoteur() {
  pinMode(EN_D, OUTPUT); pinMode(EN_G, OUTPUT);
  pinMode(IN_1_D, OUTPUT); pinMode(IN_2_D, OUTPUT);
  pinMode(IN_1_G, OUTPUT); pinMode(IN_2_G, OUTPUT);
  digitalWrite(EN_D, HIGH); digitalWrite(EN_G, HIGH);
}

void avancer(int vitesse) {
  int vitesseG = constrain((int)(vitesse * COMPENSATION_GAUCHE), 0, 255);
  analogWrite(IN_1_D, 0); analogWrite(IN_2_D, vitesse);
  analogWrite(IN_1_G, vitesseG); analogWrite(IN_2_G, 0);
}

void avancerVitesses(int vitesseD, int vitesseG) {
  vitesseD = constrain(vitesseD, 0, 255);
  vitesseG = constrain(vitesseG, 0, 255);
  analogWrite(IN_1_D, 0); analogWrite(IN_2_D, vitesseD);
  analogWrite(IN_1_G, vitesseG); analogWrite(IN_2_G, 0);
}

void arreterMoteur() {
  analogWrite(IN_1_D, 0); analogWrite(IN_2_D, 0);
  analogWrite(IN_1_G, 0); analogWrite(IN_2_G, 0);
}

void freinerMoteurCourt(unsigned long dureeMs) {
  analogWrite(IN_1_D, 255); analogWrite(IN_2_D, 255);
  analogWrite(IN_1_G, 255); analogWrite(IN_2_G, 255);
  delay(dureeMs);
  arreterMoteur();
}
