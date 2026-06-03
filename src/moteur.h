#ifndef MOTEUR_H
#define MOTEUR_H

void initialiserMoteur();
void avancer(int vitesse);
void avancerVitesses(int vitesseD, int vitesseG);
void arreterMoteur();
void freinerMoteurCourt(unsigned long dureeMs);
void avancerCm(float cm);
void tournerDegres(float degres);
void sequenceEscalier();
extern const float TICKS_PAR_CM_VAL;
extern const float ENTRAXE_CM_VAL;

#endif
