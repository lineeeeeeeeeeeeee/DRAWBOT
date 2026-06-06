#pragma once

void setupEncodeurs();
long lireTicksGauche();
long lireTicksDroite();
void resetTicks();
long cmEnTicks(float cm);
long degEnTicks(float deg);

void resetOdometryStylo();
void updateOdometryStylo();
float styloX();
float styloY();
float normaliserRad(float a);

extern float robotX;
extern float robotY;
extern float robotTheta;
