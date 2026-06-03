#ifndef ROUE_H
#define ROUE_H
 
extern volatile long ticksGauche;
extern volatile long ticksDroite;
 
void initialiserRoue();
long lireTicks();
long lireTicksGauche();
long lireTicksDroite();
void resetTicks();
float lireDistance(); 
 
#endif
