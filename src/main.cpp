#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "moteur.h"
#include "roue.h"
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define ADDR_IMU 0x6B
#define ADDR_MAG 0x1E

#define SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define TICKS_PAR_TOUR  1035      
#define DIAMETRE_ROUE   9.0f    
#define TICKS_PAR_CM 34.5f
#define ENTRAXE_CM      14.0f    

#define VITESSE        80
#define VITESSE_ROT  90
#define VITESSE_CERCLE 75
#define VITESSE_CERCLE_MIN 45
#define VITESSE_PETIT_CERCLE 110
#define VITESSE_PETIT_CERCLE_MIN 82
#define DISTANCE_STYLO_AXE_CM 13.0f
#define RAYON_MODE_CONTINU_MIN 13.0f
#define CALIBRATION_RAYON_CERCLE 0.75f
#define FREINAGE_CERCLE_MS 80
#define SEGMENTS_PETIT_CERCLE_DEFAUT 48
#define CORRECTION_DEGRES 0.65f
#define PAUSE_SEGMENT_MS 18
#define KP        1.2f  
#define TICKS_MANUEL 999999L
#define PAUSE_SEQUENCE_MS 300

long lastTicksG = 0;
long lastTicksD = 0;
unsigned long lastTime = 0;

float vitesseG = 0;
float vitesseD = 0;


enum Etat { STOP, AVANCER, RECULER, TOURNER, CERCLE, PAUSE, SEQUENCE };
Etat etatActuel  = STOP;
Etat etatSuivant = STOP;

float         cibleTicks  = 0;
float         cibleDeg    = 0;
float         cibleTicksGauche = 0;
float         cibleTicksDroite = 0;
float         cibleDeltaTicksCercle = 0;
int           sensCercleGauche = 1;
int           sensCercleDroite = 1;
float         pulseCercleGauche = 0;
float         pulseCercleDroite = 0;
bool          petitCercleActif = false;
float         rayonPetitCercle = 0;
float         anglePetitCercle = 0;
float         orientationPetitCercle = 0;
int           segmentPetitCercle = 0;
int           totalSegmentsPetitCercle = SEGMENTS_PETIT_CERCLE_DEFAUT;
int           etapeSeq    = 0;
unsigned long pauseDebut  = 0;
unsigned long pauseDuree  = 0;
bool          pauseSegmentActif = false;
unsigned long pauseSegmentDebut = 0;


BLEServer         *pServer           = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool               deviceConnected   = false;

float cmEnTicks(float cm) {
  return cm * TICKS_PAR_CM;
}

float degEnTicks(float deg) {
  float arcCm = (abs(deg) / 360.0f) * PI * ENTRAXE_CM;
  return cmEnTicks(arcCm) * CORRECTION_DEGRES;
}

long ticksMoyens() {
  return lireTicks();
}

const char* nomEtat() {
  switch (etatActuel) {
    case STOP:     return "STOP";
    case AVANCER:  return "AVANCER";
    case RECULER:  return "RECULER";
    case TOURNER:  return "TOURNER";
    case CERCLE:   return "CERCLE";
    case PAUSE:    return "PAUSE";
    case SEQUENCE: return "SEQUENCE";
    default:       return "INCONNU";
  }
}

bool distanceValide(float cm) {
  return cm > 0 && cm <= 500;
}

bool angleValide(float deg) {
  return deg != 0 && abs(deg) <= 360;
}

bool rayonCercleValide(float rayonCm) {
  return rayonCm >= 2 && rayonCm <= 20;
}

void preparerSegmentPetitCercle() {
  if (segmentPetitCercle >= totalSegmentsPetitCercle) {
    freinerMoteurCourt(FREINAGE_CERCLE_MS);
    etatActuel = STOP;
    return;
  }

  resetTicks();

  float dTheta = 2.0f * PI / totalSegmentsPetitCercle;
  float ux = cos(orientationPetitCercle);
  float uy = sin(orientationPetitCercle);
  float uPerpX = -uy;
  float uPerpY = ux;
  float tangentX = -sin(anglePetitCercle);
  float tangentY = cos(anglePetitCercle);

  float composanteAvant = tangentX * ux + tangentY * uy;
  float composanteRotation = tangentX * uPerpX + tangentY * uPerpY;
  float dPhi = (rayonPetitCercle / DISTANCE_STYLO_AXE_CM) * composanteRotation * dTheta;
  float dCentre = rayonPetitCercle * composanteAvant * dTheta;

  float distanceGauche = dCentre - dPhi * (ENTRAXE_CM / 2.0f);
  float distanceDroite = dCentre + dPhi * (ENTRAXE_CM / 2.0f);

  sensCercleGauche = distanceGauche >= 0 ? 1 : -1;
  sensCercleDroite = distanceDroite >= 0 ? 1 : -1;
  cibleTicksGauche = cmEnTicks(abs(distanceGauche));
  cibleTicksDroite = cmEnTicks(abs(distanceDroite));
  pulseCercleGauche = 0;
  pulseCercleDroite = 0;

  anglePetitCercle += dTheta;
  orientationPetitCercle += dPhi;
  segmentPetitCercle++;
}

void demarrerAvancer(float cm) {
  if (!distanceValide(cm)) return;
  resetTicks();
  cibleTicks = cmEnTicks(cm);
  etapeSeq   = 0;
  etatActuel = AVANCER;
}

void demarrerReculer(float cm) {
  if (!distanceValide(cm)) return;
  resetTicks();
  cibleTicks = cmEnTicks(cm);
  etapeSeq   = 0;
  etatActuel = RECULER;
}

void demarrerTourner(float deg) {
  if (!angleValide(deg)) return;
  resetTicks();
  cibleDeg   = deg;
  cibleTicks = degEnTicks(deg);
  etapeSeq   = 0;
  etatActuel = TOURNER;
}

void demarrerCercle(float rayonCm) {
  if (!rayonCercleValide(rayonCm)) return;

  if (rayonCm < RAYON_MODE_CONTINU_MIN) {
    petitCercleActif = true;
    rayonPetitCercle = rayonCm;
    anglePetitCercle = 0;
    orientationPetitCercle = PI / 2.0f;
    segmentPetitCercle = 0;
    if (rayonCm <= 3) {
      totalSegmentsPetitCercle = 24;
    } else if (rayonCm <= 6) {
      totalSegmentsPetitCercle = 36;
    } else if (rayonCm <= 9) {
      totalSegmentsPetitCercle = 48;
    } else {
      totalSegmentsPetitCercle = 60;
    }
    pauseSegmentActif = false;
    etapeSeq = 0;
    etatActuel = CERCLE;
    preparerSegmentPetitCercle();
    return;
  }

  petitCercleActif = false;
  resetTicks();

  float rayonCalibre = rayonCm * CALIBRATION_RAYON_CERCLE;
  float rayonRoueGauche = rayonCalibre - (ENTRAXE_CM / 2.0f);
  float rayonRoueDroite = rayonCalibre + (ENTRAXE_CM / 2.0f);
  float distanceGauche = 2.0f * PI * rayonRoueGauche;
  float distanceDroite = 2.0f * PI * rayonRoueDroite;

  sensCercleGauche = distanceGauche >= 0 ? 1 : -1;
  sensCercleDroite = distanceDroite >= 0 ? 1 : -1;
  cibleTicksGauche = cmEnTicks(abs(distanceGauche));
  cibleTicksDroite = cmEnTicks(abs(distanceDroite));
  cibleDeltaTicksCercle = cmEnTicks(2.0f * PI * ENTRAXE_CM);
  pulseCercleGauche = 0;
  pulseCercleDroite = 0;
  cibleTicks = max(cibleTicksGauche, cibleTicksDroite);
  etapeSeq   = 0;
  etatActuel = CERCLE;
}

void demarrerPause(unsigned long ms, Etat suite) {
  arreterMoteur();
  pauseDebut  = millis();
  pauseDuree  = ms;
  etatSuivant = suite;
  etatActuel  = PAUSE;
}

void demarrerManuel(Etat etat, float directionDeg = 0) {
  resetTicks();
  cibleTicks = TICKS_MANUEL;
  cibleDeg   = directionDeg;
  etapeSeq   = 0;
  etatActuel = etat;
}

void arreterRobot() {
  arreterMoteur();
  etapeSeq         = 0;
  pauseSegmentActif = false;
  etatActuel       = STOP;
}

void prochaineEtapeSequence() {
  etapeSeq++;
  switch (etapeSeq) {
    case 1: resetTicks(); cibleTicks = cmEnTicks(20);       etatActuel = AVANCER; break;
    case 2: resetTicks(); cibleDeg = -90; cibleTicks = degEnTicks(90); etatActuel = TOURNER; break;
    case 3: resetTicks(); cibleTicks = cmEnTicks(10);       etatActuel = AVANCER; break;
    case 4: resetTicks(); cibleDeg = 90;  cibleTicks = degEnTicks(90); etatActuel = TOURNER; break;
    case 5: resetTicks(); cibleTicks = cmEnTicks(40);       etatActuel = AVANCER; break;
    default: arreterMoteur(); etapeSeq = 0; etatActuel = STOP; break;
  }
}

void calculerVitesse() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  if (dt <= 0) return;

  long deltaG = ticksGauche - lastTicksG;
  long deltaD = ticksDroite - lastTicksD;

  vitesseG = deltaG / dt;
  vitesseD = deltaD / dt;

  lastTicksG = ticksGauche;
  lastTicksD = ticksDroite;
  lastTime = now;
}

void appliquerAvancer() {
  long gauche = lireTicksGauche();
  long droite = lireTicksDroite();
  int erreur = gauche - droite;
  int vD = constrain(VITESSE + (int)(erreur * KP), 0, 255);
  int vG = constrain(VITESSE - (int)(erreur * KP), 0, 255);

  analogWrite(19, 0); analogWrite(18, vD);
  analogWrite(17, vG); analogWrite(16, 0);
}

void appliquerReculer() {
  long gauche = lireTicksGauche();
  long droite = lireTicksDroite();
  int erreur = gauche - droite;
  int vD = constrain(VITESSE + (int)(erreur * KP), 0, 255);
  int vG = constrain(VITESSE - (int)(erreur * KP), 0, 255);

  analogWrite(19, vD); analogWrite(18, 0);
  analogWrite(17, 0);  analogWrite(16, vG);
}

void appliquerTourner() {
  if (cibleDeg > 0) {
    analogWrite(19, VITESSE_ROT); analogWrite(18, 0);
    analogWrite(17, VITESSE_ROT); analogWrite(16, 0);
  } else {
    analogWrite(19, 0); analogWrite(18, VITESSE_ROT);
    analogWrite(17, 0); analogWrite(16, VITESSE_ROT);
  }
}

void commanderMoteurs(int vitesseGauche, int vitesseDroite) {
  vitesseGauche = constrain(vitesseGauche, -255, 255);
  vitesseDroite = constrain(vitesseDroite, -255, 255);

  if (vitesseDroite >= 0) {
    analogWrite(19, 0);
    analogWrite(18, vitesseDroite);
  } else {
    analogWrite(19, -vitesseDroite);
    analogWrite(18, 0);
  }

  if (vitesseGauche >= 0) {
    analogWrite(17, vitesseGauche);
    analogWrite(16, 0);
  } else {
    analogWrite(17, 0);
    analogWrite(16, -vitesseGauche);
  }
}

int vitesseCerclePulse(float vitesseDemandee, float &accumulateur, int vitesseMin = VITESSE_CERCLE_MIN, int vitesseMax = VITESSE_CERCLE) {
  if (vitesseDemandee <= 0) return 0;

  if (vitesseDemandee >= vitesseMin) {
    return constrain((int)vitesseDemandee, vitesseMin, vitesseMax);
  }

  accumulateur += vitesseDemandee / vitesseMin;
  if (accumulateur >= 1.0f) {
    accumulateur -= 1.0f;
    return vitesseMin;
  }

  return 0;
}

void appliquerCercle() {
  if (petitCercleActif) {
    // Pause non-bloquante entre segments pour éliminer l'overshoot par inertie
    if (pauseSegmentActif) {
      if (millis() - pauseSegmentDebut >= PAUSE_SEGMENT_MS) {
        pauseSegmentActif = false;
        preparerSegmentPetitCercle();
      }
      return;
    }

    long ticksG = lireTicksGauche();
    long ticksD = lireTicksDroite();
    float cibleMax = max(cibleTicksGauche, cibleTicksDroite);

    if (cibleMax <= 2 || max(ticksG, ticksD) >= (long)cibleMax) {
      arreterMoteur();
      pauseSegmentActif = true;
      pauseSegmentDebut = millis();
      return;
    }

    float baseG = VITESSE_PETIT_CERCLE * cibleTicksGauche / cibleMax;
    float baseD = VITESSE_PETIT_CERCLE * cibleTicksDroite / cibleMax;

    // Synchronisation proportionnelle pour que les deux roues arrivent ensemble
    float progressionG = cibleTicksGauche > 2 ? (float)ticksG / cibleTicksGauche : 1.0f;
    float progressionD = cibleTicksDroite > 2 ? (float)ticksD / cibleTicksDroite : 1.0f;
    float correction = (progressionG - progressionD) * 30.0f;
    float demandeG = constrain(baseG - correction, 0.0f, (float)VITESSE_PETIT_CERCLE);
    float demandeD = constrain(baseD + correction, 0.0f, (float)VITESSE_PETIT_CERCLE);

    int vitesseG = vitesseCerclePulse(demandeG, pulseCercleGauche, VITESSE_PETIT_CERCLE_MIN, VITESSE_PETIT_CERCLE);
    int vitesseD = vitesseCerclePulse(demandeD, pulseCercleDroite, VITESSE_PETIT_CERCLE_MIN, VITESSE_PETIT_CERCLE);

    commanderMoteurs(vitesseG * sensCercleGauche, vitesseD * sensCercleDroite);
    return;
  }

  long ticksG = lireTicksGauche();
  long ticksD = lireTicksDroite();
  long ticksSignesG = ticksG * sensCercleGauche;
  long ticksSignesD = ticksD * sensCercleDroite;
  long deltaTicks = ticksSignesD - ticksSignesG;

  if (abs(deltaTicks) >= (long)cibleDeltaTicksCercle) {
    freinerMoteurCourt(FREINAGE_CERCLE_MS);
    etatActuel = STOP;
    return;
  }

  float cibleMax = max(cibleTicksGauche, cibleTicksDroite);
  float baseG = 0;
  float baseD = 0;

  if (cibleTicksGauche > 0) {
    baseG = VITESSE_CERCLE * cibleTicksGauche / cibleMax;
  }
  if (cibleTicksDroite > 0) {
    baseD = VITESSE_CERCLE * cibleTicksDroite / cibleMax;
  }

  float progressionG = cibleTicksGauche > 1 ? ticksG / cibleTicksGauche : 1.0f;
  float progressionD = cibleTicksDroite > 1 ? ticksD / cibleTicksDroite : 1.0f;
  float correction = (progressionG - progressionD) * 25.0f;

  float demandeG = constrain(baseG - correction, 0, VITESSE_CERCLE);
  float demandeD = constrain(baseD + correction, 0, VITESSE_CERCLE);
  int vitesseG = vitesseCerclePulse(demandeG, pulseCercleGauche);
  int vitesseD = vitesseCerclePulse(demandeD, pulseCercleDroite);

  commanderMoteurs(vitesseG * sensCercleGauche, vitesseD * sensCercleDroite);
}

void terminerMouvementSiAtteint() {
  if (ticksMoyens() < (long)cibleTicks) return;

  arreterMoteur();
  if (etapeSeq > 0) demarrerPause(PAUSE_SEQUENCE_MS, SEQUENCE);
  else              etatActuel = STOP;
}


class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s)    { deviceConnected = true;  Serial.println("Connecte!"); }
  void onDisconnect(BLEServer* s) { deviceConnected = false; arreterRobot(); s->startAdvertising(); }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String msg = pChar->getValue().c_str();
    msg.trim();
    Serial.println("Commande: " + msg);

    if (msg.startsWith("A:")) {
      demarrerAvancer(msg.substring(2).toFloat());      

    } else if (msg.startsWith("R:")) {
      demarrerReculer(msg.substring(2).toFloat());    

    } else if (msg.startsWith("T:")) {
      demarrerTourner(msg.substring(2).toFloat());   

    } else if (msg.startsWith("C:")) {
      demarrerCercle(msg.substring(2).toFloat());

    } else if (msg == "A") {
      demarrerManuel(AVANCER);

    } else if (msg == "R") {
      demarrerManuel(RECULER);

    } else if (msg == "D") {
      demarrerManuel(TOURNER, 1);

    } else if (msg == "G") {
      demarrerManuel(TOURNER, -1);

    } else if (msg == "S1") {
      etapeSeq = 0;
      prochaineEtapeSequence();

    } else if (msg == "S") {
      arreterRobot();
    }
  }
};

void writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t read16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);

  uint8_t l = Wire.read();
  uint8_t h = Wire.read();

  return (int16_t)(h << 8 | l);
}

void initialiserCapteurs() {
  Wire.begin(SDA_PIN, SCL_PIN);

  writeReg(ADDR_IMU, 0x10, 0x40); 
  writeReg(ADDR_IMU, 0x11, 0x40); 
  writeReg(ADDR_IMU, 0x12, 0x44); 

  writeReg(ADDR_MAG, 0x20, 0x70);
  writeReg(ADDR_MAG, 0x21, 0x00);
  writeReg(ADDR_MAG, 0x22, 0x00); 
  writeReg(ADDR_MAG, 0x23, 0x0C);
}

void envoyerCapteurs() {
  static unsigned long dernierEnvoi = 0;

  if (!deviceConnected) return;
  if (millis() - dernierEnvoi < 200) return;

  dernierEnvoi = millis();

  int16_t ax = read16(ADDR_IMU, 0x28);
  int16_t ay = read16(ADDR_IMU, 0x2A);
  int16_t az = -read16(ADDR_IMU, 0x2C);

  int16_t gx = read16(ADDR_IMU, 0x22);
  int16_t gy = read16(ADDR_IMU, 0x24);
  int16_t gz = read16(ADDR_IMU, 0x26);

  int16_t mx = read16(ADDR_MAG, 0x28);
  int16_t my = read16(ADDR_MAG, 0x2A);
  int16_t mz = read16(ADDR_MAG, 0x2C);

  calculerVitesse();

  String data = "DATA:"
              + String(ax) + "," + String(ay) + "," + String(az) + ","
              + String(gx) + "," + String(gy) + "," + String(gz) + ","
              + String(mx) + "," + String(my) + "," + String(mz) + ","
              + String(vitesseG) + "," + String(vitesseD) + ","
              + String(lireTicksGauche()) + "," + String(lireTicksDroite()) + ","
              + String(lireDistance()) + "," + String(nomEtat()) + ","
              + String(etapeSeq);

  pTxCharacteristic->setValue(data.c_str());
  pTxCharacteristic->notify();

}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  initialiserMoteur();
  initialiserRoue();
  initialiserCapteurs();
  lastTime = millis();

  BLEDevice::init("DRAWBOT");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHAR_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pRx = pService->createCharacteristic(CHAR_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRx->setCallbacks(new RxCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("DRAWBOT pret !");
}

void loop() {
  envoyerCapteurs();

  if (etatActuel == AVANCER) {
    appliquerAvancer();
    terminerMouvementSiAtteint();

  } else if (etatActuel == RECULER) {
    appliquerReculer();
    terminerMouvementSiAtteint();

  } else if (etatActuel == TOURNER) {
    appliquerTourner();
    terminerMouvementSiAtteint();

  } else if (etatActuel == CERCLE) {
    appliquerCercle();

  } else if (etatActuel == PAUSE) {
    if (millis() - pauseDebut >= pauseDuree) {
      etatActuel = etatSuivant;
      if (etatActuel == SEQUENCE) prochaineEtapeSequence();
    }
  }
  delay(20);
}
