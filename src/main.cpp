#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "config.h"
#include "capteurs.h"
#include "deplacements.h"
#include "encodeurs.h"
#include "moteurs.h"
#include "sequence1.h"
#include "sequence2.h"
#include "sequence3.h"

#define SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;

void envoyerTelemetry() {
  static unsigned long dernier = 0;
  if (!deviceConnected || millis() - dernier < 200) return;
  dernier = millis();

  String data = "S1DATA:"
              + String(lireTicksGauche()) + ","
              + String(lireTicksDroite()) + ","
              + etat + ","
              + String(sequenceActive ? 1 : 0) + ","
              + String(correctionRot, 2) + ","
              + String(kpLigne, 2) + ","
              + String(kdLigne, 2) + ","
              + String(pwmBase) + ","
              + String(pwmRot) + ","
              + String(kpStylo, 1) + ","
              + String(pwmStylo);
  pTxCharacteristic->setValue(data.c_str());
  pTxCharacteristic->notify();
}

void gererCommande(String msg) {
  msg.trim();
  Serial.println("Commande: " + msg);

  if (msg == "S") {
    sequenceActive = false;
    etat = "STOP";
    stopMoteurs();
  } else if (msg == "A") {
    sequenceActive = false;
    etat = "MAN_AVANCER";
    moteurs(pwmBase, pwmBase);
  } else if (msg == "R") {
    sequenceActive = false;
    etat = "MAN_RECULER";
    moteurs(-pwmBase, -pwmBase);
  } else if (msg == "G") {
    sequenceActive = false;
    etat = "MAN_GAUCHE";
    moteurs(-pwmRot, pwmRot);
  } else if (msg == "D") {
    sequenceActive = false;
    etat = "MAN_DROITE";
    moteurs(pwmRot, -pwmRot);
  } else if (msg.startsWith("A:")) {
    sequenceActive = true;
    avancerCm(msg.substring(2).toFloat());
    sequenceActive = false;
    etat = "STOP";
    stopMoteurs();
  } else if (msg.startsWith("R:")) {
    sequenceActive = true;
    reculerCm(msg.substring(2).toFloat());
    sequenceActive = false;
    etat = "STOP";
    stopMoteurs();
  } else if (msg.startsWith("T:")) {
    sequenceActive = true;
    tournerDeg(msg.substring(2).toFloat());
    sequenceActive = false;
    etat = "STOP";
    stopMoteurs();
  } else if (msg == "S1") {
    sequenceEscalier();
  } else if (msg.startsWith("C:")) {
    sequenceCercle(msg.substring(2).toFloat());
  } else if (msg == "FINDN") {
    sequenceActive = true;
    orienterVersNord();
    sequenceActive = false;
    stopMoteurs();
  } else if (msg == "S3") {
    sequenceFleche(12.0f);
  } else if (msg.startsWith("DRAW3:")) {
    sequenceFleche(msg.substring(6).toFloat());
  } else if (msg.startsWith("ROSE3:")) {
    sequenceRoseDesVents(msg.substring(6).toFloat());
  } else if (msg.startsWith("E:")) {
    String args = msg.substring(2);
    int p1 = args.indexOf(',');
    int p2 = args.indexOf(',', p1 + 1);
    if (p1 > 0 && p2 > p1) {
      float largeur = args.substring(0, p1).toFloat();
      float hauteur = args.substring(p1 + 1, p2).toFloat();
      int marches = args.substring(p2 + 1).toInt();
      sequenceMarches(largeur, hauteur, marches);
    }
  } else if (msg.startsWith("KROT:")) {
    float k = msg.substring(5).toFloat();
    if (k >= 0.10f && k <= 1.20f) {
      correctionRot = k;
      etat = "KROT=" + String(correctionRot, 2);
    }
  } else if (msg.startsWith("KP:")) {
    float k = msg.substring(3).toFloat();
    if (k >= 0.0f && k <= 10.0f) {
      kpLigne = k;
      etat = "KP=" + String(kpLigne, 2);
    }
  } else if (msg.startsWith("KD:")) {
    float k = msg.substring(3).toFloat();
    if (k >= 0.0f && k <= 10.0f) {
      kdLigne = k;
      etat = "KD=" + String(kdLigne, 2);
    }
  } else if (msg.startsWith("PWM:")) {
    String args = msg.substring(4);
    int sep = args.indexOf(',');
    if (sep > 0) {
      int base = args.substring(0, sep).toInt();
      int rot = args.substring(sep + 1).toInt();
      if (base >= 50 && base <= 180) pwmBase = base;
      if (rot >= 50 && rot <= 180) pwmRot = rot;
      etat = "PWM=" + String(pwmBase) + "/" + String(pwmRot);
    }
  } else if (msg.startsWith("KSTYLO:")) {
    float k = msg.substring(7).toFloat();
    if (k >= 5.0f && k <= 80.0f) {
      kpStylo = k;
      etat = "KSTYLO=" + String(kpStylo, 1);
    }
  } else if (msg.startsWith("PSTYLO:")) {
    int p = msg.substring(7).toInt();
    if (p >= 50 && p <= 130) {
      pwmStylo = p;
      etat = "PSTYLO=" + String(pwmStylo);
    }
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) { deviceConnected = true; Serial.println("Connecte!"); }
  void onDisconnect(BLEServer* s) {
    deviceConnected = false;
    sequenceActive = false;
    stopMoteurs();
    s->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    gererCommande(pChar->getValue().c_str());
  }
};

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  setupMoteurs();
  setupEncodeurs();
  setupCapteurs();

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
  Serial.println("DRAWBOT pret. Commandes: S1, C:10, DRAW3:12, S3, E:10,10,3, S");
}

void loop() {
  envoyerTelemetry();
  delay(20);
}
