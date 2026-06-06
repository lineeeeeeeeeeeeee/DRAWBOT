#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "moteurs.h"
#include "capteurs.h"

bool magOk = false;

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

  if (Wire.available() < 2) return 0;
  uint8_t l = Wire.read();
  uint8_t h = Wire.read();
  return (int16_t)(h << 8 | l);
}

void setupCapteurs() {
  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.beginTransmission(ADDR_MAG);
  magOk = Wire.endTransmission() == 0;
  if (!magOk) {
    etat = "MAG_ABSENT";
    return;
  }

  writeReg(ADDR_MAG, 0x20, 0x70);
  writeReg(ADDR_MAG, 0x21, 0x00);
  writeReg(ADDR_MAG, 0x22, 0x00);
  writeReg(ADDR_MAG, 0x23, 0x0C);
  etat = "MAG_OK";
}

bool magnetometreDisponible() {
  return magOk;
}

float normaliserDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float lireCapNordDeg() {
  if (!magOk) return NAN;

  float sommeX = 0.0f;
  float sommeY = 0.0f;
  const int mesures = 8;

  for (int i = 0; i < mesures; i++) {
    sommeX += read16(ADDR_MAG, 0x28);
    sommeY += read16(ADDR_MAG, 0x2A);
    delay(4);
  }

  float mx = sommeX / mesures;
  float my = sommeY / mesures;
  if (abs(mx) < 1.0f && abs(my) < 1.0f) return NAN;

  float cap = atan2(my, mx) * 180.0f / PI;
  if (cap < 0.0f) cap += 360.0f;
  return cap;
}

float erreurNordDeg() {
  float cap = lireCapNordDeg();
  if (isnan(cap)) return NAN;
  return normaliserDeg(-cap);
}

bool orienterVersNord(float toleranceDeg, unsigned long timeoutMs) {
  if (!magOk) {
    etat = "NORD_SANS_MAG";
    return false;
  }

  unsigned long debut = millis();
  while (sequenceActive && millis() - debut < timeoutMs) {
    float erreur = erreurNordDeg();
    if (isnan(erreur)) {
      etat = "NORD_INVALIDE";
      stopMoteurs();
      delay(100);
      continue;
    }

    if (abs(erreur) <= toleranceDeg) {
      stopMoteurs();
      etat = "NORD_OK";
      delay(300);
      return true;
    }

    int pwm = abs(erreur) < 25.0f ? 70 : 95;
    if (erreur > 0.0f) {
      etat = "NORD_DROITE";
      moteurs(pwm, -pwm);
    } else {
      etat = "NORD_GAUCHE";
      moteurs(-pwm, pwm);
    }

    delay(45);
    stopMoteurs();
    delay(35);
  }

  stopMoteurs();
  etat = "NORD_TIMEOUT";
  return false;
}
