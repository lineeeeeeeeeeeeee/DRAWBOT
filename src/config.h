#pragma once

#include <Arduino.h>

#define EN_D   23
#define EN_G    4
#define IN_1_D 19
#define IN_2_D 18
#define IN_1_G 17
#define IN_2_G 16
#define ENC_G_A 32
#define ENC_D_A 27
#define SDA_PIN 21
#define SCL_PIN 22
#define ADDR_MAG 0x1E

const float TICKS_PAR_CM = 34.5f;
const float ENTRAXE_CM = 14.0f;
const float DISTANCE_STYLO_CM = 13.0f;
const int PWM_MIN = 70;
const int PWM_MAX = 180;
const unsigned long PAUSE_MS = 800;

extern float correctionRot;
extern int pwmBase;
extern int pwmRot;
extern float kpLigne;
extern float kdLigne;
extern float kpStylo;
extern int pwmStylo;
extern bool sequenceActive;
extern String etat;
