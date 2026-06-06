#pragma once

void setupCapteurs();
bool magnetometreDisponible();
float lireCapNordDeg();
float normaliserDeg(float angle);
float erreurNordDeg();
bool orienterVersNord(float toleranceDeg = 8.0f, unsigned long timeoutMs = 10000);
