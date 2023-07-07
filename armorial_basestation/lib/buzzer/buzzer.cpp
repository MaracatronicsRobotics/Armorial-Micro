#include "buzzer.h"

#include <Arduino.h>

Buzzer::Buzzer() {}

void Buzzer::setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
}

void Buzzer::connectSound() {
  const int soundFrequency = 5000; // hz
  const int soundTime = 100;       // ms

  tone(BUZZER_PIN, soundFrequency, soundTime);
  delay(soundTime);
  noTone(BUZZER_PIN);
}