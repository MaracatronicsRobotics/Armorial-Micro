#ifndef ARMORIAL_SUASSUNA_BUZZER
#define ARMORIAL_SUASSUNA_BUZZER

#define BUZZER_PIN 4

class Buzzer {
public:
  Buzzer();
  static void setupBuzzer();
  static void connectSound();
};

#endif /* ARMORIAL_SUASSUNA_BUZZER */
