#include "leds.h"

#include <Arduino.h>

std::map<int, int> Leds::robotIdsToPins = {};

Leds::Leds() {}

void Leds::setupLeds() {
  robotIdsToPins.insert({0, ROBOT_0_LED});
  robotIdsToPins.insert({1, ROBOT_1_LED});
  robotIdsToPins.insert({2, ROBOT_2_LED});
  robotIdsToPins.insert({3, ROBOT_3_LED});
  robotIdsToPins.insert({4, ROBOT_4_LED});
  robotIdsToPins.insert({5, ROBOT_5_LED});

  for (std::map<int, int>::iterator it = robotIdsToPins.begin();
       it != robotIdsToPins.end(); it++) {
    pinMode(it->second, OUTPUT);
    digitalWrite(it->second, LOW);
  }
}

void Leds::turnOnRobot(const int &robotId) {
  std::map<int, int>::iterator robotLedPin = robotIdsToPins.find(robotId);
  if (robotLedPin != robotIdsToPins.end()) {
    digitalWrite(robotLedPin->second, HIGH);
  }
}

void Leds::turnOffRobot(const int &robotId) {
  std::map<int, int>::iterator robotLedPin = robotIdsToPins.find(robotId);
  if (robotLedPin != robotIdsToPins.end()) {
    digitalWrite(robotLedPin->second, LOW);
  }
}