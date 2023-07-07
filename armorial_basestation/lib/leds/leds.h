#ifndef ARMORIAL_SUASSUNA_LEDS
#define ARMORIAL_SUASSUNA_LEDS

#define ROBOT_0_LED 19
#define ROBOT_1_LED 18
#define ROBOT_2_LED 27
#define ROBOT_3_LED 14
#define ROBOT_4_LED 26
#define ROBOT_5_LED 25

#include <map>
#include <vector>

class Leds {
public:
  Leds();
  static void setupLeds();
  static void turnOnRobot(const int &robotId);
  static void turnOffRobot(const int &robotId);

private:
  static std::map<int, int> robotIdsToPins;
};

#endif /* ARMORIAL_SUASSUNA_LEDS */
