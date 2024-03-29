#ifndef ARMORIAL_SUASSUNA_ENCODER_H
#define ARMORIAL_SUASSUNA_ENCODER_H

#include <esp32-hal-gpio.h>
#include <esp_timer.h>
#include <functional>
#include <math.h>
#include <packets/packets.h>

#define PIN_ENCODER_RIGHT_A 18
#define PIN_ENCODER_RIGHT_B 13

#define PIN_ENCODER_LEFT_A 5
#define PIN_ENCODER_LEFT_B 19

#define PULSES_PER_REVOLUTION 7
#define GEAR_RATIO 100
#define ENCODER_RESOLUTION 10000 // microsseconds

#define ENCODER_CALLS_BEFORE_FEEDBACK 1

#include <SimpleKalmanFilter.h>

class Encoder {
public:
  Encoder() = default;

  float getAngularSpeedWL();
  float getAngularSpeedWR();
  static void setup();
  static void registerCallback(std::function<void()> callbackFunction);

protected:
  static void handleEncoderWL();
  static void handleEncoderWR();
  static void computeEncoderCallback(void *arg);

private:
  static float angular_speed_wl;
  static float angular_speed_wr;
  static volatile int encoder_count_wl;
  static volatile int encoder_count_wr;
  static std::function<void()> callback;
  static SimpleKalmanFilter _kalman_wl;
  static SimpleKalmanFilter _kalman_wr;
};

#endif // ARMORIAL_SUASSUNA_ENCODER_H
