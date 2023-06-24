#include "encoder.h"

float Encoder::angular_speed_wl = 0.0f;
float Encoder::angular_speed_wr = 0.0f;
volatile int Encoder::encoder_count_wl = 0;
volatile int Encoder::encoder_count_wr = 0;
std::function<void()> Encoder::callback = nullptr;

float Encoder::getAngularSpeedWL() { return angular_speed_wl; }
float Encoder::getAngularSpeedWR() { return angular_speed_wr; }

void Encoder::computeEncoderCallback(void *arg) {
  angular_speed_wl =
      (2 * M_PI * encoder_count_wl) /
      (PULSES_PER_REVOLUTION * GEAR_RATIO * (ENCODER_RESOLUTION / 1E6f));
  angular_speed_wr =
      (2 * M_PI * encoder_count_wr) /
      (PULSES_PER_REVOLUTION * GEAR_RATIO * (ENCODER_RESOLUTION / 1E6f));
  encoder_count_wl = 0;
  encoder_count_wr = 0;

  if (callback != nullptr)
    callback();
}

void Encoder::handleEncoderWL() {
  int pin_state_1 = digitalRead(PIN_ENCODER_1_A);
  int pin_state_2 = digitalRead(PIN_ENCODER_1_B);

  if (pin_state_1 == pin_state_2)
    encoder_count_wl++;
  else
    encoder_count_wl--;
}

void Encoder::handleEncoderWR() {
  int pin_state_1 = digitalRead(PIN_ENCODER_2_A);
  int pin_state_2 = digitalRead(PIN_ENCODER_2_B);

  if (pin_state_1 == pin_state_2)
    encoder_count_wr++;
  else
    encoder_count_wr--;
}

void Encoder::setup() {
  pinMode(PIN_ENCODER_1_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_1_B, INPUT_PULLUP);
  pinMode(PIN_ENCODER_2_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_1_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_1_A),
                  &Encoder::handleEncoderWL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_2_A),
                  &Encoder::handleEncoderWR, CHANGE);

  esp_timer_handle_t encoder_timer;
  esp_timer_create_args_t encoder_timer_args = {
      .callback = &Encoder::computeEncoderCallback, .name = "encoder"};
  ESP_ERROR_CHECK(esp_timer_create(&encoder_timer_args, &encoder_timer));
}

void Encoder::registerCallback(std::function<void()> callbackFunction) {
  callback = callbackFunction;
}