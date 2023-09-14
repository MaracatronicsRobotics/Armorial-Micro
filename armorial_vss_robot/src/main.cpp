#include <communication.h>
#include <controller.h>
#include <crc/crc.h>
#include <encoder.h>
#include <mpu.h>

#define COMMUNICATION_PIN 4
#define COMMUNICATION_PIN_ID 5

Encoder *encoder = new Encoder();
Controller *controller = new Controller(encoder);
Communication *communication = new Communication(controller);

void sendEncoderFeedback() {
  ControlPacket controlPacket = controller->getControlPacket();
  FeedbackPacket feedbackPacket;
  feedbackPacket.control = controlPacket.control;
  feedbackPacket.batteryPercentage = 0.0f;
  feedbackPacket.infraRedStatus = false;
  feedbackPacket.vx = controlPacket.vx;
  feedbackPacket.vy = controlPacket.vy;
  feedbackPacket.vw = controlPacket.vw;
  feedbackPacket.vw1_encoder = encoder->getAngularSpeedWL();
  feedbackPacket.vw2_encoder = encoder->getAngularSpeedWR();
  feedbackPacket.vw3_encoder = 0.0f;
  feedbackPacket.vw4_encoder = 0.0f;
  feedbackPacket.gyro_x = MPU::getGyroX();
  feedbackPacket.gyro_y = MPU::getGyroY();
  feedbackPacket.gyro_z = MPU::getGyroZ();
  feedbackPacket.timestamp = esp_timer_get_time();
  feedbackPacket.crc = 0;
  feedbackPacket.crc = compute_crc16cdma2000_byte(
      CRC_INITIAL_VALUE, (char *)&feedbackPacket, sizeof(FeedbackPacket));

  communication->sendFeedbackPacket(feedbackPacket);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Setup LED pin
  ledcSetup(COMMUNICATION_PIN_ID, 60, 2);
  ledcAttachPin(COMMUNICATION_PIN, COMMUNICATION_PIN_ID);

  // Setup H-Bridge and PWM pins
  Controller::setupPWMPins();

  // Setup MPU
  MPU::setup();
  MPU::start();

  // Setup encoder
  Encoder::setup();
  Encoder::registerCallback(sendEncoderFeedback);

  // Setup communication
  Communication::setupEspNow();
}

void loop() {
  controller->drive();
  ledcWrite(COMMUNICATION_PIN_ID, 255);
  // ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, 255);
  // ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, 0);

  // ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, 255);
  // ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, 0);

  // ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, 0);
  // ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, 255);

  // ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, 0);
  // ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, 255);

  // delay(5000);
}