#include <communication.h>
#include <controller.h>
#include <crc/crc.h>
#include <encoder.h>
#include <mpu.h>

Encoder *encoder = new Encoder();
Controller *controller = new Controller(encoder);
Communication *communication = new Communication(controller);

void sendEncoderFeedback() {
  ControlPacket controlPacket = controller->getControlPacket();
  FeedbackPacket feedbackPacket;
  feedbackPacket.control = controlPacket.control;
  feedbackPacket.batteryPercentage = 0.0f;
  feedbackPacket.infraRedStatus = false;
  feedbackPacket.vw1_encoder = encoder->getAngularSpeedWL();
  feedbackPacket.vw1 = controlPacket.vw1;
  feedbackPacket.vw2_encoder = encoder->getAngularSpeedWR();
  feedbackPacket.vw2 = controlPacket.vw2;
  feedbackPacket.vw3 = 0.0f;
  feedbackPacket.vw3_encoder = 0.0f;
  feedbackPacket.vw4 = 0.0f;
  feedbackPacket.vw4_encoder = 0.0f;
  feedbackPacket.gyro_x = MPU::getGyroXDeg();
  feedbackPacket.gyro_y = MPU::getGyroYDeg();
  feedbackPacket.gyro_z = MPU::getGyroZDeg();
  feedbackPacket.timestamp = esp_timer_get_time();
  feedbackPacket.crc = 0;
  feedbackPacket.crc = compute_crc16cdma2000_byte(
      CRC_INITIAL_VALUE, (char *)&feedbackPacket, sizeof(FeedbackPacket));

  communication->sendFeedbackPacket(feedbackPacket);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Setup H-Bridge and PWM pins
  Controller::setupHBridge();
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
  // ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, 255);
  // ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, 0);

  // ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, 255);
  // ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, 0);

  // delay(5000);

  // ledcWrite(WHEEL_LEFT_FORWARD_PIN_ID, 0);
  // ledcWrite(WHEEL_LEFT_BACKWARD_PIN_ID, 255);

  // ledcWrite(WHEEL_RIGHT_FORWARD_PIN_ID, 0);
  // ledcWrite(WHEEL_RIGHT_BACKWARD_PIN_ID, 255);

  // delay(5000);
}