#include "i2c.h"

#include <Arduino.h>
#include <Wire.h>

#include <communication.h>
#include <crc/crc.h>

void I2C::initialize() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);

  Wire.setClock(I2C_FREQUENCY_HZ);
  Wire.begin(SDA_PIN, SCL_PIN);
}

void I2C::sendControlPacket(const ControlPacket &controlPacket) {
  size_t pktSize = sizeof(ControlPacket);
  Wire.beginTransmission(STM_I2C_ADDRESS);
  uint8_t buffer[1024];
  memcpy(buffer, &controlPacket, pktSize);
  Wire.write(buffer, pktSize);
  Wire.endTransmission();
}

void I2C::getFeedbackPacket() {
  size_t pktSize = sizeof(FeedbackPacket);
  Wire.requestFrom(STM_I2C_ADDRESS, pktSize);
  uint8_t buffer[1024];
  FeedbackPacket packet;
  for (size_t i = 0; i < pktSize; i++) {
    uint8_t b = Wire.read();
    buffer[i] = b;
  }
  memcpy(&packet, buffer, pktSize);

  if (validatePacketCRC(packet)) {
    Communication::sendFeedbackPacket(packet);
  }
}