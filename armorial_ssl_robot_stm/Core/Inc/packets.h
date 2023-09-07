/*
 * packets.h
 *
 *  Created on: Aug 5, 2023
 *      Author: zsmn
 */

#ifndef ARMORIAL_SUASSUNA_PACKETS_H
#define ARMORIAL_SUASSUNA_PACKETS_H

#include <stdint.h>

typedef struct {
  char control;
  uint8_t batteryPercentage;
  uint8_t infraRedStatus;
  float vx, vy, vw;
  float vw1_encoder, vw2_encoder, vw3_encoder, vw4_encoder;
  float gyro_x, gyro_y, gyro_z;
  uint64_t timestamp;
  uint16_t crc;
} FeedbackPacket;

// Control packet
typedef struct {
  char control;
  float vx, vy, vw;
  char solenoidPower;
  uint16_t crc;
} ControlPacket;

#endif // ARMORIAL_SUASSUNA_PACKETS_H
