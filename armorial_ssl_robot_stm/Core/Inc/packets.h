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
  float vw1, vw2, vw3, vw4;
  float vw1_encoder, vw2_encoder, vw3_encoder, vw4_encoder;
  uint64_t timestamp;
  uint16_t crc;
} FeedbackPacket;

typedef struct {
  char control;
  float vw1, vw2, vw3, vw4;
  char solenoidPower;
  uint16_t crc;
} ControlPacket;

#endif // ARMORIAL_SUASSUNA_PACKETS_H
