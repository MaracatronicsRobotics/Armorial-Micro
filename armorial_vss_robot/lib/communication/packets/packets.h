#ifndef ARMORIAL_SUASSUNA_PACKETS_H
#define ARMORIAL_SUASSUNA_PACKETS_H

#include <stdint.h>

#define ROBOT_ID 0
#define ZERO 0.001

typedef struct {
  char control = ROBOT_ID;
  uint8_t batteryPercentage = 0;
  bool infraRedStatus = false;
  float vw1 = 0, vw2 = 0, vw3 = 0, vw4 = 0;
  float vw1_encoder = 0, vw2_encoder = 0, vw3_encoder = 0, vw4_encoder = 0;
  uint64_t timestamp = 0;
  uint16_t crc = 0;
} FeedbackPacket;

// Control packet
typedef struct {
  char control = ROBOT_ID;
  float vw1 = 0, vw2 = 0, vw3 = 0, vw4 = 0;
  char solenoidPower = 0;
  uint16_t crc = 0;
} ControlPacket;

typedef struct {
  float vw1 = 0, vw2 = 0, vw3 = 0, vw4 = 0;
  char solenoidPower = 0;
} LastControlPacket;

#endif // ARMORIAL_SUASSUNA_PACKETS_H
