#ifndef ARMORIAL_SUASSUNA_PACKETS_H
#define ARMORIAL_SUASSUNA_PACKETS_H

#include <stdint.h>

#define ROBOT_ID 2

typedef struct {
  char control = ROBOT_ID;
  uint8_t batteryPercentage = 0;
  uint8_t infraRedStatus = false;
  float vx = 0, vy = 0, vw = 0;
  float vw1_encoder = 0, vw2_encoder = 0, vw3_encoder = 0, vw4_encoder = 0;
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  uint64_t timestamp = 0;
  uint16_t crc = 0;
} FeedbackPacket;

// Control packet
typedef struct {
  char control = ROBOT_ID;
  float vx = 0, vy = 0, vw = 0;
  char solenoidPower = 0;
  uint16_t crc = 0;
} ControlPacket;

#endif // ARMORIAL_SUASSUNA_PACKETS_H
