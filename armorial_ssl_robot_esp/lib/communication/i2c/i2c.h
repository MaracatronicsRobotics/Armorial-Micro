#ifndef ARMORIAL_SUASSUNA_I2C_H
#define ARMORIAL_SUASSUNA_I2C_H

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQUENCY_HZ 400000
#define STM_I2C_ADDRESS 0x1F

#include <packets/packets.h>

class I2C {
public:
  static void initialize();
  static void sendControlPacket(const ControlPacket &controlPacket);
  static void getFeedbackPacket();
};

#endif // ARMORIAL_SUASSUNA_I2C_H
