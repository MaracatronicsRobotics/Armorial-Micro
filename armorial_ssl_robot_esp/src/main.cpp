#include <communication.h>
#include <i2c/i2c.h>
#include <packets/packets.h>

#include <crc/crc.h>

void setup() {
  Serial.begin(115200);

  I2C::initialize();
  Communication::setupEspNow();
  Communication::setupFeedbacks();
}

void loop() {}
