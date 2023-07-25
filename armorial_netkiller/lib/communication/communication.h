#ifndef ARMORIAL_SUASSUNA_COMMUNICATION
#define ARMORIAL_SUASSUNA_COMMUNICATION

#include <crc/crc.h>
#include <packets/packets.h>
#include <peer/peer.h>

#define BROADCAST_ADDRESS                                                      \
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

static void timer_callback(void *arg);

inline ControlPacket generateControlGarbage() {
  ControlPacket garbagePacket;
  garbagePacket.control = '0';
  float garbageFloat[4] = {0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < 4; i++) {
    garbageFloat[i] = std::rand() % 100 + 30;
  }
  garbagePacket.vw1 = garbageFloat[0];
  garbagePacket.vw2 = garbageFloat[1];
  garbagePacket.vw3 = garbageFloat[2];
  garbagePacket.vw4 = garbageFloat[3];
  garbagePacket.solenoidPower = std::rand() % 100;
  garbagePacket.crc = 54321;

  return garbagePacket;
}

inline FeedbackPacket generateFeedbackGarbage() {
  FeedbackPacket garbagePacket;
  garbagePacket.control = '5';
  garbagePacket.batteryPercentage = std::rand() % 100;
  garbagePacket.infraRedStatus = std::rand() % 2;
  float garbageFloat[4] = {0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < 4; i++) {
    garbageFloat[i] = std::rand() % 100 + 30;
  }
  garbagePacket.vw1 = garbageFloat[0];
  garbagePacket.vw2 = garbageFloat[1];
  garbagePacket.vw3 = garbageFloat[2];
  garbagePacket.vw4 = garbageFloat[3];
  garbagePacket.vw1_encoder = garbageFloat[0];
  garbagePacket.vw2_encoder = garbageFloat[1];
  garbagePacket.vw3_encoder = garbageFloat[2];
  garbagePacket.vw4_encoder = garbageFloat[3];
  garbagePacket.timestamp = std::rand() % 1000000;
  garbagePacket.crc = std::rand() % 1000000;

  return garbagePacket;
}

inline uint8_t generateGarbage() { return std::rand() % 255; }

inline void SendGarbage(int timeOut = 1) {
  Serial.println("Sending garbage");
  esp_timer_handle_t espTimerHandle;
  const esp_timer_create_args_t espTimerCreateArgs = {
      .callback = &timer_callback,
      .arg = (void *)espTimerHandle,
      .name = "timer"};
  ESP_ERROR_CHECK(esp_timer_create(&espTimerCreateArgs, &espTimerHandle));
  ESP_ERROR_CHECK(esp_timer_start_once(espTimerHandle, timeOut * 1000000));

  uint8_t broadcastMacAddress[MAC_ADDR_SIZE] = BROADCAST_ADDRESS;
  // FeedbackPacket garbagePacket = generateFeedbackGarbage();
  // ControlPacket garbagePacket = generateControlGarbage();
  uint8_t garbagePacket = generateGarbage();
  size_t garbageSize = std::rand() % 100 + 1;

  while (esp_timer_is_active(espTimerHandle)) {
    esp_err_t ret =
        esp_now_send(broadcastMacAddress, &garbagePacket, garbageSize);
  }
}

static void timer_callback(void *arg) { Serial.println("Timer expired"); }

#endif /* ARMORIAL_SUASSUNA_COMMUNICATION */
