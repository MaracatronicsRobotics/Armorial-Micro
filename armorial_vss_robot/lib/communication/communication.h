#ifndef ARMORIAL_SUASSUNA_COMMUNICATION_H
#define ARMORIAL_SUASSUNA_COMMUNICATION_H

#include <WiFi.h>
#include <controller.h>
#include <esp_now.h>

#define BASE_STATION_MAC_ADDRESS                                               \
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
#define CONFIG_ESPNOW_CHANNEL 0
#define CONFIG_ESPNOW_ENCRYPT false

class Communication {
public:
  Communication(Controller *controller);
  static void setupEspNow();

  esp_err_t sendFeedbackPacket(const FeedbackPacket &feedbackPacket);

protected:
  static void EspNowDataReceived(const uint8_t *mac,
                                 const uint8_t *incomingData, int len);

private:
  static Controller *_controller;
  static esp_now_peer_info_t peerInfo;
};

#endif // ARMORIAL_SUASSUNA_COMMUNICATION_H
