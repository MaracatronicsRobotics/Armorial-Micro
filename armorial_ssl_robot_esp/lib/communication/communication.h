#ifndef ARMORIAL_SUASSUNA_COMMUNICATION_H
#define ARMORIAL_SUASSUNA_COMMUNICATION_H

#include <WiFi.h>
#include <esp_now.h>
#include <packets/packets.h>

#define CONFIG_WIFI_CHANNEL 11
#define CONFIG_ESPNOW_CHANNEL 0
#define CONFIG_ESPNOW_ENCRYPT false

#define FEEDBACK_TIME 1000000 // microseconds

class Communication {
public:
  Communication();
  static void setupEspNow();
  static void setupFeedbacks();

  static esp_err_t sendFeedbackPacket(const FeedbackPacket &feedbackPacket);

protected:
  static void EspNowDataReceived(const uint8_t *mac,
                                 const uint8_t *incomingData, int len);

  static bool checkIfMacIsBaseStation(const uint8_t *mac);

  static void computeFeedbackCallback(void *arg);

private:
  static esp_now_peer_info_t baseStationPeerInfo;
  static bool receivedFromBaseStation;
};

#endif // ARMORIAL_SUASSUNA_COMMUNICATION_H