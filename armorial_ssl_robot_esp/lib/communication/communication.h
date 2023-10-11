#ifndef ARMORIAL_SUASSUNA_COMMUNICATION_H
#define ARMORIAL_SUASSUNA_COMMUNICATION_H

#include <Ticker.h>
#include <espnow.h>
#include <packets/packets.h>

#define CONFIG_WIFI_CHANNEL 11
#define CONFIG_ESPNOW_CHANNEL 0
#define CONFIG_ESPNOW_ENCRYPT false

#define FEEDBACK_TIME 0.5 // seconds

class Communication {
public:
  Communication();
  static void setupEspNow();
  static void setupFeedbacks();

  static bool sendFeedbackPacket(const FeedbackPacket &feedbackPacket);

protected:
  static void EspNowDataReceived(u8 *mac, u8 *incomingData, unsigned char len);

  static bool checkIfMacIsBaseStation(const uint8_t *mac);

  static void computeFeedbackCallback();

private:
  static u8 baseStationMacAddr[6];
  static bool receivedFromBaseStation;
  static bool receivedRobotPacket;
  static Ticker feedbackTimer;
};

#endif // ARMORIAL_SUASSUNA_COMMUNICATION_H