#ifndef ARMORIAL_SUASSUNA_COMMUNICATION
#define ARMORIAL_SUASSUNA_COMMUNICATION

#include <crc/crc.h>
#include <packets/packets.h>
#include <peer/peers.h>
#include <regex>

#include <esp_now.h>
#include <esp_wifi.h>

#define BROADCAST_ADDRESS                                                      \
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
#define SEQUENTIAL_ERRORS_TO_REMOVE_PEER 5

class Communication {
public:
  Communication();
  static void setupEspNow();
  static void resetUnconnectedPeersFeedbackTimestamp(const uint64_t &timestamp);
  static bool processSerial(std::string &data);
  static bool getFeedbackFromRobot(const int &robotId, std::string &buffer);
  static uint64_t getLastFeedbackTimestamp(const int &robotId);

protected:
  static void EspNowDataReceived(const uint8_t *mac,
                                 const uint8_t *incomingData, int len);
  static void EspNowDataSent(const uint8_t *mac, esp_now_send_status_t status);

private:
  static bool checkIfHasDelimiterMatch(std::string &data);
  static int getPlayerIdFromPacket(const ControlPacket &controlPacket);
  static bool isDelimiter(const char &c);
  static bool isAddressValidForPlayer(const uint8_t &playerId,
                                      const uint8_t *upcomingMacAddress);
  static std::string addDelimiters(const std::string &message);
  static bool isValidPlayerId(const int &playerId);

  static int commandFailCounter[MAX_NUM_ROBOTS];
  static uint64_t lastFeedbackTimestamp[MAX_NUM_ROBOTS];
  static bool canSendFeedbacks;
  static std::string feedbacksBuffer[MAX_NUM_ROBOTS];
};

#endif /* ARMORIAL_SUASSUNA_COMMUNICATION */
