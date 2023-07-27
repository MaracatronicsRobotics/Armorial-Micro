#include "communication.h"

#include <buzzer.h>

int Communication::commandFailCounter[MAX_NUM_ROBOTS] = {0};
uint64_t Communication::lastFeedbackTimestamp[MAX_NUM_ROBOTS] = {0};
bool Communication::canSendFeedbacks = false;
std::string Communication::feedbacksBuffer[MAX_NUM_ROBOTS] = {};

//uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Communication::Communication() {}

void Communication::setupEspNow() {
  // Set device as a Wi-Fi Station
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(&Communication::EspNowDataReceived));
  ESP_ERROR_CHECK(esp_now_register_send_cb(&Communication::EspNowDataSent));
  Peers::insertPeer(100, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
}

void Communication::resetUnconnectedPeersFeedbackTimestamp(
    const uint64_t &timestamp) {
  for (int i = 0; i < MAX_NUM_ROBOTS; i++) {
    if (!Peers::peerExists(i)) {
      lastFeedbackTimestamp[i] = timestamp;
    }
  }
}

bool Communication::processSerial(std::string &data) {
  bool parsedPacket = false;
  while (checkIfHasDelimiterMatch(data)) {
    size_t startOfPattern = data.find(LEFT_DELIMITER);
    size_t endOfPattern = data.find(RIGHT_DELIMITER);

    if (startOfPattern != 0) {
      data.erase(data.begin(), data.begin() + startOfPattern);
      continue;
    }

    int matchSize = endOfPattern - startOfPattern + 3;

    std::string packetToValidate;
    for (int i = startOfPattern + 3; i < endOfPattern; i++) {
      packetToValidate += data.at(i);
    }
    data.erase(startOfPattern, matchSize);

    ControlPacket structuredPacket;
    uint8_t robotMacAddress[MAC_ADDR_SIZE] = BROADCAST_ADDRESS;
    if (packetToValidate.size() == sizeof(ControlPacket)) {
      memcpy(&structuredPacket, packetToValidate.c_str(),
             sizeof(ControlPacket));
      if (validatePacketCRC(structuredPacket)) {
        bool hasPeerAddress = Peers::getPeerAddressById(
            getPlayerIdFromPacket(structuredPacket), robotMacAddress);
        esp_err_t ret =
            esp_now_send(robotMacAddress, (uint8_t *)packetToValidate.c_str(),
                         sizeof(ControlPacket));
          //esp_now_send(broadcastAddr, (uint8_t *)packetToValidate.c_str(),
          //               sizeof(ControlPacket));
        parsedPacket = true;
      }
    }
  }

  if (parsedPacket) {
    canSendFeedbacks = true;
  }

  return parsedPacket;
}

bool Communication::getFeedbackFromRobot(const int &robotId,
                                         std::string &buffer) {
  if (feedbacksBuffer[robotId].size()) {
    std::string feedbackBuffer = feedbacksBuffer[robotId];
    feedbacksBuffer[robotId].clear();
    buffer = feedbackBuffer;
    return true;
  }

  return false;
}

uint64_t Communication::getLastFeedbackTimestamp(const int &robotId) {
  return lastFeedbackTimestamp[robotId];
}

void Communication::EspNowDataSent(const uint8_t *mac,
                                   esp_now_send_status_t status) {
  int playerIdentifier;
  if (Peers::getPeerIdByAddress(mac, &playerIdentifier)) {
    if (status != ESP_NOW_SEND_FAIL) {
      commandFailCounter[playerIdentifier] =
          std::max(0, commandFailCounter[playerIdentifier] - 1);
    } else {
      commandFailCounter[playerIdentifier]++;
      if (commandFailCounter[playerIdentifier] >=
          SEQUENTIAL_ERRORS_TO_REMOVE_PEER) {
        if (Peers::deletePeer(playerIdentifier)) {
          Buzzer::connectSound();
        }
        commandFailCounter[playerIdentifier] = 0;
      }
    }
  }
}

// ESPNow callbacks
void Communication::EspNowDataReceived(const uint8_t *mac,
                                       const uint8_t *incomingData, int len) {
  if (canSendFeedbacks && len == sizeof(FeedbackPacket)) {
    FeedbackPacket packet;
    memcpy(&packet, incomingData, sizeof(FeedbackPacket));
    uint8_t playerId = packet.control & 0x0F;

    if (validatePacketCRC(packet)) {
      if (isValidPlayerId(playerId)) {
        if (!Peers::peerExists(playerId)) {
          std::array<uint8_t, MAC_ADDR_SIZE> peerAddress;
          memcpy(peerAddress.data(), mac, MAC_ADDR_SIZE);
          Peers::insertPeer(playerId, peerAddress);
          Buzzer::connectSound();
        } else {
          if (!isAddressValidForPlayer(playerId, mac)) {
            return;
          }
        }
        std::string buf;
        for (int i = 0; i < sizeof(FeedbackPacket); i++) {
          buf += (char)incomingData[i];
        }

        std::string feedback = addDelimiters(buf);
        feedbacksBuffer[playerId] = feedback;
        lastFeedbackTimestamp[playerId] = esp_timer_get_time();
      }
    }
  }
}

bool Communication::checkIfHasDelimiterMatch(std::string &data) {
  std::smatch matches;
  std::regex regexExpression("<{3}(.*?)>{3}");

  return std::regex_search(data, matches, regexExpression);
}

int Communication::getPlayerIdFromPacket(const ControlPacket &controlPacket) {
  return (controlPacket.control & 0x0F);
}

bool Communication::isDelimiter(const char &c) {
  return (LEFT_DELIMITER.find(c) != std::string::npos ||
          RIGHT_DELIMITER.find(c) != std::string::npos);
}

bool Communication::isAddressValidForPlayer(const uint8_t &playerId,
                                            const uint8_t *upcomingMacAddress) {
  uint8_t registeredPeerAddress[MAC_ADDR_SIZE];
  Peers::getPeerAddressById(playerId, registeredPeerAddress);
  return (memcmp(upcomingMacAddress, registeredPeerAddress, MAC_ADDR_SIZE) ==
          0);
}

std::string Communication::addDelimiters(const std::string &message) {
  std::string delimitedMessage = LEFT_DELIMITER;
  delimitedMessage += message;
  delimitedMessage += RIGHT_DELIMITER;

  return delimitedMessage;
}

bool Communication::isValidPlayerId(const int &playerId) {
  return (playerId >= 0 && playerId < MAX_NUM_ROBOTS);
}