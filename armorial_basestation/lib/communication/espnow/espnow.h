#ifndef ARMORIAL_SUASSUNA_ESPNOW
#define ARMORIAL_SUASSUNA_ESPNOW

#include <communication.h>
#include <esp_wifi.h>
#include <peer/peer.h>
#define SEQUENTIAL_ERRORS_TO_REMOVE_PEER 5

int commandFailCounter[MAX_NUM_ROBOTS] = {0};
bool canSendFeedbacks = false;
inline bool CanSendFeedbacks() { return canSendFeedbacks; }
inline void SetCanSendFeedbacks() { canSendFeedbacks = true; }

std::string feedbackBuffer[MAX_NUM_ROBOTS];
// TODO: set size of array equals MAX_NUM_ROBOTS, and find the other 6 missing pins
uint8_t ledPeerPins[6] = { 19, 18, 27, 14, 26, 25};
uint8_t buzzerPin = 4;

bool isUpcomingMacAddressValidForPlayer(const uint8_t &playerId,
                                        const uint8_t *upcomingMacAddress) {
  uint8_t registeredPeerAddress[MAC_ADDR_SIZE];
  GetPeerAddress(playerId, registeredPeerAddress);
  return (memcmp(upcomingMacAddress, registeredPeerAddress, MAC_ADDR_SIZE) ==
          0);
}

std::string addDelimiters(const std::string &message) {
  std::string delimitedMessage = LEFT_DELIMITER;
  delimitedMessage += message;
  delimitedMessage += RIGHT_DELIMITER;

  return delimitedMessage;
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  int playerIdentifier;
  if(GetPeerIdByAddress(mac, &playerIdentifier)) {
    if(status != ESP_NOW_SEND_FAIL) {
      commandFailCounter[playerIdentifier] = std::max(0, commandFailCounter[playerIdentifier] - 1);
    }
    else {
      commandFailCounter[playerIdentifier]++;
      if(commandFailCounter[playerIdentifier] >= SEQUENTIAL_ERRORS_TO_REMOVE_PEER) {
        if(DelPeer(playerIdentifier)) {
          esp_now_del_peer(mac);
          tone(buzzerPin, 5000, 100);
          delay(100);
          noTone(buzzerPin);
        }
        commandFailCounter[playerIdentifier] = 0;
      }
    }
  }
}

// ESPNow callbacks
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (CanSendFeedbacks() && len == sizeof(FeedbackPacket)) {
    FeedbackPacket packet;
    memcpy(&packet, incomingData, sizeof(FeedbackPacket));
    uint8_t playerId = packet.control & 0x0F;

    if (validate_feedbackpacket_crc(packet)) {
      if (!PeerExists(playerId)) {
        std::array<uint8_t, MAC_ADDR_SIZE> peerAddress;
        memcpy(peerAddress.data(), mac, MAC_ADDR_SIZE);
        InsertPeer(playerId, peerAddress);
        tone(buzzerPin, 5000, 100);
        delay(100);
        noTone(buzzerPin);
      } else {
        if (!isUpcomingMacAddressValidForPlayer(playerId, mac)) {
          return;
        }
      }
    }

    std::string buf;
    for (int i = 0; i < sizeof(FeedbackPacket); i++) {
      buf += (char)incomingData[i];
    }

    std::string feedback = addDelimiters(buf);
    feedbackBuffer[playerId] = feedback;
  }
}

inline void InitEspNow() {
  // Setting pins mode
  for (int robotId = 0; robotId < 6; robotId++) {
    pinMode(ledPeerPins[robotId], OUTPUT);
    digitalWrite(ledPeerPins[robotId], LOW);
  }

  pinMode(buzzerPin, OUTPUT);

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
  ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));
  ESP_ERROR_CHECK(esp_now_register_send_cb(OnDataSent));
  InsertPeer(100, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
}

#endif/* ARMORIAL_SUASSUNA_ESPNOW */
