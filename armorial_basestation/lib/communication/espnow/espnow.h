#ifndef ARMORIAL_SUASSUNA_ESPNOW
#define ARMORIAL_SUASSUNA_ESPNOW

#include <esp_wifi.h>
#include <communication.h>
#include <peer/peer.h>

bool canSendFeedbacks = false;
inline bool CanSendFeedbacks() { return canSendFeedbacks; }
inline void SetCanSendFeedbacks() { canSendFeedbacks = true; }

std::string feedbackBuffer[MAX_NUM_ROBOTS];

bool isUpcomingMacAddressValidForPlayer(const uint8_t& playerId, const uint8_t* upcomingMacAddress) {
  uint8_t registeredPeerAddress[MAC_ADDR_SIZE];
  GetPeerAddress(playerId, registeredPeerAddress);
  return (memcmp(upcomingMacAddress, registeredPeerAddress, MAC_ADDR_SIZE) == 0);
}

std::string addDelimiters(const std::string& message) {
  std::string delimitedMessage = LEFT_DELIMITER;
  delimitedMessage += message;
  delimitedMessage += RIGHT_DELIMITER;

  return delimitedMessage;
}

// ESPNow callbacks
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (CanSendFeedbacks() && len == sizeof(FeedbackPacket)) {
    FeedbackPacket packet;
    memcpy(&packet, incomingData, sizeof(FeedbackPacket));
    uint8_t playerId = packet.control & 0x0F;

    if(validate_feedbackpacket_crc(packet)) {
      if (!PeerExists(playerId)) {
        std::array<uint8_t, MAC_ADDR_SIZE> peerAddress;
        memcpy(peerAddress.data(), mac, MAC_ADDR_SIZE);
        InsertPeer(playerId, peerAddress);
      }
      else {
        if(!isUpcomingMacAddressValidForPlayer(playerId, mac)) {
          return ;
        }
      }

      std::string buf;
      for(int i = 0; i < sizeof(FeedbackPacket); i++) {
        buf += (char) incomingData[i];
      }

      std::string feedback = addDelimiters(buf);
      feedbackBuffer[playerId] = feedback;
    }
  }
}

inline void InitEspNow() {
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
  InsertPeer(100, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
}

#endif /* ARMORIAL_SUASSUNA_ESPNOW */
