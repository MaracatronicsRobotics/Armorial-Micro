#ifndef ARMORIAL_SUASSUNA_ESPNOW_H
#define ARMORIAL_SUASSUNA_ESPNOW_H

#include <WiFi.h>
#include <communication.h>
#include <esp_wifi.h>
#include <peer/peer.h>

bool canSendFeedbacks = false;
inline bool CanSendFeedbacks() { return canSendFeedbacks; }
inline void SetCanSendFeedbacks() { canSendFeedbacks = true; }

// ESPNow callbacks
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (CanSendFeedbacks() && len == sizeof(FeedbackPacket)) {
    Serial.write((byte *)&startDelimiter, startDelimiter.length());
    Serial.write(incomingData, len);
    Serial.write((byte *)&endDelimiter, endDelimiter.length());
    Serial.flush();
  }
}

inline void InitEspNow() {
  // Disable power save mode
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  // Set device as a Wi-Fi Station
  tcpip_adapter_init();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  /* In order to simplify example, channel is set after WiFi started.
   * This is not necessary in real application if the two devices have
   * been already on the same channel.
   */
  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));
  AddPeersToEspNow();
}

#endif // ARMORIAL_SUASSUNA_ESPNOW_H
