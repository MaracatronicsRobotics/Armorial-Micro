#ifndef ARMORIAL_SUASSUNA_ESPNOW_H
#define ARMORIAL_SUASSUNA_ESPNOW_H

#include <WiFi.h>
#include <communication.h>
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
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.channel(CONFIG_ESPNOW_CHANNEL);

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));
  AddPeersToEspNow();
}

#endif // ARMORIAL_SUASSUNA_ESPNOW_H
