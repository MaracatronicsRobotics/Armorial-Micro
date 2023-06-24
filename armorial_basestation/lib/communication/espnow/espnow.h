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
  WiFi.mode(WIFI_STA);
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));
  AddPeersToEspNow();
}

#endif // ARMORIAL_SUASSUNA_ESPNOW_H
