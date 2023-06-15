#ifndef ARMORIAL_SUASSUNA_ESPNOW_H
#define ARMORIAL_SUASSUNA_ESPNOW_H

#include "packet.h"
#include "peer.h"
#include <WiFi.h>

bool canSendFeedbacks = false;
inline bool CanSendFeedbacks() { return canSendFeedbacks; }
inline void SetCanSendFeedbacks() { canSendFeedbacks = true; }

// ESPNow callbacks
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (CanSendFeedbacks() && len == sizeof(struct_feedback)) {
    Serial.write(&startDelimiter, sizeof(char));
    Serial.write(incomingData, len);
    Serial.write(&endDelimiter, sizeof(char));
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
