#ifndef ARMORIAL_SUASSUNA_ESPNOW
#define ARMORIAL_SUASSUNA_ESPNOW

#include <communication.h>
#include <esp_wifi.h>
#include <peer/peer.h>

// ESPNow callbacks
void OnDataSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  uint8_t wifi_channel;
  wifi_second_chan_t second_channel;
  esp_wifi_get_channel(&wifi_channel, &second_channel);
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.print("Throwing garbage successfully on channel ");
    Serial.println(wifi_channel);
  }
}

inline void InitEspNow(int CONFIG_WIFI_CHANNEL = 1) {
  // Set device as a Wi-Fi Station
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  CONFIG_WIFI_CHANNEL++;
  ESP_ERROR_CHECK(
      esp_wifi_set_channel(CONFIG_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

  // Setting timer
  //ESP_ERROR_CHECK(esp_timer_init());

  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_send_cb(OnDataSend));
  InsertPeer(100, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});
}

inline void ReconfigureEspNow() {
  Serial.println("Reconfiguring ESPNow");
  uint8_t wifi_channel;
  wifi_second_chan_t second_channel;
  esp_wifi_get_channel(&wifi_channel, &second_channel);
  ESP_ERROR_CHECK(esp_wifi_disconnect());
  ESP_ERROR_CHECK(esp_wifi_stop());
  ESP_ERROR_CHECK(esp_wifi_deinit());
  ESP_ERROR_CHECK(esp_event_loop_delete_default());
  //ESP_ERROR_CHECK(esp_timer_deinit());

  uint32_t CONFIG_WIFI_CHANNEL = 0;
  if (wifi_channel == 1) {
    CONFIG_WIFI_CHANNEL = 1;
  } else {
    CONFIG_WIFI_CHANNEL = 0;
  }
  InitEspNow(CONFIG_WIFI_CHANNEL);
}

#endif /* ARMORIAL_SUASSUNA_ESPNOW */
