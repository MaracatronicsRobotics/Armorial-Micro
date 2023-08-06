#include "communication.h"
#include <crc/crc.h>
#include <esp_wifi.h>

#include <i2c/i2c.h>

esp_now_peer_info_t Communication::baseStationPeerInfo = {};
bool Communication::receivedFromBaseStation = false;

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

  // Init ESP-NOW
  ESP_ERROR_CHECK(esp_now_init());

  // Setup data receive callback
  ESP_ERROR_CHECK(esp_now_register_recv_cb(&Communication::EspNowDataReceived));
}

void Communication::setupFeedbacks() {
  esp_timer_handle_t feedbacks_timer;
  esp_timer_create_args_t feedbacks_timer_args = {
      .callback = &Communication::computeFeedbackCallback, .name = "feedback"};
  ESP_ERROR_CHECK(esp_timer_create(&feedbacks_timer_args, &feedbacks_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(feedbacks_timer, FEEDBACK_TIME));
}

esp_err_t
Communication::sendFeedbackPacket(const FeedbackPacket &feedbackPacket) {
  if (!receivedFromBaseStation) {
    return ESP_ERR_ESPNOW_NOT_INIT;
  }

  return esp_now_send(baseStationPeerInfo.peer_addr, (uint8_t *)&feedbackPacket,
                      sizeof(FeedbackPacket));
}

void Communication::EspNowDataReceived(const uint8_t *mac,
                                       const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlPacket)) {
    ControlPacket controlPacket;
    memcpy(&controlPacket, incomingData, sizeof(ControlPacket));
    if (validatePacketCRC(controlPacket)) {
      if (!receivedFromBaseStation) {
        receivedFromBaseStation = true;

        // Register basestation as peer
        /// TODO: get channel from a configuration packet
        memcpy(baseStationPeerInfo.peer_addr, mac, 6);
        baseStationPeerInfo.channel = CONFIG_ESPNOW_CHANNEL;
        baseStationPeerInfo.encrypt = CONFIG_ESPNOW_ENCRYPT;

        // Add peer
        ESP_ERROR_CHECK(esp_now_add_peer(&baseStationPeerInfo));
      }

      if (!checkIfMacIsBaseStation(mac)) {
        return;
      }

      uint8_t robotId = controlPacket.control & 0x0F;
      if (robotId == ROBOT_ID) {
        I2C::sendControlPacket(controlPacket);
      }
    }
  }
}

bool Communication::checkIfMacIsBaseStation(const uint8_t *mac) {
  return memcmp(mac, baseStationPeerInfo.peer_addr, 6) == 0;
}

void Communication::computeFeedbackCallback(void *arg) {
  I2C::getFeedbackPacket();
}