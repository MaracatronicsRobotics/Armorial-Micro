#include "communication.h"
#include <crc/crc.h>
#include <esp_wifi.h>

Controller *Communication::_controller = nullptr;
esp_now_peer_info_t Communication::peerInfo = {};

Communication::Communication(Controller *controller) {
  _controller = controller;
}

void Communication::setupEspNow() {
  // Disable power save mode
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  ESP_ERROR_CHECK(esp_now_init());

  // Setup data receive callback
  ESP_ERROR_CHECK(esp_now_register_recv_cb(&Communication::EspNowDataReceived));

  // Register basestation as peer
  uint8_t baseStationAddress[] = BASE_STATION_MAC_ADDRESS;
  memcpy(peerInfo.peer_addr, baseStationAddress, 6);
  peerInfo.channel = 11;
  peerInfo.encrypt = false;

  // Add peer
  ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

esp_err_t
Communication::sendFeedbackPacket(const FeedbackPacket &feedbackPacket) {
  uint8_t baseStationAddress[] = BASE_STATION_MAC_ADDRESS;
  return esp_now_send(baseStationAddress, (uint8_t *)&feedbackPacket,
                      sizeof(FeedbackPacket));
}

void Communication::EspNowDataReceived(const uint8_t *mac,
                                       const uint8_t *incomingData, int len) {
  ControlPacket controlPacket;
  memcpy(&controlPacket, incomingData, sizeof(ControlPacket));

  // if (validate_controlpacket_crc(controlPacket)) {
  _controller->setControlPacket(controlPacket);
  //}
}