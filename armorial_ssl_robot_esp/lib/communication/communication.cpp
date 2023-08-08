#include "communication.h"
#include <ESP8266WiFi.h>
#include <crc/crc.h>
#include <i2c/i2c.h>

u8 Communication::baseStationMacAddr[6] = {};
bool Communication::receivedFromBaseStation = false;
Ticker Communication::feedbackTimer = {};

Communication::Communication() {}

void Communication::setupEspNow() {
  // Configure wifi channel
  WiFi.mode(WIFI_STA);
  wifi_set_channel(CONFIG_WIFI_CHANNEL);

  // Init ESP-NOW
  if (esp_now_init() != ERR_OK) {
    Serial.println("ESP-NOW initialization failed");
    ESP.restart();
    return;
  }

  // Setup data receive callback
  esp_now_register_recv_cb(&Communication::EspNowDataReceived);
}

void Communication::setupFeedbacks() {
  feedbackTimer.attach(FEEDBACK_TIME, computeFeedbackCallback);
}

bool Communication::sendFeedbackPacket(const FeedbackPacket &feedbackPacket) {
  if (!receivedFromBaseStation) {
    return false;
  }

  return (esp_now_send(baseStationMacAddr, (uint8_t *)&feedbackPacket,
                       sizeof(FeedbackPacket)) == ERR_OK);
}

void Communication::EspNowDataReceived(u8 *mac, u8 *incomingData,
                                       unsigned char len) {
  if (len == sizeof(ControlPacket)) {
    ControlPacket controlPacket;
    memcpy(&controlPacket, incomingData, sizeof(ControlPacket));
    if (validatePacketCRC(controlPacket)) {
      if (!receivedFromBaseStation) {
        receivedFromBaseStation = true;

        // Register basestation as peer
        /// TODO: get channel from a configuration packet
        memcpy(baseStationMacAddr, mac, 6);

        // Add peer
        esp_now_add_peer(baseStationMacAddr, ESP_NOW_ROLE_SLAVE,
                         CONFIG_ESPNOW_CHANNEL, NULL, 0);
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
  return memcmp(mac, baseStationMacAddr, 6) == 0;
}

void Communication::computeFeedbackCallback() { I2C::getFeedbackPacket(); }