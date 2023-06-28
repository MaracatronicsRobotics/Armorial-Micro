#ifndef ARMORIAL_SUASSUNA_PEER_H
#define ARMORIAL_SUASSUNA_PEER_H

#include <Arduino.h>
#include <esp_now.h>
#include <map>
#include <vector>

#define ROBOTS_COUNT 6

#define MAC_ADDR_SIZE 6

#define CONFIG_WIFI_CHANNEL 0
#define CONFIG_ESPNOW_CHANNEL 0
#define CONFIG_ESPNOW_ENCRYPT false

esp_now_peer_info_t peerInfo;
std::map<int, std::array<uint8_t, MAC_ADDR_SIZE>> peers;

bool PeerExists(const int &robotId) {
  return (peers.find(robotId) != peers.end());
}

bool GetPeerAddress(const int robotId, uint8_t *address) {
  if (peers.find(robotId) != peers.end()) {
    memcpy(address, peers.at(robotId).data(), MAC_ADDR_SIZE);
    return true;
  }
  return false;
}

inline void InsertPeer(const int &robotId,
                       const std::array<uint8_t, MAC_ADDR_SIZE> &address) {
  /// TODO: try to get channel from a config packet
  peers.insert({robotId, address});
  peerInfo.channel = CONFIG_ESPNOW_CHANNEL;
  peerInfo.encrypt = CONFIG_ESPNOW_ENCRYPT;
  memcpy(peerInfo.peer_addr, address.data(), MAC_ADDR_SIZE);
  ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

#endif // ARMORIAL_SUASSUNA_PEER_H
