#include "peers.h"

std::map<int, std::array<uint8_t, MAC_ADDR_SIZE>> Peers::peers = {};
esp_now_peer_info_t Peers::peerInfo = {};

Peers::Peers() {}

void Peers::insertPeer(const int &robotId,
                       const std::array<uint8_t, MAC_ADDR_SIZE> &address) {
  peers.insert({robotId, address});
  peerInfo.channel = CONFIG_ESPNOW_CHANNEL;
  peerInfo.encrypt = CONFIG_ESPNOW_ENCRYPT;
  memcpy(peerInfo.peer_addr, address.data(), MAC_ADDR_SIZE);
  ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

bool Peers::getPeerIdByAddress(const uint8_t *address, int *id) {
  for (std::map<int, std::array<uint8_t, MAC_ADDR_SIZE>>::iterator iter =
           peers.begin();
       iter != peers.end(); ++iter) {
    uint8_t peerAddress[MAC_ADDR_SIZE];
    memcpy(peerAddress, iter->second.data(), MAC_ADDR_SIZE);
    if (memcmp(peerAddress, address, MAC_ADDR_SIZE) == 0) {
      (*id) = iter->first;
      return true;
    }
  }

  return false;
}

bool Peers::getPeerAddressById(const int &robotId, uint8_t *address) {
  if (peerExists(robotId)) {
    memcpy(address, peers.at(robotId).data(), MAC_ADDR_SIZE);
    return true;
  }
  return false;
}

bool Peers::deletePeer(const int &robotId) {
  uint8_t peerAddress[MAC_ADDR_SIZE];
  if (getPeerAddressById(robotId, peerAddress)) {
    esp_now_del_peer(peerAddress);
    peers.erase(robotId);
  }
  return !peerExists(robotId);
}

bool Peers::peerExists(const int &robotId) {
  return (peers.find(robotId) != peers.end());
}