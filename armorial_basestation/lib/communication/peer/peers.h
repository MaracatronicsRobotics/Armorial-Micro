#ifndef ARMORIAL_SUASSUNA_PEER_H
#define ARMORIAL_SUASSUNA_PEER_H

#include <Arduino.h>
#include <esp_now.h>
#include <map>
#include <vector>

#define MAC_ADDR_SIZE 6

#define CONFIG_WIFI_CHANNEL 11
#define CONFIG_ESPNOW_CHANNEL 0
#define CONFIG_ESPNOW_ENCRYPT false

class Peers {
public:
  Peers();

  static void insertPeer(const int &robotId,
                         const std::array<uint8_t, MAC_ADDR_SIZE> &address);
  static bool getPeerIdByAddress(const uint8_t *address, int *id);
  static bool getPeerAddressById(const int &robotId, uint8_t *address);
  static bool deletePeer(const int &robotId);
  static bool peerExists(const int &robotId);

private:
  static std::map<int, std::array<uint8_t, MAC_ADDR_SIZE>> peers;
  static esp_now_peer_info_t peerInfo;
};

#endif // ARMORIAL_SUASSUNA_PEER_H
