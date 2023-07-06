#ifndef ARMORIAL_SUASSUNA_COMMUNICATION
#define ARMORIAL_SUASSUNA_COMMUNICATION

#include <crc/crc.h>
#include <packets/packets.h>
#include <peer/peer.h>
#include <regex>

#define BROADCAST_ADDRESS                                                      \
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
#define SEQUENTIAL_ERRORS_TO_REMOVE_PEER 10

int commandFailCounter[MAX_NUM_ROBOTS] = {0};

bool isDelimiter(const char &c) { return (c == '<' || c == '>'); }

inline int GetPlayerIdFromPacket(const ControlPacket &packet) {
  return (packet.control & 0x0F);
}

inline bool CheckIfHasPattern(const std::string& str) {
  std::smatch matches;
  std::regex regexExpression("<{3}(.*?)>{3}");

  return std::regex_search(str, matches, regexExpression);
}

inline bool ProcessPattern(std::string& str) {
  bool parsedPacket = false;
  while(CheckIfHasPattern(str)) {
    size_t startOfPattern = str.find(LEFT_DELIMITER);
    size_t endOfPattern = str.find(RIGHT_DELIMITER);

    if(startOfPattern != 0) {
      str.erase(str.begin(), str.begin() + startOfPattern);
      continue;
    }

    int matchSize = endOfPattern - startOfPattern + 3;

    std::string packetToValidate;
    for(int i = startOfPattern + 3; i < endOfPattern; i++) {
      packetToValidate += str.at(i);
    }
    str.erase(startOfPattern, matchSize);

    ControlPacket structuredPacket;
    uint8_t robotMacAddress[MAC_ADDR_SIZE] = BROADCAST_ADDRESS;
    if(packetToValidate.size() == sizeof(ControlPacket)) {
      memcpy(&structuredPacket, packetToValidate.c_str(), sizeof(ControlPacket));
      if (validate_controlpacket_crc(structuredPacket)) {
          bool hasPeerAddress = GetPeerAddress(GetPlayerIdFromPacket(structuredPacket),
                         robotMacAddress);
          esp_err_t ret = esp_now_send(robotMacAddress, (uint8_t *)packetToValidate.c_str(), sizeof(ControlPacket));
          
          if(hasPeerAddress) {
            if(ret != ESP_OK) {
              commandFailCounter[GetPlayerIdFromPacket(structuredPacket)]++;
              if(commandFailCounter[GetPlayerIdFromPacket(structuredPacket)] >= SEQUENTIAL_ERRORS_TO_REMOVE_PEER) {
                if(DelPeer(GetPlayerIdFromPacket(structuredPacket))) {
                  esp_now_del_peer(robotMacAddress);
                }
                commandFailCounter[GetPlayerIdFromPacket(structuredPacket)] = 0;
              }
            }
            else {
              commandFailCounter[GetPlayerIdFromPacket(structuredPacket)] = std::max(0, commandFailCounter[GetPlayerIdFromPacket(structuredPacket)] - 1);
            }
          }
          parsedPacket = true;
      }
    }
  }

  return parsedPacket;
}

#endif/* ARMORIAL_SUASSUNA_COMMUNICATION */
