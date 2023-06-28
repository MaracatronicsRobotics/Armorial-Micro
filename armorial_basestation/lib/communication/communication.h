#ifndef ARMORIAL_SUASSUNA_COMMUNICATION_H
#define ARMORIAL_SUASSUNA_COMMUNICATION_H

#include <crc/crc.h>
#include <packets/packets.h>
#include <peer/peer.h>
#include <regex>

#define BROADCAST_ADDRESS                                                      \
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }

const String startDelimiter("<<<");
const String endDelimiter(">>>");
const char dummyByte = '0';

bool isDelimiter(const char &c) { return (c == '<' || c == '>'); }

inline int GetPlayerIdFromPacket(const ControlPacket &packet) {
  return (packet.control & 0x0F);
}

inline bool ProcessAndSendControl(char *data, const long &size) {
  std::string dataAsStr;
  for (int i = 0; i < size; i++) {
    dataAsStr += data[i];
  }

  std::smatch matches;
  std::regex regexExpression("<{3}(.*?)>{3}");

  ControlPacket structuredPacket;
  uint8_t robotMacAddress[MAC_ADDR_SIZE] = BROADCAST_ADDRESS;
  bool parsedPacket = false;
  while (std::regex_search(dataAsStr, matches, regexExpression)) {
    for (auto match : matches) {
      if (match.length() == sizeof(ControlPacket)) {
        char buff[sizeof(ControlPacket)];
        memcpy(buff, match.str().c_str(), sizeof(ControlPacket));
        memcpy(&structuredPacket, buff, sizeof(ControlPacket));
        if (validate_controlpacket_crc(structuredPacket)) {
          GetPeerAddress(GetPlayerIdFromPacket(structuredPacket),
                         robotMacAddress);
          esp_now_send(robotMacAddress, (uint8_t *)&buff, sizeof(buff));
          parsedPacket = true;
        }
      }
      dataAsStr = matches.suffix().str();
    }
  }
  return parsedPacket;
}

#endif // ARMORIAL_SUASSUNA_COMMUNICATION_H
