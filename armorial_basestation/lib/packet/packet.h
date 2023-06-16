#ifndef ARMORIAL_SUASSUNA_PACKET_H
#define ARMORIAL_SUASSUNA_PACKET_H

#include "peer.h"
#include <regex>
#include <stdint.h>

const String startDelimiter("<<<");
const String endDelimiter(">>>");
const char dummyByte = '0';

// Feedback packet
typedef struct {
  char control;
  uint8_t batteryPercentage;
  bool infraRedStatus;
  float vw1, vw2, vw3, vw4;
  float vw1_encoder, vw2_encoder, vw3_encoder, vw4_encoder;
  uint64_t timestamp;
  uint16_t crc;
} FeedbackPacket;

// Control packet
typedef struct {
  char control;
  float vw1, vw2, vw3, vw4;
  char solenoidPower;
  uint16_t crc;
} ControlPacket;

bool isDelimiter(const char &c) { return (c == '<' || c == '>'); }

inline int GetPlayerIdFromPacket(const ControlPacket &packet) {
  return (packet.control & 0x0F);
}

inline bool ProcessAndSendControl(char *data, const long &size) {
  std::string dataAsStr;
  for (int i = 0; i < size; i++) {
    if (isDelimiter(data[i])) {
      dataAsStr += data[i];
    } else {
      dataAsStr += dummyByte;
    }
  }

  std::smatch m;
  std::regex e("<{3}(.*?)>{3}");

  ControlPacket structuredPacket;
  uint8_t robotMacAddress[MAC_ADDR_SIZE];
  bool parsedPacket = false;
  while (std::regex_search(dataAsStr, m, e)) {
    for (auto x : m) {
      if (x.length() == sizeof(ControlPacket)) {
        char buff[sizeof(ControlPacket)];
        memcpy(buff, x.str().c_str(), sizeof(ControlPacket));
        memcpy(&structuredPacket, buff, sizeof(ControlPacket));
        if (GetPeerAddress(GetPlayerIdFromPacket(structuredPacket),
                           robotMacAddress)) {
          esp_now_send(robotMacAddress, (uint8_t *)&buff, sizeof(buff));
        }
        parsedPacket = true;
      }
    }
    dataAsStr = m.suffix().str();
  }

  return parsedPacket;
}

#endif // ARMORIAL_SUASSUNA_PACKET_H
