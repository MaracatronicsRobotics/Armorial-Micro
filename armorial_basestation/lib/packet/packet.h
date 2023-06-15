#ifndef ARMORIAL_SUASSUNA_PACKET_H
#define ARMORIAL_SUASSUNA_PACKET_H

#include "peer.h"
#include <stdint.h>

const char startDelimiter = '<';
const char endDelimiter = '>';

typedef struct struct_control {
  char control;
  float wv1;
  float wv2;
  float wv3;
  float wv4;
  char solenoid;
  uint16_t crc;
} struct_control;

typedef struct struct_feedback {
  char identifier;
  float wvl, wvl_encoder;
  float wvr, wvr_encoder;
} struct_feedback;

inline int GetPlayerIdFromPacket(const struct_control &packet) {
  return (packet.control & 0x0F);
}

inline bool ProcessAndSendControl(char *data, const long &size) {
  String dataAsStr(data);
  String strBuff;
  struct_control structuredPacket;
  uint8_t robotMacAddress[MAC_ADDR_SIZE];
  bool parsedPacket = false;

  for (int i = 0; i < dataAsStr.length(); i++) {
    if (dataAsStr[i] == startDelimiter) {
      strBuff.clear();
    } else if (dataAsStr[i] != endDelimiter) {
      strBuff += dataAsStr[i];
    } else if (dataAsStr[i] == endDelimiter) {
      if (strBuff.length() == sizeof(struct_control)) {
        char buff[sizeof(struct_control)];
        strBuff.toCharArray(buff, sizeof(struct_control));
        memcpy(&structuredPacket, buff, sizeof(struct_control));
        if (GetPeerAddress(GetPlayerIdFromPacket(structuredPacket),
                           robotMacAddress)) {
          esp_now_send(robotMacAddress, (uint8_t *)&buff, sizeof(buff));
        }
        parsedPacket = true;
      }
      strBuff.clear();
    }
  }

  return parsedPacket;
}

#endif // ARMORIAL_SUASSUNA_PACKET_H
