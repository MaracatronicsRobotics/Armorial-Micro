#ifndef ARMORIAL_SUASSUNA_PACKET_H
#define ARMORIAL_SUASSUNA_PACKET_H

#include <stdint.h>

typedef struct ssl_pack {
  char control;
  float wv1;
  float wv2;
  float wv3;
  float wv4;
  char solenoid;
  uint16_t crc;
} ssl_pack;

typedef struct struct_feedback {
  char identifier;
  float wvl, wvl_encoder;
  float wvr, wvr_encoder;
} struct_feedback;

#endif // ARMORIAL_SUASSUNA_PACKET_H
