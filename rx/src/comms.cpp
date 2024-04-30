#include <stdint.h>
#include <stddef.h>

#include "comms.h"

const char *startSeq = START_SEQ;

uint8_t calcCrc8(uint8_t *data, size_t len) {
  uint8_t crc = 0;

  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ CRC8_POLY;
        } else {
            crc <<= 1;
            
        }
    }
  }

  return crc;
}
