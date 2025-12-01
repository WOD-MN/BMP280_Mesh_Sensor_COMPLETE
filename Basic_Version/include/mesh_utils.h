#ifndef MESH_UTILS_H
#define MESH_UTILS_H

#include <stdint.h>

class MeshUtils {
public:
  static uint16_t calculateCRC16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
      crc ^= data[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc = (crc >> 1) ^ 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    return crc;
  }

  static void toHex(char* dest, const uint8_t* src, size_t len) {
    for (size_t i = 0; i < len; i++) {
      sprintf(&dest[i * 2], "%02X", src[i]);
    }
    dest[len * 2] = 0;
  }
};

#endif
