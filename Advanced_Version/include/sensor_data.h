#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

struct SensorReading {
  uint16_t nodeId;
  uint32_t sequenceNum;
  float temperature;
  float pressure;
  float altitude;
  uint8_t hopLimit;
  uint8_t rssi;
  int8_t snr;
  uint32_t timestamp;
};

struct MeshPacket {
  uint16_t sourceId;
  uint16_t destinationId;
  uint16_t sequenceId;
  uint8_t hopCount;
  uint8_t hopLimit;
  uint8_t payloadType;  // 0x01 for sensor data
  uint16_t payloadLength;
  uint8_t* payload;
};

#endif
