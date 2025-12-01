#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>

// LoRa Radio pins (adjust for your board)
#define LORA_CS    D8  // GPIO15
#define LORA_RST   D4  // GPIO2
#define LORA_DIO0  D3  // GPIO0

// I2C pins for BMP280
#define BMP_SDA D2  // GPIO4
#define BMP_SCL D1  // GPIO5

// Radio parameters (from meshcore recommendations)
#define LORA_FREQ  915.0f
#define LORA_BW    125.0f
#define LORA_SF    9
#define LORA_CR    7
#define TX_POWER   17

// Sensor reading interval (seconds)
#define SENSOR_INTERVAL 30

// Mesh network parameters
#define MAX_HOP_LIMIT   8
#define NODE_NAME       "BMP280_Sensor_01"
#define MESH_VERSION    1

// ========================================
// Advanced BMP280 Mesh Sensor Node
// Using meshcore architecture patterns
// ========================================

Adafruit_BMP280 bmp;
unsigned long lastSensorTime = 0;
uint32_t packetCounter = 0;
uint32_t txCount = 0;
uint32_t fwdCount = 0;
uint32_t rxCount = 0;

// Neighbor tracking (simplified)
struct Neighbor {
  uint8_t pubkey[32];
  int8_t lastRSSI;
  uint32_t lastSeen;
};

#define MAX_NEIGHBORS 10
Neighbor neighbors[MAX_NEIGHBORS];
uint8_t numNeighbors = 0;

// Sensor data packet format
#pragma pack(push, 1)
struct SensorPacket {
  uint16_t nodeId;        // 2 bytes
  uint32_t sequenceNum;   // 4 bytes
  float temperature;      // 4 bytes
  float pressure;         // 4 bytes
  float altitude;         // 4 bytes
  uint8_t hopLimit;       // 1 byte
  uint16_t crc;           // 2 bytes
};
#pragma pack(pop)

// Mesh packet wrapper (compatible with meshcore)
#pragma pack(push, 1)
struct MeshPacketWrapper {
  uint8_t flags;          // 1 byte
  uint8_t type;           // 1 byte (0x01 = sensor data)
  uint8_t hopLimit;       // 1 byte
  uint16_t transportCode; // 2 bytes (for routing)
  SensorPacket sensor;    // 21 bytes
};
#pragma pack(pop)

// ========================================
// CRC16 Implementation
// ========================================
uint16_t calculateCRC16(const uint8_t* data, size_t len) {
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

// ========================================
// BMP280 Sensor Functions
// ========================================
void setupBMP280() {
  Serial.println("[SETUP] Initializing BMP280 sensor...");

  if (!bmp.begin(0x76)) {
    Serial.println("[ERROR] BMP280 not found at 0x76, trying 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("[FATAL] BMP280 initialization failed!");
      while (1) {
        delay(1000);
        Serial.println("[ERROR] BMP280 not detected");
      }
    }
  }

  // Configure sensor sampling
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );

  Serial.println("[OK] BMP280 initialized");
}

// ========================================
// Radio Functions
// ========================================
void setupRadio() {
  Serial.println("[SETUP] Initializing LoRa radio...");

  SPI.begin();

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(100);

  Serial.printf("[CONFIG] Frequency: %.1f MHz\n", LORA_FREQ);
  Serial.printf("[CONFIG] Bandwidth: %.1f kHz, SF: %d, CR: %d\n", 
                LORA_BW, LORA_SF, LORA_CR);
  Serial.printf("[CONFIG] TX Power: %d dBm\n", TX_POWER);
  Serial.println("[OK] LoRa radio configured");
}

// ========================================
// Sensor Reading
// ========================================
void readAndTransmitSensorData() {
  if (millis() - lastSensorTime < (SENSOR_INTERVAL * 1000)) {
    return;
  }

  lastSensorTime = millis();
  packetCounter++;

  // Read sensor values
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0f;
  float altitude = bmp.readAltitude(1013.25);

  // Build sensor packet
  SensorPacket sensorData;
  sensorData.nodeId = 0x0001;
  sensorData.sequenceNum = packetCounter;
  sensorData.temperature = temperature;
  sensorData.pressure = pressure;
  sensorData.altitude = altitude;
  sensorData.hopLimit = 5;

  // Calculate CRC
  uint16_t crc = calculateCRC16(
    (const uint8_t*)&sensorData,
    sizeof(SensorPacket) - 2
  );
  sensorData.crc = crc;

  // Wrap in mesh packet
  MeshPacketWrapper wrapper;
  wrapper.flags = 0x00;
  wrapper.type = 0x01;  // Sensor data
  wrapper.hopLimit = 5;
  wrapper.transportCode = 0x0001;
  wrapper.sensor = sensorData;

  // Log transmission
  Serial.printf("\n[TX #%lu] Sensor Reading\n", packetCounter);
  Serial.printf("├─ Temp: %.2f°C\n", temperature);
  Serial.printf("├─ Pressure: %.2f hPa\n", pressure);
  Serial.printf("├─ Altitude: %.2f m\n", altitude);
  Serial.printf("├─ Sequence: %lu\n", packetCounter);
  Serial.printf("├─ Hop Limit: %d\n", wrapper.hopLimit);
  Serial.printf("└─ CRC: 0x%04X\n", crc);

  txCount++;

  // TODO: Send via LoRa radio
  // radioSendPacket((uint8_t*)&wrapper, sizeof(wrapper));
}

// ========================================
// Mesh Packet Reception (Placeholder)
// ========================================
void processReceivedPacket(uint8_t* data, size_t len) {
  if (len < sizeof(MeshPacketWrapper)) {
    return;
  }

  MeshPacketWrapper* pkt = (MeshPacketWrapper*)data;
  rxCount++;

  // Validate CRC
  uint16_t expectedCRC = calculateCRC16(
    (const uint8_t*)&pkt->sensor,
    sizeof(SensorPacket) - 2
  );

  if (pkt->sensor.crc != expectedCRC) {
    Serial.printf("[RX ERROR] CRC mismatch: got 0x%04X, expected 0x%04X\n",
                  pkt->sensor.crc, expectedCRC);
    return;
  }

  // Log reception
  Serial.printf("\n[RX] Sensor Data from Node 0x%04X\n", pkt->sensor.nodeId);
  Serial.printf("├─ Sequence: %lu\n", pkt->sensor.sequenceNum);
  Serial.printf("├─ Temp: %.2f°C\n", pkt->sensor.temperature);
  Serial.printf("├─ Pressure: %.2f hPa\n", pkt->sensor.pressure);
  Serial.printf("├─ Hop Limit: %d\n", pkt->hopLimit);
  Serial.printf("└─ Status: OK\n");

  // Forward if hop limit > 0
  if (pkt->hopLimit > 0) {
    MeshPacketWrapper fwdPkt = *pkt;
    fwdPkt.hopLimit--;

    // Add random delay (recommended by meshcore)
    unsigned long delay_ms = random(500, 2000);

    Serial.printf("[FWD] Scheduling forward in %lu ms (hop %d)\n",
                  delay_ms, fwdPkt.hopLimit);

    // TODO: Schedule packet forwarding
    // scheduledForward((uint8_t*)&fwdPkt, sizeof(fwdPkt), delay_ms);
    fwdCount++;
  }
}

// ========================================
// Neighbor Management
// ========================================
void addNeighbor(const uint8_t* pubkey, int8_t rssi) {
  if (numNeighbors < MAX_NEIGHBORS) {
    memcpy(neighbors[numNeighbors].pubkey, pubkey, 32);
    neighbors[numNeighbors].lastRSSI = rssi;
    neighbors[numNeighbors].lastSeen = millis();
    numNeighbors++;

    Serial.printf("[NEIGHBOR] Added neighbor #%d (RSSI: %d)\n", 
                  numNeighbors, rssi);
  }
}

void printNeighbors() {
  Serial.printf("\n[NEIGHBORS] Total: %d\n", numNeighbors);
  for (int i = 0; i < numNeighbors; i++) {
    Serial.printf("  %d. RSSI: %d dBm, Last seen: %lu ms ago\n",
                  i + 1,
                  neighbors[i].lastRSSI,
                  millis() - neighbors[i].lastSeen);
  }
}

// ========================================
// Statistics
// ========================================
void printStatistics() {
  Serial.println("\n=== Mesh Network Statistics ===");
  Serial.printf("TX: %lu packets\n", txCount);
  Serial.printf("RX: %lu packets\n", rxCount);
  Serial.printf("FWD: %lu packets\n", fwdCount);
  Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
  Serial.printf("Neighbors: %d\n", numNeighbors);
  Serial.printf("Heap: %u bytes\n", ESP.getFreeHeap());
}

// ========================================
// Setup
// ========================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n\n=== BMP280 Mesh Sensor Node ===");
  Serial.printf("Firmware Version: %d\n", MESH_VERSION);
  Serial.printf("Node Name: %s\n", NODE_NAME);
  Serial.printf("Node ID: 0x0001\n");

  // Initialize I2C for BMP280
  Wire.begin(BMP_SDA, BMP_SCL);

  // Initialize sensor
  setupBMP280();

  // Initialize radio
  setupRadio();

  // Seed random for mesh timing
  randomSeed(analogRead(A0));

  Serial.println("\n[OK] All systems ready\n");
}

// ========================================
// Main Loop
// ========================================
void loop() {
  // Read and transmit sensor data at interval
  readAndTransmitSensorData();

  // TODO: Process received packets
  // if (radioDataAvailable()) {
  //   uint8_t buffer[256];
  //   size_t len = radioReceivePacket(buffer);
  //   processReceivedPacket(buffer, len);
  // }

  // Print statistics every 5 minutes
  static unsigned long lastStats = 0;
  if (millis() - lastStats > (5 * 60 * 1000)) {
    lastStats = millis();
    printStatistics();
  }

  // Print neighbors every minute
  static unsigned long lastNeighbors = 0;
  if (millis() - lastNeighbors > (1 * 60 * 1000)) {
    lastNeighbors = millis();
    printNeighbors();
  }

  yield();
}
