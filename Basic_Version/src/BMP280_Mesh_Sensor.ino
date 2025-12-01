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

// Radio parameters
#define LORA_FREQ  915.0f
#define LORA_BW    125.0f
#define LORA_SF    9
#define LORA_CR    7
#define TX_POWER   20

// Sensor reading interval (seconds)
#define SENSOR_INTERVAL 30

// ========================================
// Simplified Mesh Sensor Node
// ========================================

Adafruit_BMP280 bmp;
unsigned long lastSensorTime = 0;
uint16_t nodeId = 0x0001;  // Unique node ID
uint32_t packetCounter = 0;

struct SensorData {
  uint16_t nodeId;
  uint32_t counter;
  float temperature;
  float pressure;
  float altitude;
  uint8_t hopLimit;
  uint16_t crc;
};

// Simple CRC16 calculation
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

void setupBMP280() {
  Serial.println("Initializing BMP280...");

  if (!bmp.begin(0x76)) {  // 0x76 or 0x77 depending on SDO pin
    Serial.println("ERROR: BMP280 not found!");
    while (1) {
      delay(1000);
      Serial.println("BMP280 initialization failed. Check wiring.");
    }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  Serial.println("BMP280 initialized successfully");
}

void setupRadio() {
  Serial.println("Initializing LoRa radio...");

  // SPI initialization
  SPI.begin();

  // Setup pins
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(100);

  Serial.println("LoRa radio pins configured");
  Serial.printf("LoRa Frequency: %.1f MHz\n", LORA_FREQ);
  Serial.printf("Bandwidth: %.1f kHz, SF: %d, CR: %d\n", LORA_BW, LORA_SF, LORA_CR);
}

void sendSensorData() {
  if (millis() - lastSensorTime < (SENSOR_INTERVAL * 1000)) {
    return;  // Not time yet
  }

  lastSensorTime = millis();

  // Read sensor data
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0f;  // Convert to hPa
  float altitude = bmp.readAltitude(1013.25);    // Approximate altitude

  packetCounter++;

  // Create sensor data packet
  SensorData data;
  data.nodeId = nodeId;
  data.counter = packetCounter;
  data.temperature = temperature;
  data.pressure = pressure;
  data.altitude = altitude;
  data.hopLimit = 5;  // Number of hops allowed
  data.crc = 0;

  // Calculate CRC
  uint16_t crc = calculateCRC16((uint8_t*)&data, sizeof(data) - 2);
  data.crc = crc;

  // Print to serial
  Serial.printf("\n=== Sensor Reading #%lu ===\n", packetCounter);
  Serial.printf("Temperature: %.2f C\n", temperature);
  Serial.printf("Pressure: %.2f hPa\n", pressure);
  Serial.printf("Altitude: %.2f m\n", altitude);
  Serial.printf("CRC: 0x%04X\n", crc);
  Serial.println("Ready to transmit via mesh network...");

  // TODO: Transmit data via LoRa radio
  // The actual mesh transmission would happen here
  // This is where you'd call your mesh library's transmit function
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== BMP280 Mesh Sensor Node ===");
  Serial.printf("Node ID: 0x%04X\n", nodeId);

  // Initialize I2C
  Wire.begin(BMP_SDA, BMP_SCL);

  // Initialize sensors
  setupBMP280();

  // Initialize radio
  setupRadio();

  Serial.println("\nSetup complete. Starting sensor readings...");
}

void loop() {
  // Send sensor data at interval
  sendSensorData();

  // Simulate mesh network operations
  // - Receive incoming packets
  // - Forward packets with reduced hop limit
  // - Maintain neighbor table

  yield();
}
