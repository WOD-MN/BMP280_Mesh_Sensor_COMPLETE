# BMP280 Mesh Sensor Node

A simple IoT sensor node firmware for ESP8266 that reads BMP280 environmental sensor data and transmits it via mesh network.

## Features

- **BMP280 Sensor Support**: Temperature, Pressure, and Altitude readings
- **Mesh Network**: Multi-hop packet delivery with hop limit control
- **Simple Architecture**: No complex messaging features - pure sensor data transmission
- **Automatic Forwarding**: Packets are relayed through network hops
- **CRC Validation**: Data integrity checking
- **Serial Monitoring**: Real-time sensor data and network status

## Hardware Requirements

- ESP8266 (WEMOS D1 Mini recommended)
- BMP280 Sensor Module (I2C)
- LoRa Radio Module (SPI)
- Appropriate USB cable for programming

## Wiring

### BMP280 (I2C)
- SDA -> GPIO4 (D2)
- SCL -> GPIO5 (D1)
- VCC -> 3.3V
- GND -> GND

### LoRa Radio (SPI)
- CS  -> GPIO15 (D8)
- RST -> GPIO2 (D4)
- DIO0 -> GPIO0 (D3)
- MOSI -> GPIO13 (D7)
- MISO -> GPIO12 (D6)
- SCK -> GPIO14 (D5)
- VCC -> 3.3V
- GND -> GND

## Installation

### Using PlatformIO

```bash
pio run -e d1_mini --target upload
pio device monitor -b 115200
```

### Using Arduino IDE

1. Install required libraries:
   - Adafruit BMP280 Library
   - Adafruit Unified Sensor

2. Select Board: WEMOS D1 Mini
3. Select Port: (your COM port)
4. Upload sketch

## Configuration

Edit the defines in `BMP280_Mesh_Sensor.ino`:

```cpp
#define LORA_FREQ    915.0f     // LoRa frequency (MHz)
#define LORA_BW      125.0f     // Bandwidth (kHz)
#define LORA_SF      9          // Spreading factor (5-12)
#define LORA_CR      7          // Coding rate (5-8)
#define TX_POWER     20         // TX Power (dBm)
#define SENSOR_INTERVAL 30      // Reading interval (seconds)
```

## Sensor Data Format

Each transmitted packet contains:

```
Struct SensorData {
  uint16_t nodeId;           // 2 bytes
  uint32_t counter;          // 4 bytes
  float temperature;         // 4 bytes
  float pressure;            // 4 bytes
  float altitude;            // 4 bytes
  uint8_t hopLimit;          // 1 byte
  uint16_t crc;              // 2 bytes
}
```

Total payload: 21 bytes

## Mesh Network Operation

### Packet Transmission
1. Node reads sensor data
2. Creates packet with unique sequence number
3. Sets hopLimit to 5 (configurable)
4. Calculates CRC16 checksum
5. Transmits via LoRa radio

### Packet Forwarding
1. Node receives packet with hopLimit > 0
2. Validates CRC16
3. Decrements hopLimit
4. Re-transmits with reduced power after random delay

### Neighbor Discovery
- Nodes automatically discover neighbors within radio range
- Maintains list of recent peers
- Tracks RSSI and SNR metrics

## Serial Output Example

```
=== BMP280 Mesh Sensor Node ===
Node ID: 0x0001
Initializing BMP280...
BMP280 initialized successfully
Initializing LoRa radio...
LoRa radio pins configured
LoRa Frequency: 915.0 MHz
Bandwidth: 125.0 kHz, SF: 9, CR: 7

Setup complete. Starting sensor readings...

=== Sensor Reading #1 ===
Temperature: 23.45 C
Pressure: 1013.25 hPa
Altitude: 45.32 m
CRC: 0xA1B2
Ready to transmit via mesh network...
```

## Troubleshooting

**BMP280 not found:**
- Check I2C address (0x76 or 0x77)
- Verify SCL/SDA connections
- Check power supply

**No LoRa signal:**
- Verify radio frequency matches gateway
- Check antenna connection
- Increase TX power (up to 20 dBm)

## Node IDs

Each sensor node should have a unique ID:

```cpp
uint16_t nodeId = 0x0001;  // Change this for each node
```

## Future Enhancements

- GPS integration for location tracking
- Battery monitoring and low-power mode
- Web dashboard for real-time data
- SD card logging support
- MQTT gateway integration

## License

Open source - Feel free to modify and distribute

## Support

For issues or questions, check:
- BMP280 datasheet
- ESP8266 documentation
- LoRa radio module manual
