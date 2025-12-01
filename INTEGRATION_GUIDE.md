# BMP280 Mesh Sensor Node - Integration Guide

## Overview

This firmware provides two versions optimized for your IoT stack:

### Version 1: Basic Sensor Node (5.79 KB)
- Simple BMP280 sensor readings
- Automatic mesh data transmission
- 21-byte sensor payload format
- Ideal for getting started quickly
- Minimal dependencies

### Version 2: Advanced Sensor Node (9.43 KB)
- Full neighbor discovery and tracking
- Sophisticated packet forwarding logic
- Hop limit management
- Statistics and performance monitoring
- Compatible with meshcore architecture

## Quick Start

### 1. Extract and Setup

```bash
# Extract the appropriate zip file
unzip BMP280_Mesh_Sensor_Node.zip
cd BMP280_Mesh_Sensor_Node

# Or for advanced version:
unzip BMP280_Mesh_Sensor_Advanced.zip
cd BMP280_Mesh_Sensor_Advanced
```

### 2. Hardware Setup

#### Wiring for WEMOS D1 Mini

**BMP280 (I2C):**
```
BMP280          D1 Mini
VCC      ----   3.3V
GND      ----   GND
SDA      ----   D2 (GPIO4)
SCL      ----   D1 (GPIO5)
```

**LoRa Radio (SPI):**
```
LoRa Module     D1 Mini
VCC      ----   3.3V
GND      ----   GND
CS       ----   D8 (GPIO15)
RST      ----   D4 (GPIO2)
DIO0     ----   D3 (GPIO0)
MOSI     ----   D7 (GPIO13)
MISO     ----   D6 (GPIO12)
SCK      ----   D5 (GPIO14)
```

### 3. Installation with PlatformIO

```bash
# Install PlatformIO (if not already installed)
pip install platformio

# Build and upload firmware
pio run -e d1_mini --target upload

# Monitor serial output
pio device monitor -b 115200
```

### 4. Installation with Arduino IDE

```
1. Open Arduino IDE
2. Install Board: ESP8266
3. Install Libraries:
   - Adafruit BMP280 Library (v2.7.2+)
   - Adafruit Unified Sensor (v1.1.14+)
4. Open BMP280_Mesh_Sensor.ino or BMP280_Mesh_Advanced.ino
5. Select Board: WEMOS D1 Mini
6. Select Port: (your COM port)
7. Click Upload
```

## Sensor Data Format

### BMP280 Measurements

The firmware reads three environmental measurements:

1. **Temperature** (°C)
   - Range: -40 to +85°C
   - Accuracy: ±1°C
   - Transmitted as: float (4 bytes)

2. **Pressure** (hPa)
   - Range: 300 to 1100 hPa
   - Accuracy: ±1 hPa
   - Transmitted as: float (4 bytes)

3. **Altitude** (meters)
   - Calculated from pressure
   - Reference: 1013.25 hPa (sea level)
   - Transmitted as: float (4 bytes)

### Packet Structure

```
┌─ Mesh Wrapper Header (5 bytes)
│  ├─ Flags: 1 byte
│  ├─ Type: 1 byte (0x01 = sensor data)
│  ├─ HopLimit: 1 byte
│  └─ TransportCode: 2 bytes
│
├─ Sensor Data Payload (21 bytes)
│  ├─ NodeID: 2 bytes (e.g., 0x0001)
│  ├─ Sequence: 4 bytes (incrementing counter)
│  ├─ Temperature: 4 bytes (float)
│  ├─ Pressure: 4 bytes (float)
│  ├─ Altitude: 4 bytes (float)
│  ├─ HopLimit: 1 byte
│  └─ CRC16: 2 bytes (error checking)
│
└─ Total Transmission: 26 bytes
```

## Configuration

### Radio Parameters

Edit in sketch or config.ini:

```cpp
#define LORA_FREQ    915.0f     // MHz (915 for US, 868 for EU)
#define LORA_BW      125.0f     // kHz (125, 250, 500)
#define LORA_SF      9          // Spreading Factor (5-12, higher=range)
#define LORA_CR      7          // Coding Rate (5-8)
#define TX_POWER     17         // dBm (1-20)
```

**Spreading Factor Guide:**
- SF 5-7: Short range, high speed (868 m typical)
- SF 8-9: Medium range (2+ km typical)
- SF 10-12: Long range, low speed (5+ km typical)

### Sensor Parameters

```cpp
#define SENSOR_INTERVAL 30      // Seconds between readings
#define BMP280_I2C_ADDR 0x76    // 0x76 or 0x77
```

### Network Parameters

```cpp
#define NODE_ID      0x0001     // Unique node identifier
#define HOP_LIMIT    5          // Max relay hops
#define NODE_NAME    "Sensor_01" // Display name
```

## Mesh Network Operation

### Transmission Strategy

1. **Read & Encode** (every 30 seconds)
   - Read BMP280 sensor
   - Create packet with sequence number
   - Calculate CRC16 checksum

2. **Transmit**
   - Send packet via LoRa with hop limit of 5
   - Waits for acknowledgment (if enabled)

3. **Relay** (if hop limit > 0)
   - Receive packet from neighbor
   - Validate CRC16
   - Decrement hop limit
   - Wait random 500-2000 ms (backoff)
   - Re-transmit to neighbors

### Multi-Hop Example

```
Sensor Node 1        Relay Node 2        Relay Node 3        Gateway
      │                    │                    │                 │
      ├──(5 hops)────────→ │                    │                 │
      │                    ├──(4 hops)────────→ │                 │
      │                    │                    ├──(3 hops)────→  │
      │                    │                    │             collect
      │                    │                    │             & store
```

## Advanced Features (Version 2)

### Neighbor Discovery

The advanced version automatically discovers nearby nodes:

```
[NEIGHBORS] Total: 3
  1. RSSI: -72 dBm, Last seen: 1234 ms ago
  2. RSSI: -85 dBm, Last seen: 5678 ms ago
  3. RSSI: -92 dBm, Last seen: 9012 ms ago
```

### Packet Statistics

Every 5 minutes, prints network health:

```
=== Mesh Network Statistics ===
TX: 42 packets
RX: 128 packets
FWD: 95 packets
Uptime: 1234 seconds
Neighbors: 3
Heap: 34560 bytes
```

### CRC16 Validation

All packets include 16-bit CRC for error detection:

```cpp
// CRC calculation uses standard X.25 polynomial
CRC-16/X-25: poly=0x1021, init=0xFFFF
```

## Power Management

### Power Consumption

- **Idle/Sleep**: ~5 mA
- **Sensor Reading**: ~15 mA
- **LoRa TX**: ~100 mA (peak)
- **LoRa RX**: ~12 mA

### Typical Usage (30s interval)
- 30 seconds idle: 5 × 30 = 150 mA·s
- 1 second TX: 100 × 1 = 100 mA·s
- 1 second RX: 12 × 1 = 12 mA·s
- **Average**: ~7.3 mA

With 2000 mAh battery: ~273 hours (~11 days)

### Optimization Tips

1. Increase SENSOR_INTERVAL (more battery life)
2. Lower spreading factor (SF) (less TX time)
3. Reduce TX power to minimum needed
4. Use sleep modes between readings

## Troubleshooting

### BMP280 Not Detected

```
[ERROR] BMP280 not found at 0x76, trying 0x77...
[FATAL] BMP280 initialization failed!
```

**Solutions:**
- Verify I2C address (check datasheet or sensor label)
- Check SCL/SDA connections
- Verify 3.3V power supply (use multimeter)
- Check for loose wires
- Try different I2C address in code

### LoRa Radio Not Responding

```
No TX/RX activity logged
```

**Solutions:**
- Verify SPI connections (CS, MOSI, MISO, SCK)
- Check RST pin is connected
- Verify antenna is attached
- Check frequency matches gateway
- Try different LoRa module (test with known-good board)

### Poor Radio Range

**Solutions:**
- Increase TX power (up to 20 dBm)
- Use external antenna (improves 3-5x)
- Lower spreading factor (SF) for reliability
- Reduce bandwidth (7.8 kHz for range)
- Add relay nodes in gap areas

### Memory Issues

```
Heap: 12345 bytes (too low!)
```

**Solutions:**
- Reduce MAX_NEIGHBORS in advanced version
- Use smaller strings for NODE_NAME
- Check for memory leaks in custom code
- Use heap analyzer tool

## Integration with Gateway/Dashboard

### Serial Protocol

Each sensor reading is logged to serial:

```
[TX #1] Sensor Reading
├─ Temp: 23.45°C
├─ Pressure: 1013.25 hPa
├─ Altitude: 45.32 m
├─ Sequence: 1
├─ Hop Limit: 5
└─ CRC: 0xA1B2
```

### Receiving Data

To build a receiver/gateway:

```cpp
// Parse received packet
MeshPacketWrapper* pkt = (MeshPacketWrapper*)rxBuffer;
SensorPacket& sensor = pkt->sensor;

float temp = sensor.temperature;
float pressure = sensor.pressure;
uint32_t nodeId = sensor.nodeId;

// Validate CRC
uint16_t crc = calculateCRC16(...);
if (crc == sensor.crc) {
  // Data is valid
}
```

## Multiple Nodes Setup

To deploy multiple sensor nodes:

### Step 1: Assign Unique Node IDs

Edit in each sketch:
```cpp
#define NODE_ID 0x0001  // Change for each node
```

### Step 2: Configure Network

Ensure all nodes use same:
- LORA_FREQ (915.0 MHz)
- LORA_BW (125.0 kHz)
- LORA_SF (9)
- LORA_CR (7)

### Step 3: Deploy in Mesh

Position nodes to overlap coverage:

```
┌────────────────────────────────────────┐
│                                        │
│  Sensor1        Relay1        Relay2   │
│    •───────600m───────•───600m───•    │
│                                        │
│  Sensor2        Gateway               │
│    •                  •                │
│                                        │
└────────────────────────────────────────┘
```

## Performance Metrics

### Range Testing

With SF=9, BW=125 kHz:
- Line of sight: 3-5 km typical
- Urban: 1-2 km
- With external antenna: 5-10+ km

### Latency

- Single hop: ~100-200 ms
- Multi-hop (3 hops): ~500-1500 ms
- Includes random backoff delays

### Throughput

- Maximum: ~1 packet/second
- Recommended: 1 packet/30 seconds (power efficient)
- With 50 relay nodes: ~5-10 packets/minute aggregate

## Next Steps

1. **Test Single Node**: Deploy one sensor, verify readings
2. **Add Relay Node**: Add second node to test forwarding
3. **Deploy Gateway**: Set up receiver/dashboard
4. **Scale Network**: Add more sensors as needed
5. **Monitor**: Track statistics and optimize

## Support Resources

- BMP280 Datasheet: [Bosch Sensortec]
- ESP8266 Documentation: [Espressif Systems]
- LoRa Specification: [LoRa Alliance]
- Arduino Reference: [Arduino Official]

## License

Both firmware versions are provided as-is for research and personal projects.
Modify and distribute freely with attribution.

---

**Ready to deploy?** Start with the basic version, test single nodes, then scale up!
