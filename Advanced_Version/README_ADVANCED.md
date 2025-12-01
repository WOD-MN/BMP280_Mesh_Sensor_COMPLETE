# BMP280 Mesh Sensor Node (Advanced)

An enhanced IoT sensor node firmware for ESP8266 with full meshcore architecture integration patterns.

## Features

- **Advanced Mesh Networking**: Multi-hop packet forwarding with hop limit
- **Neighbor Discovery**: Tracks nearby nodes with RSSI monitoring
- **Packet Routing**: Intelligent forwarding with random backoff delay
- **Statistics Tracking**: TX/RX/FWD packet counts and network health
- **CRC16 Validation**: Packet integrity checking
- **Structured Logging**: Detailed debug output with categories
- **Memory Efficient**: Optimized for ESP8266 constraints

## Packet Structure

### Mesh Packet Format
```
[Flags: 1B][Type: 1B][HopLimit: 1B][TransportCode: 2B][SensorData: 21B]
Total: 26 bytes
```

### Sensor Data Format
```
[NodeID: 2B][Sequence: 4B][Temp: 4B][Pressure: 4B][Altitude: 4B][Hop: 1B][CRC: 2B]
Total: 21 bytes
```

## Building

### With PlatformIO
```bash
pio run -e d1_mini --target upload
pio device monitor -b 115200
```

### With Arduino IDE
1. Open `BMP280_Mesh_Advanced.ino`
2. Select Board: WEMOS D1 Mini
3. Upload

## Configuration

Edit defines in sketch:

```cpp
#define SENSOR_INTERVAL 30    // Reading interval (seconds)
#define MAX_HOP_LIMIT   8     // Maximum hop limit
#define TX_POWER        17    // TX power (dBm)
#define NODE_NAME       "BMP280_Sensor_01"
```

## Mesh Network Behavior

### Packet Transmission
1. Reads sensor every 30 seconds
2. Creates packet with unique sequence number
3. Sets hop limit to 5 (allows 5 relay hops)
4. Calculates CRC16 checksum
5. Transmits via LoRa radio

### Packet Reception
1. Validates CRC16 checksum
2. Logs sensor reading
3. If hopLimit > 0:
   - Decrements hopLimit
   - Waits random 500-2000ms (per meshcore recommendation)
   - Re-transmits with reduced power

### Neighbor Discovery
- Maintains list of discovered nodes
- Tracks RSSI (signal strength) for each neighbor
- Updates last-seen timestamp
- Max 10 neighbors tracked simultaneously

## Serial Output Example

```
=== BMP280 Mesh Sensor Node ===
Firmware Version: 1
Node Name: BMP280_Sensor_01
Node ID: 0x0001
[SETUP] Initializing BMP280 sensor...
[OK] BMP280 initialized
[SETUP] Initializing LoRa radio...
[CONFIG] Frequency: 915.0 MHz
[CONFIG] Bandwidth: 125.0 kHz, SF: 9, CR: 7
[CONFIG] TX Power: 17 dBm
[OK] LoRa radio configured

[OK] All systems ready

[TX #1] Sensor Reading
├─ Temp: 23.45°C
├─ Pressure: 1013.25 hPa
├─ Altitude: 45.32 m
├─ Sequence: 1
├─ Hop Limit: 5
└─ CRC: 0xA1B2

[RX] Sensor Data from Node 0x0002
├─ Sequence: 42
├─ Temp: 24.12°C
├─ Pressure: 1012.50 hPa
├─ Hop Limit: 3
└─ Status: OK

[FWD] Scheduling forward in 1234 ms (hop 2)
```

## Meshcore Integration

This firmware follows meshcore patterns:

- **Packet Structure**: Uses transportCode for routing
- **Hop Limiting**: Decrements and re-forwards packets
- **CRC16**: Validates all sensor data
- **Random Backoff**: 500-2000ms delay before forwarding
- **Neighbor Tracking**: Maintains peer information
- **Statistics**: Tracks TX/RX/FWD metrics

### Extending for Full meshcore

To integrate with full meshcore library:

1. Include meshcore headers:
```cpp
#include <Mesh.h>
#include <BaseChatMesh.h>
```

2. Create mesh instance:
```cpp
class SensorNodeMesh : public BaseChatMesh {
  // Implement required methods
};
```

3. Use mesh API for routing:
```cpp
mesh->sendMessage(...);
mesh->broadcastPacket(...);
```

## Power Consumption

- Sleep: ~10 mA
- Sensor reading: ~15 mA
- LoRa TX: ~100 mA (peak)
- Typical average: 30-50 mA (with 30s interval)

## Antenna Recommendation

- External antenna: +6-9 dBi gain
- PCB antenna: -2-3 dBi gain
- Distance improvement: ~3x with external antenna

## Troubleshooting

### No sensor readings
- Check I2C address (0x76 or 0x77)
- Verify SCL/SDA wiring
- Check power supply to sensor

### Poor radio range
- Verify antenna connection
- Increase TX power (up to 20 dBm)
- Check frequency matches gateway
- Reduce spreading factor (SF) for shorter range

### Packet loss in forwarding
- Increase hop limit
- Reduce SF for more reliable short-range hops
- Add more relay nodes in area
- Check CRC validation (buffer overflow?)

## Future Enhancements

- LoRa gateway integration
- MQTT bridge support
- SD card data logging
- Low-power sleep modes
- GPS integration
- Multiple sensor types
- Web dashboard

## License

Open source - Modify and distribute freely

## References

- meshcore Architecture
- BMP280 Datasheet
- ESP8266 Specifications
- LoRa Modulation Guide
