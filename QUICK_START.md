# BMP280 Mesh Sensor Node - Quick Start

## What's Included

This package contains everything you need to build a mesh network sensor node:

### Two Firmware Versions:
- **Basic**: Lightweight, sensor data transmission only (5.79 KB)
- **Advanced**: Full mesh features with neighbor tracking (9.43 KB)

### Complete Documentation:
- Integration guide with troubleshooting
- Hardware wiring diagrams
- Configuration reference
- Build & deployment guide

## 5-Minute Setup

### 1. Extract
```bash
unzip BMP280_Mesh_Sensor_COMPLETE.zip
cd Basic_Version  # or Advanced_Version
```

### 2. Wire Hardware
- BMP280 SDA → D2, SCL → D1
- LoRa CS → D8, RST → D4, DIO0 → D3

### 3. Install & Upload
```bash
pip install platformio
pio run -e d1_mini --target upload
pio device monitor -b 115200
```

### 4. Verify
Watch serial output for:
```
[OK] BMP280 initialized
[OK] LoRa radio configured
[TX #1] Sensor Reading
```

Done! Your sensor node is now transmitting!

## Choose Your Version

### Basic Version (Start Here)
✓ Simple & quick
✓ Minimal code
✓ 5.79 KB firmware
✓ Good for first-time setup
→ Use this to learn mesh basics

### Advanced Version (Full Features)
✓ Neighbor discovery
✓ Statistics tracking
✓ Sophisticated forwarding
✓ 9.43 KB firmware
→ Use this for production networks

## Next Steps

1. **Single Node Test**: Upload to one board, verify sensor readings
2. **Multi-Node Setup**: Deploy 2-3 nodes to test mesh routing
3. **Build Gateway**: Add receiver node with data logging
4. **Scale Network**: Add more sensors as needed

## Key Features

- Automatic mesh packet routing (multi-hop)
- BMP280 sensor: temperature, pressure, altitude
- CRC16 error checking
- LoRa 915 MHz (adjustable)
- Low power operation
- Serial debugging output

## Wiring Reference

```
ESP8266 WEMOS D1 Mini
├─ BMP280 (I2C)
│  ├─ SDA → D2 (GPIO4)
│  ├─ SCL → D1 (GPIO5)
│  └─ VCC → 3.3V
│
└─ LoRa Module (SPI)
   ├─ CS  → D8 (GPIO15)
   ├─ RST → D4 (GPIO2)
   ├─ DIO0 → D3 (GPIO0)
   └─ MOSI → D7, MISO → D6, SCK → D5
```

## Troubleshooting

**BMP280 not found?**
- Check I2C address (0x76 or 0x77)
- Verify SCL/SDA connections
- Check 3.3V power supply

**No LoRa signal?**
- Verify SPI connections
- Check antenna is attached
- Match gateway frequency

**Memory issues?**
- Use basic version first
- Check heap usage in serial output
- Free up unused libraries

## Radio Settings

```cpp
Frequency: 915.0 MHz (US) or 868 MHz (EU)
Bandwidth: 125 kHz (can adjust)
Spreading Factor: 9 (SF 5-12)
Coding Rate: 7 (CR 5-8)
TX Power: 17 dBm (up to 20)
```

## Support

Full integration guide included with:
- Detailed wiring diagrams
- Configuration reference
- Multi-node deployment guide
- Performance metrics
- Advanced troubleshooting

## Ready?

1. Choose your version (Basic or Advanced)
2. Follow hardware wiring
3. Run `pio run -e d1_mini --target upload`
4. Watch serial output
5. Deploy additional nodes

Enjoy your mesh network! 🚀
