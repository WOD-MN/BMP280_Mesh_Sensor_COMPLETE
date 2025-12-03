#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include "esp_system.h"


// ========================================================
// BUILD‑TIME CONFIG
// ========================================================
// Select region: 0=EU868, 1=US915, 2=AS923
#define REGION_PROFILE      0
// Node role flags
#define ROLE_SENSOR         1   // 1 = send sensor data
#define ROLE_ROUTER         1   // 1 = forward / mesh
#define ROLE_SINK           0   // 1 = logical sink node (dest 0x0000)


// Radio profiles
// 0 = long‑range (slower), 1 = fast (shorter range)
#define RADIO_PROFILE       0


// Optional low‑power mode: deep sleep between sensor TX
#define LOW_POWER_MODE      0   // set to 1 for deep sleep node


// ========================================================
// Heltec LoRa V2 Pin Definitions
// ========================================================
#define LORA_CS    18
#define LORA_RST   14
#define LORA_DIO0  26
#define LORA_DIO1  33
#define LORA_DIO2  32


#define I2C_SDA    4
#define I2C_SCL    15


#define LED_PIN    25
#define OLED_RST   16


// ========================================================
// Radio Parameters (per region / profile)
// ========================================================
#if REGION_PROFILE == 0   // EU868
  float LORA_FREQ_BASE = 868.1f;
#elif REGION_PROFILE == 1 // US915
  float LORA_FREQ_BASE = 915.0f;
#else                     // AS923 or others
  float LORA_FREQ_BASE = 923.0f;
#endif


float LORA_FREQ  = LORA_FREQ_BASE;
float LORA_BW    = (RADIO_PROFILE == 0) ? 125.0f : 250.0f;
int   LORA_SF    = (RADIO_PROFILE == 0) ? 9 : 7;
int   LORA_CR    = 7;
int   LORA_PWR   = 17;
int   LORA_PREAMBLE = 8;


// ========================================================
// Mesh Network Parameters
// ========================================================
#define MAX_HOP_LIMIT       8
#define MESH_VERSION        2
#define MAX_PACKET_SIZE     256
#define MESH_ID             0x12345678


// Dedup cache
#define DEDUP_CACHE_SIZE    20


// ========================================================
// Timing Parameters
// ========================================================
#define SENSOR_INTERVAL       30000UL   // 30 s
#define RX_TIMEOUT            5         // ms, used by RadioLib
#define FORWARD_DELAY_MIN     500UL
#define FORWARD_DELAY_MAX     2000UL
#define STATS_INTERVAL        300000UL  // 5 min
#define DISPLAY_REFRESH       1000UL    // 1 s
#define NEIGHBOR_INTERVAL     60000UL   // 1 min
#define STATUS_MSG_DURATION   1500UL    // 1.5 s


#if LOW_POWER_MODE
  // deep sleep interval in seconds (used on sensor nodes)
  #define SLEEP_SECONDS       60ULL
#endif


// Watchdog (updated for ESP32 IDF 5.1+)
#define WDT_TIMEOUT_SEC       15


// ========================================================
// Global Objects
// ========================================================
Adafruit_BMP280 bmp;
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);

// FIX 1: Removed U8G2_I2C_OPT_NONE parameter - use default constructor
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* U8G2_I2C_OPT_NONE */ U8X8_PIN_NONE, I2C_SCL, I2C_SDA);


// ========================================================
// Global Variables
// ========================================================
RTC_DATA_ATTR uint32_t bootCount = 0;


unsigned long lastSensorTime   = 0;
unsigned long lastStatsTime    = 0;
unsigned long lastNeighborTime = 0;
unsigned long lastDisplayUpdate= 0;
unsigned long lastCleanup      = 0;
uint32_t packetCounter = 0;
uint32_t txCount = 0;
uint32_t fwdCount = 0;
uint32_t rxCount = 0;
uint32_t crcErrorCount = 0;
uint32_t rxErrorCount = 0;
int16_t lastRSSI = -120;
int8_t  lastSNR  = 0;
float currentTemp = 0;
float currentPressure = 0;
float currentAltitude = 0;
bool ledState = false;
uint8_t displayPage = 0;
unsigned long lastPageChange = 0;


// Status banner
String statusLine1;
String statusLine2;
unsigned long statusUntil = 0;


// ========================================================
// NODE_ID from MAC
// ========================================================
uint16_t getNodeIdFromMAC() {
  uint64_t mac = ESP.getEfuseMac();
  uint16_t id = (uint16_t)(mac ^ (mac >> 16) ^ (mac >> 32) ^ (mac >> 48));
  if (id == 0) id = 1;
  return id;
}


uint16_t NODE_ID = 0;


// Display name buffer
char NODE_NAME[20];


// ========================================================
// Forwarding Queue
// ========================================================
struct ForwardQueueItem {
  uint8_t packet[MAX_PACKET_SIZE];
  size_t packetLen;
  unsigned long sendTime;
};


#define MAX_QUEUE_SIZE 5
ForwardQueueItem forwardQueue[MAX_QUEUE_SIZE];
uint8_t queueHead = 0;
uint8_t queueTail = 0;


// ========================================================
// Neighbor tracking
// ========================================================
struct Neighbor {
  uint16_t nodeId;
  int8_t lastRSSI;
  int8_t lastSNR;
  uint32_t lastSeen;
  uint32_t packetCount;
  uint8_t isActive;
};


#define MAX_NEIGHBORS 15
Neighbor neighbors[MAX_NEIGHBORS];
uint8_t numNeighbors = 0;


// ========================================================
// Display modes
// ========================================================
enum DisplayMode {
  DISPLAY_SENSOR = 0,
  DISPLAY_RADIO  = 1,
  DISPLAY_MESH   = 2,
  DISPLAY_SYSTEM = 3,
  DISPLAY_MAX    = 4
};


// ========================================================
// Packet Structures
// ========================================================
#pragma pack(push, 1)
struct SensorPacket {
  uint16_t nodeId;
  uint32_t sequenceNum;
  float temperature;
  float pressure;
  float altitude;
  uint16_t crc;
};


struct MeshPacketHeader {
  uint8_t  version;
  uint8_t  type;
  uint8_t  hopLimit;
  uint16_t srcNode;
  uint16_t destNode;
  uint32_t meshId;
  uint8_t  ttl;
};


struct MeshPacket {
  MeshPacketHeader header;
  SensorPacket sensor;
  int8_t rssi;
  int8_t snr;
};
#pragma pack(pop)


// ========================================================
// Dedup Cache
// ========================================================
struct DedupEntry {
  uint16_t srcNode;
  uint32_t seq;
};


DedupEntry dedupCache[DEDUP_CACHE_SIZE];
uint8_t dedupIndex = 0;


bool isDuplicate(uint16_t src, uint32_t seq) {
  for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
    if (dedupCache[i].srcNode == src && dedupCache[i].seq == seq) {
      return true;
    }
  }
  dedupCache[dedupIndex].srcNode = src;
  dedupCache[dedupIndex].seq = seq;
  dedupIndex = (dedupIndex + 1) % DEDUP_CACHE_SIZE;
  return false;
}


// ========================================================
// CRC16
// ========================================================
uint16_t calculateCRC16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}


// ========================================================
// LED
// ========================================================
void blinkLED(int times = 1, int duration = 100) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(duration);
  }
}


void setLED(bool state) {
  digitalWrite(LED_PIN, state);
  ledState = state;
}


// ========================================================
// Status banner (non‑blocking)
// ========================================================
void setStatus(const String& l1, const String& l2 = "") {
  statusLine1 = l1;
  statusLine2 = l2;
  statusUntil = millis() + STATUS_MSG_DURATION;
}


// ========================================================
// OLED
// ========================================================
void setupOLED() {
  Serial.println("[SETUP] OLED...");
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  delay(50);


  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);


  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x18B_tf);
  u8g2.drawStr(10, 10, "Heltec LoRa V2");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(20, 35, "Mesh Sensor Node");
  u8g2.drawStr(40, 50, "Booting...");
  u8g2.sendBuffer();
}


void drawHeader() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 0, NODE_NAME);


  // simple battery box (dummy)
  u8g2.drawFrame(100, 0, 28, 10);
  u8g2.drawBox(102, 2, 20, 6);


  char pageStr[10];
  sprintf(pageStr, "Pg%d/%d", displayPage + 1, DISPLAY_MAX);
  u8g2.drawStr(90, 12, pageStr);


  u8g2.drawHLine(0, 13, 128);
}


void displaySensorPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "SENSOR DATA");
  u8g2.drawHLine(0, 26, 128);


  char tempStr[24];
  sprintf(tempStr, "Temp: %.1f C", currentTemp);
  u8g2.drawStr(5, 30, tempStr);


  char pressStr[24];
  sprintf(pressStr, "Press: %.0f hPa", currentPressure);
  u8g2.drawStr(5, 42, pressStr);


  char altStr[24];
  sprintf(altStr, "Alt : %.0f m", currentAltitude);
  u8g2.drawStr(5, 54, altStr);
}


void displayRadioPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "RADIO STATUS");
  u8g2.drawHLine(0, 26, 128);


  char rssiStr[24];
  sprintf(rssiStr, "RSSI: %d dBm", lastRSSI);
  u8g2.drawStr(5, 30, rssiStr);


  char snrStr[24];
  sprintf(snrStr, "SNR : %d dB", lastSNR);
  u8g2.drawStr(5, 42, snrStr);


  char freqStr[24];
  sprintf(freqStr, "Freq: %.1f MHz", LORA_FREQ);
  u8g2.drawStr(5, 54, freqStr);
}


void displayMeshPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "MESH NETWORK");
  u8g2.drawHLine(0, 26, 128);


  char txStr[24];
  sprintf(txStr, "TX : %lu", (unsigned long)txCount);
  u8g2.drawStr(5, 30, txStr);


  char rxStr[24];
  sprintf(rxStr, "RX : %lu", (unsigned long)rxCount);
  u8g2.drawStr(65, 30, rxStr);


  char neighStr[24];
  sprintf(neighStr, "Neigh: %d", numNeighbors);
  u8g2.drawStr(5, 42, neighStr);


  if (numNeighbors > 0) {
    Neighbor &n = neighbors[0];
    char nbStr[24];
    sprintf(nbStr, "N0x%04X %d/%d", n.nodeId, n.lastRSSI, n.lastSNR);
    u8g2.drawStr(65, 42, nbStr);
  }


  uint8_t queueSize = (queueTail - queueHead + MAX_QUEUE_SIZE) % MAX_QUEUE_SIZE;
  char queueStr[24];
  sprintf(queueStr, "Q: %d/%d", queueSize, MAX_QUEUE_SIZE);
  u8g2.drawStr(5, 54, queueStr);
}


void displaySystemPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "SYSTEM INFO");
  u8g2.drawHLine(0, 26, 128);


  char nodeStr[24];
  sprintf(nodeStr, "ID  : 0x%04X", NODE_ID);
  u8g2.drawStr(5, 30, nodeStr);


  char heapStr[24];
  sprintf(heapStr, "Heap: %u", ESP.getFreeHeap());
  u8g2.drawStr(5, 42, heapStr);


  char cpuStr[24];
  sprintf(cpuStr, "CPU : %d MHz", ESP.getCpuFreqMHz());
  u8g2.drawStr(5, 54, cpuStr);
}


void drawStatusBanner() {
  if (millis() > statusUntil) return;
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawBox(0, 56, 128, 8);
  u8g2.setDrawColor(0);
  u8g2.drawStr(1, 56, statusLine1.c_str());
  if (statusLine2.length() > 0) {
    u8g2.drawStr(1, 62, statusLine2.c_str());
  }
  u8g2.setDrawColor(1);
}


void updateDisplay() {
  if (millis() - lastDisplayUpdate < DISPLAY_REFRESH) return;
  lastDisplayUpdate = millis();


  u8g2.clearBuffer();
  drawHeader();


  switch (displayPage) {
    case DISPLAY_SENSOR: displaySensorPage(); break;
    case DISPLAY_RADIO:  displayRadioPage();  break;
    case DISPLAY_MESH:   displayMeshPage();   break;
    case DISPLAY_SYSTEM: displaySystemPage(); break;
  }


  char timeStr[24];
  sprintf(timeStr, "Up: %lus", (unsigned long)(millis() / 1000));
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(100, 56, timeStr);


  drawStatusBanner();
  u8g2.sendBuffer();


  if (millis() - lastPageChange > 10000) {
    displayPage = (displayPage + 1) % DISPLAY_MAX;
    lastPageChange = millis();
  }
}


// ========================================================
// BMP280
// ========================================================
bool setupBMP280() {
  Serial.println("[SETUP] BMP280...");
  setStatus("Init BMP280");


  // Try 0x76 first (most common), then 0x77 (alternate)
  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("[ERROR] BMP280 not found at 0x76 or 0x77!");
      setStatus("BMP280 ERROR");
      blinkLED(3, 200);
      return false;
    }
  }


  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );


  Serial.println("[OK] BMP280 ready");
  setStatus("BMP280 OK");
  return true;
}


// ========================================================
// Radio
// ========================================================
bool setupRadio() {
  Serial.println("[SETUP] LoRa...");
  setStatus("Init LoRa");


  SPI.begin(5, 19, 27, 18);


  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_PREAMBLE, LORA_PWR);
  // FIX 2: Use RADIOLIB_ERR_NONE instead of ERR_NONE
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] Radio init: %d\n", state);
    setStatus("Radio ERROR");
    blinkLED(4, 200);
    return false;
  }


  radio.setCRC(true);
  // FIX 3: Remove setRxBoostedGainMode - not available in SX1276
  // radio.setRxBoostedGainMode(true);
  
  // FIX 4: Remove setLDROAuto - not available in SX1276
  // radio.setLDROAuto();
  
  radio.setSyncWord(0x12);
  radio.invertIQ(true);
  
  // FIX 5: setDio0Action requires 2 parameters (function pointer + direction)
  // For continuous RX, we can omit this or use:
  // radio.setDio0Action(nullptr);  // removed since signature requires 2 args


  Serial.println("[OK] LoRa configured");
  char freqStr[32];
  sprintf(freqStr, "Freq %.1f BW%.0f", LORA_FREQ, LORA_BW);
  setStatus("Radio OK", freqStr);
  return true;
}


bool sendPacket(uint8_t* data, size_t len) {
  if (len == 0) return false;


  // simple TX indicator
  setLED(true);
  int state = radio.transmit(data, len);
  setLED(false);


  // FIX 6: Use RADIOLIB_ERR_NONE
  if (state == RADIOLIB_ERR_NONE) {
    txCount++;
    return true;
  } else {
    Serial.printf("[TX ERROR] %d\n", state);
    rxErrorCount++;
    return false;
  }
}


int receivePacket(uint8_t* buffer, size_t maxLen, int16_t* rssi = NULL, int8_t* snr = NULL) {
  int state = radio.receive(buffer, maxLen, RX_TIMEOUT);
  // FIX 7: Use RADIOLIB_ERR_* constants
  if (state == RADIOLIB_ERR_NONE) {
    size_t len = radio.getPacketLength();
    if (rssi) *rssi = radio.getRSSI();
    if (snr) *snr = radio.getSNR();
    rxCount++;
    return len;
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    return 0;
  } else {
    rxErrorCount++;
    return -1;
  }
}


// ========================================================
// Neighbor management
// ========================================================
void updateNeighbor(uint16_t nodeId, int8_t rssi, int8_t snr) {
  for (int i = 0; i < numNeighbors; i++) {
    if (neighbors[i].nodeId == nodeId) {
      neighbors[i].lastRSSI = rssi;
      neighbors[i].lastSNR = snr;
      neighbors[i].lastSeen = millis();
      neighbors[i].packetCount++;
      neighbors[i].isActive = 1;
      return;
    }
  }


  if (numNeighbors < MAX_NEIGHBORS) {
    neighbors[numNeighbors].nodeId = nodeId;
    neighbors[numNeighbors].lastRSSI = rssi;
    neighbors[numNeighbors].lastSNR = snr;
    neighbors[numNeighbors].lastSeen = millis();
    neighbors[numNeighbors].packetCount = 1;
    neighbors[numNeighbors].isActive = 1;
    numNeighbors++;
    String s = "New neighbor 0x" + String(nodeId, HEX);
    setStatus(s);
  }
}


void cleanupNeighbors() {
  uint32_t now = millis();
  uint32_t timeout = 300000UL;
  for (int i = 0; i < numNeighbors; i++) {
    if (now - neighbors[i].lastSeen > timeout) {
      neighbors[i].isActive = 0;
    }
  }
}


// ========================================================
// Forward queue
// ========================================================
bool addToForwardQueue(uint8_t* pkt, size_t len) {
  if (!ROLE_ROUTER) return false;
  if ((queueTail + 1) % MAX_QUEUE_SIZE == queueHead) {
    return false;
  }
  memcpy(forwardQueue[queueTail].packet, pkt, len);
  forwardQueue[queueTail].packetLen = len;
  forwardQueue[queueTail].sendTime = millis() + random(FORWARD_DELAY_MIN, FORWARD_DELAY_MAX);
  queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
  return true;
}


void processForwardQueue() {
  if (!ROLE_ROUTER) return;
  if (queueHead == queueTail) return;
  uint32_t now = millis();
  while (queueHead != queueTail) {
    if (now >= forwardQueue[queueHead].sendTime) {
      MeshPacket* mp = (MeshPacket*)forwardQueue[queueHead].packet;
      if (mp->header.ttl > 1) {
        mp->header.ttl--;
        sendPacket(forwardQueue[queueHead].packet, forwardQueue[queueHead].packetLen);
        fwdCount++;
      }
      queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
    } else {
      break;
    }
  }
}


// ========================================================
// Sensor / TX
// ========================================================
void readAndTransmitSensorData() {
  if (!ROLE_SENSOR) return;
  if (!LOW_POWER_MODE && millis() - lastSensorTime < SENSOR_INTERVAL) return;


#if !LOW_POWER_MODE
  lastSensorTime = millis();
#endif
  packetCounter++;


  currentTemp = bmp.readTemperature();
  currentPressure = bmp.readPressure() / 100.0f;
  currentAltitude = bmp.readAltitude(1013.25);


  if (isnan(currentTemp) || isnan(currentPressure)) {
    Serial.println("[SENSOR] Read fail");
    setStatus("Sensor ERROR");
    return;
  }


  SensorPacket sp;
  sp.nodeId = NODE_ID;
  sp.sequenceNum = packetCounter;
  sp.temperature = currentTemp;
  sp.pressure = currentPressure;
  sp.altitude = currentAltitude;
  sp.crc = calculateCRC16((const uint8_t*)&sp, sizeof(SensorPacket) - 2);


  MeshPacket mp;
  mp.header.version  = MESH_VERSION;
  mp.header.type     = 0x01;
  mp.header.hopLimit = MAX_HOP_LIMIT;
  mp.header.srcNode  = NODE_ID;
  mp.header.destNode = ROLE_SINK ? 0x0000 : 0xFFFF;
  mp.header.meshId   = MESH_ID;
  mp.header.ttl      = MAX_HOP_LIMIT;
  mp.sensor          = sp;
  mp.rssi            = 0;
  mp.snr             = 0;


  setStatus("TX Sensor");
  if (sendPacket((uint8_t*)&mp, sizeof(MeshPacket))) {
    blinkLED(1, 40);
    Serial.printf("[TX #%lu] T=%.1f P=%.0f\n",
                  (unsigned long)packetCounter,
                  currentTemp, currentPressure);
  } else {
    setStatus("TX Failed");
  }
}


// ========================================================
// RX processing
// ========================================================
void processReceivedPacket(uint8_t* data, size_t len, int16_t rssi, int8_t snr) {
  if (len != sizeof(MeshPacket)) return;
  MeshPacket* mp = (MeshPacket*)data;


  if (mp->header.version != MESH_VERSION) return;
  if (mp->header.meshId != MESH_ID) return;


  uint16_t crcExp = calculateCRC16((const uint8_t*)&mp->sensor,
                                   sizeof(SensorPacket) - 2);
  if (mp->sensor.crc != crcExp) {
    crcErrorCount++;
    return;
  }


  if (isDuplicate(mp->sensor.nodeId, mp->sensor.sequenceNum)) {
    return;
  }


  lastRSSI = rssi;
  lastSNR  = snr;
  updateNeighbor(mp->header.srcNode, rssi, snr);


  String s = "RX from 0x" + String(mp->header.srcNode, HEX);
  setStatus(s);


  // If sink, could add handling here (e.g., serial output, etc.)


  if (ROLE_ROUTER && mp->header.srcNode != NODE_ID && mp->header.ttl > 1) {
    addToForwardQueue(data, len);
  }
  blinkLED(2, 30);
}


// ========================================================
// Button for display
// ========================================================
void handleButtonPress() {
  static unsigned long lastPress = 0;
  if (digitalRead(0) == LOW) {
    if (millis() - lastPress > 300) {
      displayPage = (displayPage + 1) % DISPLAY_MAX;
      lastPageChange = millis();
      lastPress = millis();
      blinkLED(1, 30);
    }
  }
}


// ========================================================
// Watchdog (ESP32 IDF 5.1+ compatible)
// ========================================================
void setupWatchdog() {
  // FIX 8: Updated for ESP32 IDF 5.1+ config struct API
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
}


void feedWatchdog() {
  esp_task_wdt_reset();
}


// ========================================================
// Setup
// ========================================================
void setup() {
  Serial.begin(115200);
  delay(500);


  ++bootCount;
  NODE_ID = getNodeIdFromMAC();
  sprintf(NODE_NAME, "Node_0x%04X", NODE_ID);


  Serial.println("\n===================================");
  Serial.println("  Heltec LoRa V2 Mesh Node");
  Serial.print("  Boot #"); Serial.println(bootCount);
  Serial.print("  NODE_ID: 0x"); Serial.println(NODE_ID, HEX);
  Serial.println("===================================");


  pinMode(LED_PIN, OUTPUT);
  setLED(false);


  for (int i = 0; i < 4; i++) {
    setLED(true);
    delay(50);
    setLED(false);
    delay(50);
  }


  setupWatchdog();


  setupOLED();


  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);


  if (ROLE_SENSOR && !setupBMP280()) {
    while (1) {
      blinkLED(5, 100);
      delay(500);
    }
  }


  if (!setupRadio()) {
    while (1) {
      blinkLED(5, 200);
      delay(1000);
    }
  }


  radio.startReceive();


  pinMode(0, INPUT_PULLUP);


  randomSeed(ESP.getEfuseMac() ^ micros());


  setStatus("System Ready");
  Serial.println("[OK] Init complete");
}


// ========================================================
// Loop
// ========================================================
void loop() {
  feedWatchdog();


  handleButtonPress();


#if LOW_POWER_MODE
  // simple pattern: read, TX, then deep sleep
  readAndTransmitSensorData();


  // short delay to finish any pending serial
  delay(100);
  Serial.println("Going to deep sleep...");
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
#else
  readAndTransmitSensorData();


  uint8_t rxBuffer[MAX_PACKET_SIZE];
  int16_t rssi;
  int8_t snr;


  int len = receivePacket(rxBuffer, MAX_PACKET_SIZE, &rssi, &snr);
  if (len > 0) {
    processReceivedPacket(rxBuffer, len, rssi, snr);
    radio.startReceive();
  }


  processForwardQueue();
  updateDisplay();


  if (millis() - lastCleanup > NEIGHBOR_INTERVAL) {
    lastCleanup = millis();
    cleanupNeighbors();
  }


  if (millis() - lastStatsTime > STATS_INTERVAL) {
    lastStatsTime = millis();
    Serial.println("\n=== STATS ===");
    Serial.printf("Uptime : %lu s\n", (unsigned long)(millis() / 1000));
    Serial.printf("TX/RX/FWD: %lu/%lu/%lu\n",
                  (unsigned long)txCount,
                  (unsigned long)rxCount,
                  (unsigned long)fwdCount);
    Serial.printf("CRC Err: %lu, RX Err: %lu\n",
                  (unsigned long)crcErrorCount,
                  (unsigned long)rxErrorCount);
    Serial.printf("Heap   : %u\n", ESP.getFreeHeap());
  }


  delay(1);
#endif
}
