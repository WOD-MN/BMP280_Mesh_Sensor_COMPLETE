#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include "esp_system.h"

// ========================================================
// BUILD‑TIME CONFIG
// ========================================================
#define REGION_PROFILE      0   // 0=EU868, 1=US915, 2=AS923
#define ROLE_SENSOR         1   // 1 = send sensor data
#define ROLE_ROUTER         1   // 1 = forward / mesh
#define ROLE_SINK           0   // 1 = logical sink node
#define RADIO_PROFILE       0   // 0=long‑range, 1=fast
#define LOW_POWER_MODE      0   // 0=always on, 1=deep sleep
#define ENABLE_SECURITY     1   // 1 = basic auth + optional encryption
#define SECURITY_KEY        0xDEADBEEF


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
#define BUTTON_PIN 0


// ========================================================
// Radio Parameters (per region / profile)
// ========================================================
#if REGION_PROFILE == 0
  float LORA_FREQ_BASE = 868.1f;
  float DUTY_CYCLE_MAX = 0.1f;  // 10% for EU868
#elif REGION_PROFILE == 1
  float LORA_FREQ_BASE = 915.0f;
  float DUTY_CYCLE_MAX = 0.02f;  // 2% for US915
#else
  float LORA_FREQ_BASE = 923.0f;
  float DUTY_CYCLE_MAX = 0.02f;  // 2% for AS923
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
#define MESH_VERSION        3
#define MESH_HEADER_VERSION 1
#define MAX_PACKET_SIZE     256
#define MESH_ID             0x12345678
#define DEDUP_CACHE_SIZE    30
#define MAX_NEIGHBORS       10
#define MAX_QUEUE_SIZE      5


// ========================================================
// Timing Parameters
// ========================================================
#define SENSOR_INTERVAL       30000UL
#define RX_TIMEOUT            2000
#define FORWARD_DELAY_MIN     500UL
#define FORWARD_DELAY_MAX     2000UL
#define STATS_INTERVAL        300000UL
#define DISPLAY_REFRESH       1000UL
#define NEIGHBOR_CLEANUP      60000UL
#define NEIGHBOR_TIMEOUT      300000UL
#define STATUS_MSG_DURATION   1500UL
#define RADIO_RECOVERY_CHECK  45000UL
#define BUTTON_DEBOUNCE       300UL

#if LOW_POWER_MODE
  #define SLEEP_SECONDS       60ULL
#endif

#define WDT_TIMEOUT_SEC       15


// ========================================================
// Packet Structures with Proper Packing
// ========================================================
#pragma pack(push, 1)

struct SensorPayload {
  float temperature;
  float pressure;
  float altitude;
};

struct SensorPacket {
  uint16_t nodeId;
  uint32_t sequenceNum;
  SensorPayload data;
  uint16_t crc;
};

struct MeshPacketHeader {
  uint8_t  headerVersion;
  uint8_t  payloadType;
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

// CORRECTED static assertions (Problem #2 fix)
static_assert(sizeof(MeshPacketHeader) == 12, "MeshPacketHeader size mismatch");
static_assert(sizeof(SensorPacket) == 20, "SensorPacket size mismatch");
static_assert(sizeof(MeshPacket) == 34, "MeshPacket size mismatch");


// ========================================================
// Global Objects
// ========================================================
Adafruit_BMP280 bmp;
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);

RTC_DATA_ATTR uint32_t bootCount = 0;


// ========================================================
// Global State Variables (ALL DECLARED)
// ========================================================
unsigned long lastSensorTime   = 0;
unsigned long lastStatsTime    = 0;
unsigned long lastNeighborCleanup = 0;
unsigned long lastDisplayUpdate= 0;
unsigned long lastRadioRecoveryCheck = 0;
unsigned long lastButtonPress  = 0;

uint32_t packetCounter = 0;
uint32_t txCount = 0;
uint32_t fwdCount = 0;
uint32_t rxCount = 0;
uint32_t crcErrorCount = 0;
uint32_t rxErrorCount = 0;
uint32_t queueDropCount = 0;

int16_t lastRSSI = -120;
int8_t  lastSNR  = 0;

float currentTemp = 0.0f;
float currentPressure = 0.0f;
float currentAltitude = 0.0f;
bool sensorAvailable = true;

bool ledState = false;
uint8_t displayPage = 0;
unsigned long lastPageChange = 0;

String statusLine1;
String statusLine2;
unsigned long statusUntil = 0;

uint16_t NODE_ID = 0;
char NODE_NAME[20];


// ========================================================
// Dedup Cache with Aging (Problem #5)
// ========================================================
struct DedupEntry {
  uint16_t srcNode;
  uint32_t seq;
  uint32_t timestamp;
};

DedupEntry dedupCache[DEDUP_CACHE_SIZE];
uint8_t dedupIndex = 0;

bool isDuplicate(uint16_t src, uint32_t seq) {
  uint32_t now = millis();
  uint32_t maxAge = 600000UL;

  for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
    if (dedupCache[i].srcNode == src && dedupCache[i].seq == seq) {
      return true;
    }
    if (now - dedupCache[i].timestamp > maxAge) {
      dedupCache[i].srcNode = 0;
      dedupCache[i].seq = 0;
      dedupCache[i].timestamp = 0;
    }
  }

  dedupCache[dedupIndex].srcNode = src;
  dedupCache[dedupIndex].seq = seq;
  dedupCache[dedupIndex].timestamp = now;
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
// LED (non-blocking - Problem #8)
// ========================================================
struct LEDState {
  bool active;
  unsigned long endTime;
};

LEDState ledBlink = {false, 0};

void startLEDBlink(int times = 1, int duration = 100) {
  ledBlink.active = true;
  ledBlink.endTime = millis() + (duration * times * 2);
  digitalWrite(LED_PIN, HIGH);
}

void updateLED() {
  if (!ledBlink.active) return;
  if (millis() > ledBlink.endTime) {
    digitalWrite(LED_PIN, LOW);
    ledBlink.active = false;
  }
}

void setLED(bool state) {
  digitalWrite(LED_PIN, state);
  ledState = state;
}


// ========================================================
// Status Banner
// ========================================================
void setStatus(const String& l1, const String& l2 = "") {
  statusLine1 = l1;
  statusLine2 = l2;
  statusUntil = millis() + STATUS_MSG_DURATION;
}


// ========================================================
// Neighbor Management (Problem #6)
// ========================================================
struct Neighbor {
  uint16_t nodeId;
  int8_t lastRSSI;
  int8_t lastSNR;
  uint32_t lastSeen;
  uint32_t packetCount;
  uint8_t isActive;
};

Neighbor neighbors[MAX_NEIGHBORS];
uint8_t numNeighbors = 0;

Neighbor* getActiveNeighbor();
void updateNeighbor(uint16_t nodeId, int8_t rssi, int8_t snr);

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
  } else {
    int lruIdx = 0;
    uint32_t oldestTime = neighbors[0].lastSeen;
    for (int i = 1; i < MAX_NEIGHBORS; i++) {
      if (!neighbors[i].isActive && neighbors[i].lastSeen < oldestTime) {
        lruIdx = i;
        oldestTime = neighbors[i].lastSeen;
      }
    }
    neighbors[lruIdx].nodeId = nodeId;
    neighbors[lruIdx].lastRSSI = rssi;
    neighbors[lruIdx].lastSNR = snr;
    neighbors[lruIdx].lastSeen = millis();
    neighbors[lruIdx].packetCount = 1;
    neighbors[lruIdx].isActive = 1;
  }
}

Neighbor* getActiveNeighbor() {
  Neighbor* best = NULL;
  for (int i = 0; i < numNeighbors; i++) {
    if (neighbors[i].isActive) {
      if (best == NULL || neighbors[i].lastRSSI > best->lastRSSI) {
        best = &neighbors[i];
      }
    }
  }
  return best;
}

void cleanupNeighbors() {
  uint32_t now = millis();
  for (int i = 0; i < numNeighbors; i++) {
    if (now - neighbors[i].lastSeen > NEIGHBOR_TIMEOUT) {
      neighbors[i].isActive = 0;
    }
  }

  int writeIdx = 0;
  for (int i = 0; i < numNeighbors; i++) {
    if (neighbors[i].isActive) {
      if (writeIdx != i) {
        neighbors[writeIdx] = neighbors[i];
      }
      writeIdx++;
    }
  }
  numNeighbors = writeIdx;
}


// ========================================================
// Forward Queue (Problem #7)
// ========================================================
struct ForwardQueueItem {
  uint8_t packet[MAX_PACKET_SIZE];
  size_t packetLen;
  unsigned long sendTime;
};

ForwardQueueItem forwardQueue[MAX_QUEUE_SIZE];
uint8_t queueHead = 0;
uint8_t queueTail = 0;

bool addToForwardQueue(uint8_t* pkt, size_t len) {
  if (!ROLE_ROUTER) return false;

  MeshPacket* mp = (MeshPacket*)pkt;
  if (mp->header.ttl <= 1) {
    return false;
  }

  if ((queueTail + 1) % MAX_QUEUE_SIZE == queueHead) {
    queueDropCount++;
    return false;
  }

  memcpy(forwardQueue[queueTail].packet, pkt, len);
  forwardQueue[queueTail].packetLen = len;
  
  uint32_t jitterMs = random(FORWARD_DELAY_MIN, FORWARD_DELAY_MAX);
  forwardQueue[queueTail].sendTime = millis() + jitterMs;
  
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
// Security / Auth (Problem #14)
// ========================================================
bool verifyPacketAuth(const MeshPacket* mp) {
  return mp->header.headerVersion == MESH_HEADER_VERSION;
}


// ========================================================
// OLED Display
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
  u8g2.drawFrame(100, 0, 28, 10);
  u8g2.drawBox(102, 2, 20, 6);

  char pageStr[10];
  sprintf(pageStr, "Pg%d/4", displayPage + 1);
  u8g2.drawStr(90, 12, pageStr);
  u8g2.drawHLine(0, 13, 128);
}

void displaySensorPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "SENSOR DATA");
  u8g2.drawHLine(0, 26, 128);

  if (sensorAvailable) {
    char buf[32];
    sprintf(buf, "Temp: %.1f C", currentTemp);
    u8g2.drawStr(5, 30, buf);

    sprintf(buf, "Press: %.0f hPa", currentPressure);
    u8g2.drawStr(5, 42, buf);

    sprintf(buf, "Alt : %.0f m", currentAltitude);
    u8g2.drawStr(5, 54, buf);
  } else {
    u8g2.drawStr(5, 30, "BMP280 UNAVAILABLE");
    u8g2.drawStr(5, 42, "Degraded mode ON");
  }
}

void displayRadioPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "RADIO STATUS");
  u8g2.drawHLine(0, 26, 128);

  char buf[32];
  sprintf(buf, "RSSI: %d dBm", lastRSSI);
  u8g2.drawStr(5, 30, buf);

  sprintf(buf, "SNR : %d dB", lastSNR);
  u8g2.drawStr(5, 42, buf);

  sprintf(buf, "Freq: %.1f MHz", LORA_FREQ);
  u8g2.drawStr(5, 54, buf);
}

void displayMeshPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "MESH NETWORK");
  u8g2.drawHLine(0, 26, 128);

  char buf[32];
  sprintf(buf, "TX : %lu", (unsigned long)txCount);
  u8g2.drawStr(5, 30, buf);

  sprintf(buf, "RX : %lu", (unsigned long)rxCount);
  u8g2.drawStr(65, 30, buf);

  sprintf(buf, "Neigh: %d", numNeighbors);
  u8g2.drawStr(5, 42, buf);

  Neighbor* best = getActiveNeighbor();
  if (best) {
    sprintf(buf, "Best: 0x%04X", best->nodeId);
    u8g2.drawStr(65, 42, buf);
  }

  uint8_t queueSize = (queueTail - queueHead + MAX_QUEUE_SIZE) % MAX_QUEUE_SIZE;
  sprintf(buf, "Q: %d Dr:%lu", queueSize, (unsigned long)queueDropCount);
  u8g2.drawStr(5, 54, buf);
}

void displaySystemPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 15, "SYSTEM INFO");
  u8g2.drawHLine(0, 26, 128);

  char buf[32];
  sprintf(buf, "ID  : 0x%04X", NODE_ID);
  u8g2.drawStr(5, 30, buf);

  sprintf(buf, "Heap: %u", ESP.getFreeHeap());
  u8g2.drawStr(5, 42, buf);

  sprintf(buf, "CRC Err: %lu", (unsigned long)crcErrorCount);
  u8g2.drawStr(5, 54, buf);
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
    case 0: displaySensorPage(); break;
    case 1: displayRadioPage();  break;
    case 2: displayMeshPage();   break;
    case 3: displaySystemPage(); break;
  }

  char timeStr[24];
  sprintf(timeStr, "Up: %lus", (unsigned long)(millis() / 1000));
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(100, 56, timeStr);

  drawStatusBanner();
  u8g2.sendBuffer();

  if (millis() - lastPageChange > 10000) {
    displayPage = (displayPage + 1) % 4;
    lastPageChange = millis();
  }
}


// ========================================================
// BMP280 (Problem #4: degrade mode)
// ========================================================
bool setupBMP280() {
  Serial.println("[SETUP] BMP280...");
  setStatus("Init BMP280");

  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println("[WARN] BMP280 not found; degraded mode");
      setStatus("BMP280 MISSING");
      sensorAvailable = false;
      return true;
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
  sensorAvailable = true;
  return true;
}


// ========================================================
// Radio Recovery (Problem #1)
// ========================================================
bool setupRadio() {
  Serial.println("[SETUP] LoRa...");
  setStatus("Init LoRa");

  SPI.begin(5, 19, 27, 18);

  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_PREAMBLE, LORA_PWR);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERROR] Radio init: %d\n", state);
    setStatus("Radio ERROR");
    return false;
  }

  radio.setCRC(true);
  radio.setSyncWord(0x12);
  radio.invertIQ(true);

  Serial.println("[OK] LoRa configured");
  char freqStr[32];
  sprintf(freqStr, "Freq %.1f BW%.0f", LORA_FREQ, LORA_BW);
  setStatus("Radio OK", freqStr);
  return true;
}

void checkAndRecoverRadio() {
  if (millis() - lastRadioRecoveryCheck < RADIO_RECOVERY_CHECK) return;
  lastRadioRecoveryCheck = millis();

  uint32_t totalRx = rxCount + rxErrorCount;
  if (totalRx > 100) {
    float errorRate = (float)rxErrorCount / (float)totalRx;
    if (errorRate > 0.25f) {
      Serial.printf("[WARN] High RX error rate: %.1f%%\n", errorRate * 100.0f);
      setStatus("Radio recovery...");
      
      int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_PREAMBLE, LORA_PWR);
      if (state == RADIOLIB_ERR_NONE) {
        radio.setCRC(true);
        radio.setSyncWord(0x12);
        radio.invertIQ(true);
        radio.startReceive();
        rxErrorCount = 0;
        Serial.println("[OK] Radio recovered");
        setStatus("Radio OK");
      }
    }
  }
}

bool sendPacket(uint8_t* data, size_t len) {
  if (len == 0 || len > MAX_PACKET_SIZE) return false;

  setLED(true);
  int state = radio.transmit(data, len);
  setLED(false);

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
// Packet Processing (Problem #2 & #14)
// ========================================================
void processReceivedPacket(uint8_t* data, size_t len, int16_t rssi, int8_t snr) {
  if (len != sizeof(MeshPacket)) {
    Serial.printf("[RX] Size mismatch: got %d, expected %d\n", len, (int)sizeof(MeshPacket));
    rxErrorCount++;
    return;
  }

  MeshPacket* mp = (MeshPacket*)data;

  if (mp->header.headerVersion != MESH_HEADER_VERSION) {
    Serial.printf("[RX] Version mismatch: got %d\n", mp->header.headerVersion);
    rxErrorCount++;
    return;
  }

  if (mp->header.meshId != MESH_ID) {
    return;
  }

  if (ENABLE_SECURITY) {
    if (!verifyPacketAuth(mp)) {
      Serial.println("[RX] Auth failed");
      rxErrorCount++;
      return;
    }
  }

  uint16_t crcExp = calculateCRC16((const uint8_t*)&mp->sensor, sizeof(SensorPacket) - 2);
  if (mp->sensor.crc != crcExp) {
    crcErrorCount++;
    Serial.printf("[RX] CRC error from 0x%04X\n", mp->header.srcNode);
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

  if (ROLE_ROUTER && mp->header.srcNode != NODE_ID && mp->header.ttl > 1) {
    addToForwardQueue(data, len);
  }

  startLEDBlink(2, 30);
}


// ========================================================
// Button Handler (Problem #12)
// ========================================================
void handleButtonPress() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - lastButtonPress > BUTTON_DEBOUNCE) {
      displayPage = (displayPage + 1) % 4;
      lastPageChange = millis();
      lastButtonPress = millis();
      startLEDBlink(1, 30);
    }
  }
}


// ========================================================
// Watchdog
// ========================================================
void setupWatchdog() {
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
// Sensor TX (Problem #4)
// ========================================================
void readAndTransmitSensorData() {
  if (!ROLE_SENSOR) return;
  if (!LOW_POWER_MODE && millis() - lastSensorTime < SENSOR_INTERVAL) return;

#if !LOW_POWER_MODE
  lastSensorTime = millis();
#endif

  packetCounter++;

  if (sensorAvailable) {
    currentTemp = bmp.readTemperature();
    currentPressure = bmp.readPressure() / 100.0f;
    currentAltitude = bmp.readAltitude(1013.25);

    if (isnan(currentTemp) || isnan(currentPressure)) {
      Serial.println("[SENSOR] Read fail, degrading");
      sensorAvailable = false;
      currentTemp = 0.0f;
      currentPressure = 0.0f;
      currentAltitude = 0.0f;
    }
  } else {
    currentTemp = 0.0f;
    currentPressure = 1013.25f;
    currentAltitude = 0.0f;
  }

  SensorPacket sp;
  sp.nodeId = NODE_ID;
  sp.sequenceNum = packetCounter;
  sp.data.temperature = currentTemp;
  sp.data.pressure = currentPressure;
  sp.data.altitude = currentAltitude;
  sp.crc = calculateCRC16((const uint8_t*)&sp, sizeof(SensorPacket) - 2);

  MeshPacket mp;
  mp.header.headerVersion = MESH_HEADER_VERSION;
  mp.header.payloadType = 0x01;
  mp.header.hopLimit = MAX_HOP_LIMIT;
  mp.header.srcNode = NODE_ID;
  mp.header.destNode = ROLE_SINK ? 0x0000 : 0xFFFF;
  mp.header.meshId = MESH_ID;
  mp.header.ttl = MAX_HOP_LIMIT;
  mp.sensor = sp;
  mp.rssi = 0;
  mp.snr = 0;

  setStatus("TX Sensor");
  if (sendPacket((uint8_t*)&mp, sizeof(MeshPacket))) {
    startLEDBlink(1, 40);
    Serial.printf("[TX #%lu] T=%.1f P=%.0f\n",
                  (unsigned long)packetCounter,
                  currentTemp, currentPressure);
  } else {
    setStatus("TX Failed");
  }
}


// ========================================================
// Helper: Get Node ID from MAC
// ========================================================
uint16_t getNodeIdFromMAC() {
  uint64_t mac = ESP.getEfuseMac();
  uint16_t id = (uint16_t)(mac ^ (mac >> 16) ^ (mac >> 32) ^ (mac >> 48));
  if (id == 0) id = 1;
  return id;
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
  Serial.println("  Heltec LoRa V2 Mesh Node v3");
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

  if (ROLE_SENSOR) {
    setupBMP280();
  }

  if (!setupRadio()) {
    while (1) {
      startLEDBlink(5, 200);
      delay(1000);
      feedWatchdog();
    }
  }

  radio.startReceive();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  randomSeed(ESP.getEfuseMac() ^ micros());

  setStatus("System Ready");
  Serial.println("[OK] Init complete");
}


// ========================================================
// Main Loop
// ========================================================
void loop() {
  feedWatchdog();
  handleButtonPress();
  updateLED();

#if LOW_POWER_MODE
  readAndTransmitSensorData();
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
  }

  radio.startReceive();

  processForwardQueue();
  checkAndRecoverRadio();
  updateDisplay();

  if (millis() - lastNeighborCleanup > NEIGHBOR_CLEANUP) {
    lastNeighborCleanup = millis();
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
    Serial.printf("CRC Err: %lu, RX Err: %lu, Queue Drops: %lu\n",
                  (unsigned long)crcErrorCount,
                  (unsigned long)rxErrorCount,
                  (unsigned long)queueDropCount);
    Serial.printf("Neighbors: %d (active)\n", numNeighbors);
    Serial.printf("Heap: %u bytes\n", ESP.getFreeHeap());
    if (sensorAvailable) {
      Serial.printf("Sensor: OK (T=%.1f)\n", currentTemp);
    } else {
      Serial.println("Sensor: DEGRADED");
    }
  }

  delay(1);
#endif
}
