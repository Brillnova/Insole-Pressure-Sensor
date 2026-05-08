/*
  ESP32-C3 Insole Logger v3 — Master/Slave + OTA + ESP-NOW + Time Sync
  --------------------------------------------------------------------
  - Single CD74HC4067 (16ch) per device (RIGHT on MASTER, LEFT on SLAVE)
  - Build-time ROLE selection (MASTER or SLAVE)
  - Wireless: ESP-NOW (fast/low-overhead)
  - MASTER:
      * Generates frame_id at LOG_HZ, reads its own 16ch, sends TICK
      * Collects SLAVE 16ch by matching frame_id
      * Logs combined CSV: time_ms,frame_id,R_ch0..15,L_ch0..15,flags
  - SLAVE:
      * On TICK, reads 16ch and sends DATA(frame_id,...)
  - OTA (optional): ArduinoOTA over Wi-Fi STA
  - Time sync:
      * Periodic 2-way SYNC (NTP-like) to estimate offset/drift (printed)
      * Frame-based sampling uses TICK → aligned even without offset correction

  IMPORTANT:
  - Fill PEER MAC below (the other device's Wi-Fi MAC)
  - For OTA, set WIFI_SSID/PASS. If OTA is enabled, both devices should be
    connected to the same AP (same channel). Otherwise, set ESPNOW_CHANNEL
    to a fixed channel on both devices.
  - SD card: format as FAT32. Use short wiring and 8~20 MHz SPI.

  Comments are in English (per user preference).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <SD.h>
#include <esp_wifi.h>

// =================== Build-time configuration ===================
#define FW_VERSION     "v3.1.0"

// ---- Select ROLE and SIDE ----
// Exactly one ROLE must be defined:
// #define ROLE_MASTER
#define ROLE_SLAVE

// Side of THIS device (used for naming & mapping)
// #define SIDE_RIGHT
#define SIDE_LEFT

// ---- OTA settings ----
#define OTA_ENABLE     1                 // 1=enable OTA, 0=disable
#define WIFI_SSID      "wifi name"
#define WIFI_PASS      "wifi pw"
#define OTA_HOSTNAME   "insole-c3"

// ---- ESP-NOW settings ----
#define ESPNOW_CHANNEL 0                 // 1..14; use 0 to follow STA AP channel
// Fill with the OTHER device's MAC address (use Serial.println(WiFi.macAddress()))
// Example: {0x24,0x6F,0x28,0xAA,0xBB,0xCC}
static uint8_t PEER_ADDR[6] = { 0x8C,0xD0,0xB2,0xA9,0x01,0x92 };  // MASTER MAC : 0x8C,0xD0,0xB2,0xA9,0x01,0x92 | SLAVE MAC : 0x8C,0xD0,0xB2,0xA8,0xFB,0x6E

// ---- Sampling / Logging ----
#define LOG_HZ               20         // frame rate [Hz]
#define BAUD                 115200
#define PRINT_HEADER_SERIAL  1
#define ADC_BITS             12
#define ADC_SETTLE_US        120
#define ADC_DUMMY_WAIT_US    80
#define ADC_AVG_SAMPLES      4

// SD file naming
#if defined(ROLE_MASTER)
  #define LOG_PREFIX "COMBO_LOG"
#else
  #if defined(SIDE_RIGHT)
    #define LOG_PREFIX "R_LOG"
  #else
    #define LOG_PREFIX "L_LOG"
  #endif
#endif

// =================== Includes ===================
#if OTA_ENABLE
  #include <ArduinoOTA.h>
#endif

// =================== Hardware mapping ===================
// CD74HC4067 address lines + EN (active-LOW)
#define PIN_S0      1
#define PIN_S1      2
#define PIN_S2      3
#define PIN_S3      10
#define PIN_EN      8     // tie to GND and set USE_EN=0 if you prefer always-on
#define USE_EN      0

// ADC pin (ESP32-C3 ADC1 pin; adjust to your board)
#define ADC_PIN     0     // e.g., GPIO1 (A1)

// microSD SPI pins
#define SD_SCK      4
#define SD_MISO     5
#define SD_MOSI     6
#define SD_CS       7

// =================== MUX helpers ===================
static inline void setAddress(uint8_t ch){
  digitalWrite(PIN_S0, (ch >> 0) & 1);
  digitalWrite(PIN_S1, (ch >> 1) & 1);
  digitalWrite(PIN_S2, (ch >> 2) & 1);
  digitalWrite(PIN_S3, (ch >> 3) & 1);
}
static inline void muxEnable(bool on){
#if USE_EN
  digitalWrite(PIN_EN, on ? LOW : HIGH); // active-LOW
#else
  (void)on;
#endif
}
static inline int readStableADC(){
  delayMicroseconds(ADC_DUMMY_WAIT_US);
  (void)analogRead(ADC_PIN);
  delayMicroseconds(ADC_DUMMY_WAIT_US);
  int acc = 0;
  for(int i=0;i<ADC_AVG_SAMPLES;i++){
    acc += analogRead(ADC_PIN);
    delayMicroseconds(40);
  }
  return acc / ADC_AVG_SAMPLES;
}
static inline int readChannel(uint8_t ch){
  muxEnable(false);
  setAddress(ch);
  delayMicroseconds(20);
  muxEnable(true);
  delayMicroseconds(ADC_SETTLE_US);
  return readStableADC();
}

// Optional per-side mapping (logical 0..15 -> physical MUX ch)
#if defined(SIDE_RIGHT)
static const int8_t mux_map[16] = { 0,1,2,3, 4,5,6,7, 8,9,10,11, 12,13,14,15 };
#else
static const int8_t mux_map[16] = { 0,1,2,3, 4,5,6,7, 8,9,10,11, 12,13,14,15 };
#endif

// =================== ESP-NOW message types ===================
enum MsgType : uint16_t {
  MSG_TICK      = 0x1000, // master -> slave (frame trigger)
  MSG_DATA      = 0x1001, // slave -> master (sample payload)
  MSG_SYNC_REQ  = 0x1002, // master -> slave
  MSG_SYNC_RESP = 0x1003  // slave -> master
};

struct __attribute__((packed)) TickMsg {
  uint16_t type;        // MSG_TICK
  uint32_t frame_id;    // global frame id
  uint32_t fire_ms;     // optional: scheduled time at master (ms)
};

struct __attribute__((packed)) DataMsg {
  uint16_t type;        // MSG_DATA
  uint32_t frame_id;
  uint16_t seq;         // slave local sequence
  uint32_t t_local_ms;  // slave millis at sampling
  uint8_t  side;        // 0=LEFT, 1=RIGHT
  uint16_t ch[16];      // 16 channels
};

struct __attribute__((packed)) SyncReq {
  uint16_t type;        // MSG_SYNC_REQ
  uint32_t t0_ms;       // master send time
};
struct __attribute__((packed)) SyncResp {
  uint16_t type;        // MSG_SYNC_RESP
  uint32_t t0_ms;       // echoed
  uint32_t t1_ms;       // slave recv time
  uint32_t t2_ms;       // slave send time
};

// =================== Globals ===================
static const uint32_t FRAME_DT_US = 1000000UL / LOG_HZ;

File logFile;
uint32_t sample_interval_us = FRAME_DT_US;
volatile bool espnow_ready = false;
uint16_t slave_seq = 0;

#if defined(ROLE_MASTER)
// Ring buffer for pending frames
struct FrameEntry {
  uint32_t frame_id;
  bool haveR;
  bool haveL;
  uint32_t t_ms;   // master millis at frame
  uint16_t R[16];
  uint16_t L[16];
  uint8_t flags;   // bit0: L missing, bit1: timeout
};
#define FRAME_BUF_SZ 16
FrameEntry fbuf[FRAME_BUF_SZ];
uint32_t cur_frame_id = 0;
uint32_t last_sync_ms = 0;
int32_t  est_offset_ms = 0; // estimated slave-master offset (for info)

#else // ROLE_SLAVE
// nothing extra
#endif

// =================== SD helpers ===================
String nextLogFilename(const char* prefix){
  for(int i=0;i<=999;i++){
    char name[24];
    sprintf(name, "/%s%03d.CSV", prefix, i);
    if(!SD.exists(name)) return String(name);
  }
  return String("/") + prefix + String("999.CSV");
}
bool initSD(){
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if(!SD.begin(SD_CS, SPI, 8000000)){
    Serial.println("[SD] init failed.");
    return false;
  }
  Serial.println("[SD] init OK.");
  return true;
}
bool openLog(const char* prefix){
  String fn = nextLogFilename(prefix);
  logFile = SD.open(fn.c_str(), FILE_WRITE);
  if(!logFile){ Serial.println("[SD] open failed"); return false; }
  Serial.print("[SD] logging to "); Serial.println(fn);
#if defined(ROLE_MASTER)
  // Combined header
  logFile.print("# fw=" FW_VERSION "\n");
  logFile.println("time_ms,frame_id,R_ch0,R_ch1,R_ch2,R_ch3,R_ch4,R_ch5,R_ch6,R_ch7,R_ch8,R_ch9,R_ch10,R_ch11,R_ch12,R_ch13,R_ch14,R_ch15,"
                  "L_ch0,L_ch1,L_ch2,L_ch3,L_ch4,L_ch5,L_ch6,L_ch7,L_ch8,L_ch9,L_ch10,L_ch11,L_ch12,L_ch13,L_ch14,L_ch15,flags");
#else
  // Single-side header
  logFile.print("# fw=" FW_VERSION "\n");
  logFile.println("time_ms,frame_id,seq,ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15");
#endif
  logFile.flush();
#if PRINT_HEADER_SERIAL
  Serial.println("[FW] " FW_VERSION);
#endif
  return true;
}

// =================== OTA ===================
void setupOTA(){
#if OTA_ENABLE
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WiFi] Connecting");
  uint32_t t0 = millis();
  while(WiFi.status()!=WL_CONNECTED && (millis()-t0)<15000){
    Serial.print(".");
    delay(500);
  }
  if(WiFi.status()==WL_CONNECTED){
    Serial.print("\n[WiFi] Connected: "); Serial.println(WiFi.localIP());
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.begin();
  }else{
    Serial.println("\n[WiFi] Not connected (OTA disabled this session).");
  }
#else
  WiFi.mode(WIFI_STA); // needed for ESP-NOW
#endif
}

// =================== ESP-NOW ===================
void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void onDataRecv(const esp_now_recv_info* info, const uint8_t* incomingData, int len);

// ---- Definitions ----
void onDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  (void)info; (void)status;
  // Serial.printf("[ESPNOW] send status=%d\n", status);
}

void onDataRecv(const esp_now_recv_info* info, const uint8_t* incomingData, int len) {
  if (len < 2) return;
  const uint16_t type = *(const uint16_t*)incomingData;

#if defined(ROLE_MASTER)

  if (type == MSG_DATA && len >= (int)sizeof(DataMsg)) {
    const DataMsg* m = (const DataMsg*)incomingData;
    const int idx = findSlot(m->frame_id);

  #if defined(SIDE_RIGHT)
    // MASTER is RIGHT; incoming assumed LEFT
    memcpy(fbuf[idx].L, m->ch, sizeof(uint16_t)*16);
    fbuf[idx].haveL = true;
  #else
    memcpy(fbuf[idx].R, m->ch, sizeof(uint16_t)*16);
    fbuf[idx].haveR = true;
  #endif

  } else if (type == MSG_SYNC_RESP && len >= (int)sizeof(SyncResp)) {
    const SyncResp* s = (const SyncResp*)incomingData;
    const uint32_t t3 = millis();
    est_offset_ms = ((int32_t)s->t1_ms - (int32_t)s->t0_ms
                   + (int32_t)s->t2_ms - (int32_t)t3) / 2;
    // Serial.printf("[SYNC] offset≈%ld ms\n", (long)est_offset_ms);
  }

#else // ROLE_SLAVE

  if (type == MSG_TICK && len >= (int)sizeof(TickMsg)) {
    const TickMsg* t = (const TickMsg*)incomingData;
    int vals[16]; readAll16(vals);

    DataMsg out{};
    out.type = MSG_DATA;
    out.frame_id = t->frame_id;
    out.seq = ++slave_seq;
    out.t_local_ms = millis();
  #if defined(SIDE_RIGHT)
    out.side = 1;
  #else
    out.side = 0;
  #endif
    for (int i=0; i<16; i++) out.ch[i] = (uint16_t)vals[i];

    if (espnow_ready) esp_now_send(PEER_ADDR, (const uint8_t*)&out, sizeof(out));

  } else if (type == MSG_SYNC_REQ && len >= (int)sizeof(SyncReq)) {
    const SyncReq* q = (const SyncReq*)incomingData;
    SyncResp r{};
    r.type = MSG_SYNC_RESP;
    r.t0_ms = q->t0_ms;
    r.t1_ms = millis();
    r.t2_ms = millis();
    if (espnow_ready) esp_now_send(PEER_ADDR, (const uint8_t*)&r, sizeof(r));
  }

#endif
}

bool initESPNOW(){
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed");
    return false;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, PEER_ADDR, 6);
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  peer.channel = ESPNOW_CHANNEL; // 0 = follow current STA channel (if OTA connected)
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] add_peer failed");
    return false;
  }
  espnow_ready = true;
  Serial.println("[ESPNOW] ready");
  return true;
}

// =================== Master: frame buffer helpers ===================
#if defined(ROLE_MASTER)
int findSlot(uint32_t fid){
  int freeIdx = -1;
  for(int i=0;i<FRAME_BUF_SZ;i++){
    if(fbuf[i].frame_id == fid) return i;
    if(fbuf[i].frame_id==0 && freeIdx==-1) freeIdx = i;
  }
  if(freeIdx>=0){
    fbuf[freeIdx] = FrameEntry{};
    fbuf[freeIdx].frame_id = fid;
    fbuf[freeIdx].t_ms = millis();
    return freeIdx;
  }
  // overwrite the oldest
  int oldest = 0; uint32_t ot = fbuf[0].t_ms;
  for(int i=1;i<FRAME_BUF_SZ;i++){ if(fbuf[i].t_ms < ot){ ot=fbuf[i].t_ms; oldest=i; } }
  fbuf[oldest] = FrameEntry{};
  fbuf[oldest].frame_id = fid;
  fbuf[oldest].t_ms = millis();
  return oldest;
}
void flushReadyFrames(){
  uint32_t now = millis();
  for(int i=0;i<FRAME_BUF_SZ;i++){
    if(fbuf[i].frame_id==0) continue;
    bool timeout = (now - fbuf[i].t_ms) > 70; // 70 ms timeout
    if((fbuf[i].haveR && fbuf[i].haveL) || timeout){
      uint8_t flags = 0;
      if(!fbuf[i].haveL) flags |= 0x01;
      if(timeout)        flags |= 0x02;
      // write CSV line
      if(logFile){
        // Serial.print("[CSV] R: ");
        // for(int k=0;k<16;k++){
        //   Serial.print(fbuf[i].R[k]);
        //   if(k<15) Serial.print(",");
        // }
        // Serial.print(" | L: ");
        // for(int k=0;k<16;k++){
        //   Serial.print(fbuf[i].L[k]);
        //   if(k<15) Serial.print(",");
        // }
        // Serial.println();

        logFile.print(fbuf[i].t_ms);
        logFile.print(','); logFile.print(fbuf[i].frame_id);
        for(int k=0;k<16;k++){ logFile.print(','); logFile.print(fbuf[i].R[k]); }
        for(int k=0;k<16;k++){ logFile.print(','); logFile.print(fbuf[i].L[k]); }
        logFile.print(','); logFile.println(flags);
      }
      fbuf[i] = FrameEntry{}; // clear slot
    }
  }
}
#endif

// =================== Sensor read (16ch) ===================
void readAll16(int *out){
  for(uint8_t i=0;i<16;i++){
    uint8_t phys = (uint8_t)mux_map[i];
    out[i] = readChannel(phys);
  }
}

// =================== ESP-NOW receive handler ===================
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  if(len < 2) return;
  uint16_t type = *(uint16_t*)incomingData;
#if defined(ROLE_MASTER)
  if(type == MSG_DATA && len >= (int)sizeof(DataMsg)){
    const DataMsg* m = (const DataMsg*)incomingData;
    int idx = findSlot(m->frame_id);
    // Assume SLAVE is LEFT by default; if not, switch assignment
#if defined(SIDE_RIGHT)
    // MASTER is RIGHT; incoming assumed LEFT
    memcpy(fbuf[idx].L, m->ch, sizeof(uint16_t)*16);
#else
    memcpy(fbuf[idx].R, m->ch, sizeof(uint16_t)*16);
#endif
    fbuf[idx].haveL = true; // mark LEFT arrived (or RIGHT if MASTER is LEFT)
  } else if(type == MSG_SYNC_RESP && len >= (int)sizeof(SyncResp)){
    const SyncResp* s = (const SyncResp*)incomingData;
    uint32_t t3 = millis();
    int32_t o = ((int32_t)s->t1_ms - (int32_t)s->t0_ms + (int32_t)s->t2_ms - (int32_t)t3) / 2;
    est_offset_ms = o;
    Serial.printf("[SYNC] offset≈%ld ms (t0=%lu t1=%lu t2=%lu t3=%lu)\n",
                  (long)est_offset_ms, (unsigned long)s->t0_ms, (unsigned long)s->t1_ms,
                  (unsigned long)s->t2_ms, (unsigned long)t3);
  }
#else // ROLE_SLAVE
  if(type == MSG_TICK && len >= (int)sizeof(TickMsg)){
    const TickMsg* t = (const TickMsg*)incomingData;
    int vals[16]; readAll16(vals);
    DataMsg out{};
    out.type = MSG_DATA;
    out.frame_id = t->frame_id;
    out.seq = ++slave_seq;
    out.t_local_ms = millis();
#if defined(SIDE_RIGHT)
    out.side = 1;
#else
    out.side = 0;
#endif
    for(int i=0;i<16;i++) out.ch[i] = (uint16_t)vals[i];
    if(espnow_ready) esp_now_send(PEER_ADDR, (const uint8_t*)&out, sizeof(out));
  } else if(type == MSG_SYNC_REQ && len >= (int)sizeof(SyncReq)){
    const SyncReq* q = (const SyncReq*)incomingData;
    uint32_t t1 = millis();
    SyncResp r{};
    r.type = MSG_SYNC_RESP;
    r.t0_ms = q->t0_ms;
    r.t1_ms = t1;
    r.t2_ms = millis();
    if(espnow_ready) esp_now_send(PEER_ADDR, (const uint8_t*)&r, sizeof(r));
  }
#endif
}

// =================== Setup / Loop ===================
void setup(){
  Serial.begin(BAUD);
  delay(50);

#if defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);
#endif

  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
#if USE_EN
  pinMode(PIN_EN, OUTPUT);
  muxEnable(false);
#endif

  setupOTA();
  if(!initESPNOW()){
    Serial.println("[ERR] ESPNOW init failed, halting.");
    while(1) delay(1000);
  }

#if defined(ROLE_MASTER)
  if(!initSD()) Serial.println("[WARN] SD not available (Serial-only).");
  else openLog(LOG_PREFIX);
  Serial.print("[MAC] "); Serial.println(WiFi.macAddress());
#else
  if(!initSD()) Serial.println("[WARN] SD not available (Serial-only).");
  else openLog(LOG_PREFIX);
  Serial.print("[MAC] "); Serial.println(WiFi.macAddress());
#endif

#if PRINT_HEADER_SERIAL
  Serial.printf("[ROLE] %s | [SIDE] %s | LOG_HZ=%d\n",
#ifdef ROLE_MASTER
  "MASTER"
#else
  "SLAVE"
#endif
  ,
#ifdef SIDE_RIGHT
  "RIGHT"
#else
  "LEFT"
#endif
  , LOG_HZ);
#endif
}

void loop(){
#if OTA_ENABLE
  if(WiFi.status()==WL_CONNECTED) ArduinoOTA.handle();
#endif

#if defined(ROLE_MASTER)
  static uint32_t t0 = micros();
  uint32_t now_us = micros();
  if((now_us - t0) < FRAME_DT_US) return;
  t0 += FRAME_DT_US;

  // New frame
  uint32_t fid = ++cur_frame_id;
  int valsR[16]; readAll16(valsR); // MASTER assumed RIGHT by default

  // --- (For debugging) Sensor values serial output ---
  // Serial.print("[SENSOR] R: ");
  // for(int i=0; i<16; i++){
  //   Serial.print(valsR[i]);
  //   if(i < 15) Serial.print(",");
  // }

  // Store to buffer
  int idx = findSlot(fid);
#if defined(SIDE_RIGHT)
  for(int i=0; i<16; i++) fbuf[idx].R[i] = (uint16_t)valsR[i];
  fbuf[idx].haveR = true;
#else
  for(int i=0; i<16; i++) fbuf[idx].L[i] = (uint16_t)valsR[i];
  fbuf[idx].haveL = true;
#endif

  // --- SLAVE(LEFT) ---
  // if(fbuf[idx].haveL) {
  //   Serial.print(" | L: ");
  //   for(int i=0; i<16; i++){
  //     Serial.print(fbuf[idx].L[i]);
  //     if(i < 15) Serial.print(",");
  //   }
  // } else {
  //   Serial.print(" | L: (waiting)");
  // }
  // Serial.println();

  // Send TICK
  TickMsg t{}; t.type = MSG_TICK; t.frame_id = fid; t.fire_ms = millis();
  if(espnow_ready) esp_now_send(PEER_ADDR, (const uint8_t*)&t, sizeof(t));

  // Flush ready/timeout frames
  flushReadyFrames();

  // Periodic SYNC (every 5s)
  static uint32_t last_sync_ms = 0;
  if(millis() - last_sync_ms > 5000){
    SyncReq s{}; s.type = MSG_SYNC_REQ; s.t0_ms = millis();
    if(espnow_ready) esp_now_send(PEER_ADDR, (const uint8_t*)&s, sizeof(s));
    last_sync_ms = millis();
  }

  // Periodic SD flush
  static uint32_t lastFlush = 0;
  if(logFile && (millis() - lastFlush) > 500){
    logFile.flush();
    lastFlush = millis();
  }

#else // ROLE_SLAVE
  // SLAVE performs no timed loop; reacts to TICK in onDataRecv
  static uint32_t lastFlush = 0;
  if(logFile && (millis() - lastFlush) > 1000){
    logFile.flush();
    lastFlush = millis();
  }
#endif
}
