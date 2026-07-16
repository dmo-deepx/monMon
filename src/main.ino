// ============================================================================
//  monMon — ROVER firmware  (LilyGO T-Display-S3, ESP32-S3)
//
//  Bridges a u-blox ZED-F9P to an XBee 3 (802.15.4 API) radio link:
//    • RTCM3 corrections arrive from the base (broadcast) -> streamed to F9P
//    • F9P NMEA (GGA) is sent back to the base, unicast + ACKed, at 1 Hz
//    • TFT shows fix quality, sats, position, link RSSI and correction age
//    • A button press sends a stakeout MARK (Phase 4 seed)
//
//  The XBee network (PAN ID / channel / this rover's address) is force-applied
//  on every boot from monmon_config.h — the module only needs API mode (AP=1)
//  enabled once. Rover address defaults to a unique value derived from the MAC.
// ============================================================================

#include <TFT_eSPI.h>
#include <SPI.h>
#include <XBee.h>
#include <TinyGPS++.h>
#include <OneButton.h>

#include "pin_config.h"
#include "monmon_config.h"
#include "monmon_packet.h"

// ---- peripherals ----------------------------------------------------------
TFT_eSPI     tft = TFT_eSPI();
XBee         xbee = XBee();
TinyGPSPlus  gps;
TinyGPSCustom ggaQuality(gps, "GNGGA", 6);   // GGA fix-quality field
OneButton    btn1(PIN_BUTTON_1, true, true);
OneButton    btn2(PIN_BUTTON_2, true, true);

// ---- state ----------------------------------------------------------------
uint16_t roverAddr   = 0;
uint8_t  txSeq       = 0;
uint32_t lastPosMs   = 0;
uint32_t lastDispMs  = 0;
uint32_t lastRtcmMs  = 0;
int      downRssi    = 0;        // positive magnitude; dBm = -downRssi
bool     haveRssi    = false;

char     lineBuf[128];
size_t   lineLen     = 0;
char     lastGga[120];
bool     haveGga     = false;

volatile bool markPending = false;

char     baseInfo[80];
bool     haveBaseInfo = false;
uint32_t lastBaseMs   = 0;

// ===========================================================================
//  XBee helpers
// ===========================================================================
static bool xbeeAt(const char* cmd, uint8_t* val, uint8_t vlen) {
  while (Serial1.available()) Serial1.read();      // drop stale frames first
  uint8_t c[2] = { (uint8_t)cmd[0], (uint8_t)cmd[1] };
  AtCommandRequest req(c, val, vlen);
  xbee.send(req);
  // Keep reading until we see the AT response for *this* command (skip modem
  // status / other frames the module may interleave), or give up.
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    if (xbee.readPacket(200) && xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      AtCommandResponse r;
      xbee.getResponse().getAtCommandResponse(r);
      uint8_t* rc = r.getCommand();
      if (rc[0] == c[0] && rc[1] == c[1]) {
        if (!r.isOk())
          Serial.printf("[XBee] AT %c%c rejected, status=%d "
                        "(2=invalid cmd/wrong protocol, 3=invalid param)\n",
                        c[0], c[1], r.getStatus());
        return r.isOk();
      }
    }
  }
  return false;
}

// Query an AT parameter's value. Returns value length, or <0 on error.
static int xbeeAtQuery(const char* cmd, uint8_t* out, int maxLen) {
  while (Serial1.available()) Serial1.read();
  uint8_t c[2] = { (uint8_t)cmd[0], (uint8_t)cmd[1] };
  AtCommandRequest req(c);
  xbee.send(req);
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    if (xbee.readPacket(200) && xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
      AtCommandResponse r;
      xbee.getResponse().getAtCommandResponse(r);
      uint8_t* rc = r.getCommand();
      if (rc[0] == c[0] && rc[1] == c[1]) {
        if (!r.isOk()) return -1;
        int n = r.getValueLength();
        if (n > maxLen) n = maxLen;
        memcpy(out, r.getValue(), n);
        return n;
      }
    }
  }
  return -2;
}

// Dump identifying params. On XBee 3 the first VR nibble = protocol:
// 1xxx Zigbee, 2xxx 802.15.4, 3xxx DigiMesh.
static void xbeeDiag() {
  uint8_t v[8];
  int n = xbeeAtQuery("VR", v, sizeof(v));
  Serial.print("[XBee] VR=");
  for (int i = 0; i < n; i++) Serial.printf("%02X", v[i]);
  Serial.printf("  (%s)\n", (n > 0 && (v[0] >> 4) == 0x2) ? "802.15.4"
                          : (n > 0 && (v[0] >> 4) == 0x3) ? "DigiMesh"
                          : (n > 0 && (v[0] >> 4) == 0x1) ? "Zigbee" : "unknown");
  n = xbeeAtQuery("HV", v, sizeof(v));
  Serial.print("[XBee] HV=");
  for (int i = 0; i < n; i++) Serial.printf("%02X", v[i]);
  Serial.println();
  n = xbeeAtQuery("AP", v, sizeof(v));
  Serial.printf("[XBee] AP=%d\n", n > 0 ? v[0] : -1);
}

// Query one AT parameter over API; true if the module answers OK (=> API mode).
static bool xbeeApiProbe() {
  uint8_t c[2] = { 'A', 'P' };
  AtCommandRequest req(c);            // no value => query
  xbee.send(req);
  if (xbee.readPacket(700) && xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
    AtCommandResponse r;
    xbee.getResponse().getAtCommandResponse(r);
    return r.isOk();
  }
  return false;
}

// ---- transparent-mode (AT command mode) fallback -------------------------
static bool xbeeExpectOK(uint32_t timeout) {
  uint32_t t0 = millis();
  String s;
  while (millis() - t0 < timeout) {
    while (Serial1.available()) {
      s += (char)Serial1.read();
      if (s.endsWith("OK\r")) return true;
    }
  }
  return false;
}

static bool xbeeEnterCmdMode() {
  delay(1100);              // pre-guard silence (default GT = 1s)
  while (Serial1.available()) Serial1.read();
  Serial1.print("+++");     // no CR
  return xbeeExpectOK(1500);
}

static bool xbeeCmd(const char* atline) {
  Serial1.print(atline);
  Serial1.print('\r');
  return xbeeExpectOK(1500);
}

// Candidate bauds to probe, most-likely first (115200 = our target, 9600 = factory).
static const long XBEE_BAUDS[] = { 115200, 9600, 38400, 57600, 19200, 230400 };

// Bring the XBee to API mode @ 115200 from *any* starting state, and persist it.
static bool xbeeBootstrap() {
  // Pass 1: already in API mode at some baud?
  for (long b : XBEE_BAUDS) {
    Serial1.begin(b, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
    xbee.setSerial(Serial1);
    delay(150);
    while (Serial1.available()) Serial1.read();
    if (xbeeApiProbe()) {
      Serial.printf("[XBee] API mode at %ld baud\n", b);
      if (b != XBEE_BAUD) {                 // switch module to 115200 and persist
        uint8_t bd = 7;                     // 7 = 115200
        xbeeAt("BD", &bd, 1);
        xbeeAt("WR", nullptr, 0);
        xbeeAt("AC", nullptr, 0);
        delay(200);
        Serial1.begin(XBEE_BAUD, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
        delay(150);
      }
      return true;
    }
  }
  // Pass 2: transparent mode — convert via AT command mode.
  for (long b : XBEE_BAUDS) {
    Serial1.begin(b, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
    xbee.setSerial(Serial1);
    delay(150);
    if (xbeeEnterCmdMode()) {
      Serial.printf("[XBee] transparent mode at %ld baud; converting to API\n", b);
      xbeeCmd("ATAP2");                       // escaped API (xbee-arduino requires it)
      xbeeCmd("ATBD7");                        // 115200 (applies on exit)
      xbeeCmd("ATWR");                         // persist
      xbeeCmd("ATCN");                         // exit command mode
      delay(300);
      Serial1.begin(XBEE_BAUD, SERIAL_8N1, XBEE_RX_PIN, XBEE_TX_PIN);
      delay(150);
      while (Serial1.available()) Serial1.read();
      if (xbeeApiProbe()) return true;
    }
  }
  return false;
}

// Force this swarm's network params (volatile — reapplied every boot, no flash wear).
static void configureXBeeNetwork() {
  delay(200);
  while (Serial1.available()) Serial1.read();       // let post-conversion frames drain

  uint8_t pan[2] = { (uint8_t)(MONMON_PAN_ID >> 8), (uint8_t)(MONMON_PAN_ID & 0xFF) };
  uint8_t ch     = MONMON_CHANNEL;
  uint8_t my[2]  = { (uint8_t)(roverAddr >> 8),     (uint8_t)(roverAddr & 0xFF) };
  uint8_t zero = 0, two = 2;

  // Escaped API (AP=2) MUST be set first and applied: xbee-arduino always escapes
  // its frames, so the module must match — otherwise bytes 0x7E/0x7D/0x11/0x13
  // (common in RTCM data and in AT-frame checksums) corrupt the frame.
  bool bAP = xbeeAt("AP", &two, 1);
  xbeeAt("AC", nullptr, 0);
  delay(100);

  // Framing personality: Series-1-compatible frames for xbee-arduino.
  bool bMM = xbeeAt("MM", &zero, 1);   // Digi Mode (802.15.4 + Digi header): clean framing, ACKs + retries
  bool bAO = xbeeAt("AO", &two, 1);    // legacy 0x80/0x81 RX frames (carry per-packet RSSI)
  bool bID = xbeeAt("ID", pan, 2);
  bool bCH = xbeeAt("CH", &ch, 1);
  bool bMY = xbeeAt("MY", my, 2);
  bool bAC = xbeeAt("AC", nullptr, 0);
  Serial.printf("[XBee] set MM=%d AO=%d AP=%d ID=%d CH=%d MY=%d AC=%d\n",
                bMM, bAO, bAP, bID, bCH, bMY, bAC);

  // Read the values back so we know what actually stuck on the module.
  delay(100);
  uint8_t v[4];
  int n;
  n = xbeeAtQuery("MM", v, sizeof(v));
  Serial.printf("[XBee] read MM=%d", n > 0 ? v[0] : -1);
  n = xbeeAtQuery("AO", v, sizeof(v));
  Serial.printf(" AO=%d", n > 0 ? v[0] : -1);
  n = xbeeAtQuery("MY", v, sizeof(v));
  Serial.print(" MY=");
  for (int i = 0; i < n; i++) Serial.printf("%02X", v[i]);
  Serial.printf(" (wanted MY=%04X)\n", roverAddr);
}

// Send an application packet to the base, fragmenting if larger than one frame.
static void sendPacket(uint8_t type, const uint8_t* data, size_t len) {
  size_t off = 0;
  do {
    uint8_t chunk = (len - off > monmon::MAX_DATA) ? monmon::MAX_DATA
                                                   : (uint8_t)(len - off);
    uint8_t flags = (off + chunk < len) ? monmon::FLAG_MORE : 0;
    uint8_t frame[monmon::MAX_FRAME];
    uint8_t flen  = monmon::encode(frame, type, txSeq++, flags, data + off, chunk);
    Tx16Request tx(MONMON_BASE_ADDR, frame, flen);   // unicast + MAC ACK/retry
    xbee.send(tx);
    off += chunk;
  } while (off < len);
}

// ===========================================================================
//  F9P helpers  (auto-baud bootstrap + UBX config)
// ===========================================================================
static bool f9pSawData(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (Serial2.available()) return true;
  }
  return false;
}

static void ubxSend(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
  uint8_t head[6] = { 0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t a = 0, b = 0;
  for (int i = 2; i < 6; i++) { a += head[i]; b += a; }
  Serial2.write(head, 6);
  for (uint16_t i = 0; i < len; i++) { Serial2.write(payload[i]); a += payload[i]; b += a; }
  Serial2.write(a);
  Serial2.write(b);
}

static void f9pSetBaud115200() {
  // UBX-CFG-VALSET: CFG-UART1-BAUDRATE (0x40520001) = 115200, layers RAM+BBR+Flash
  uint8_t p[12] = { 0x00, 0x07, 0x00, 0x00,
                    0x01, 0x00, 0x52, 0x40,
                    0x00, 0xC2, 0x01, 0x00 };
  ubxSend(0x06, 0x8A, p, sizeof(p));
}

static void beginF9P() {
  Serial2.begin(F9P_BAUD, SERIAL_8N1, F9P_RX_PIN, F9P_TX_PIN);
  if (!f9pSawData(1500)) {
    Serial.println("[F9P] silent at target baud — bootstrapping from default");
    Serial2.begin(F9P_BAUD_DEFAULT, SERIAL_8N1, F9P_RX_PIN, F9P_TX_PIN);
    delay(100);
    f9pSetBaud115200();
    delay(200);
    Serial2.begin(F9P_BAUD, SERIAL_8N1, F9P_RX_PIN, F9P_TX_PIN);
  }
  Serial.println("[F9P] ready");
}

// ===========================================================================
//  Buttons
// ===========================================================================
static void onMark() { markPending = true; }

// ===========================================================================
//  Display
// ===========================================================================
static void qualityLabel(int q, const char*& text, uint16_t& color) {
  switch (q) {
    case 1:  text = "GPS";       color = TFT_WHITE;  break;
    case 2:  text = "DGPS";      color = TFT_CYAN;   break;
    case 4:  text = "RTK FIX";   color = TFT_GREEN;  break;
    case 5:  text = "RTK FLOAT"; color = TFT_ORANGE; break;
    case 6:  text = "DEAD RECK"; color = TFT_YELLOW; break;
    default: text = "NO FIX";    color = TFT_RED;    break;
  }
}

static void drawField(int y, int h) {
  tft.fillRect(0, y, tft.width(), h, TFT_BLACK);
}

static void drawStatus() {
  int q = atoi(ggaQuality.value());
  const char* qtxt; uint16_t qcol;
  qualityLabel(q, qtxt, qcol);

  // fix quality (big)
  drawField(56, 34);
  tft.setTextFont(4);
  tft.setTextColor(qcol, TFT_BLACK);
  tft.setCursor(6, 58);
  tft.print(qtxt);

  // sats + hdop
  drawField(96, 20);
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(6, 98);
  tft.printf("Sat %lu  HDOP %.1f",
             (unsigned long)gps.satellites.value(),
             gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);

  // position
  drawField(118, 40);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(6, 120);
  if (gps.location.isValid()) {
    tft.printf("Lat %.7f", gps.location.lat());
    tft.setCursor(6, 138);
    tft.printf("Lon %.7f", gps.location.lng());
  } else {
    tft.print("Lat --");
    tft.setCursor(6, 138);
    tft.print("Lon --");
  }

  // link RSSI + bar
  drawField(168, 44);
  tft.setCursor(6, 170);
  if (haveRssi) {
    uint16_t rc = (downRssi >= RSSI_BAD) ? TFT_RED
                : (downRssi >= RSSI_WARN) ? TFT_ORANGE : TFT_GREEN;
    tft.setTextColor(rc, TFT_BLACK);
    tft.printf("Link %d dBm", -downRssi);
    // bar: -40 dBm (full) .. -100 dBm (empty)
    float frac = (100.0f - downRssi) / 60.0f;
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    int bw = tft.width() - 12;
    tft.drawRect(6, 192, bw, 14, TFT_DARKGREY);
    tft.fillRect(8, 194, (int)((bw - 4) * frac), 10, rc);
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("Link --");
  }

  // corrections age
  drawField(214, 20);
  tft.setCursor(6, 216);
  if (lastRtcmMs == 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("RTCM --");
  } else {
    uint32_t age = millis() - lastRtcmMs;
    tft.setTextColor(age > RTCM_STALE_MS ? TFT_RED : TFT_GREEN, TFT_BLACK);
    tft.printf("RTCM %.1fs", age / 1000.0f);
  }

  // base info (telemetry text from the base station)
  drawField(238, 62);
  tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  tft.setCursor(6, 240);
  tft.print("Base:");
  tft.setCursor(6, 258);
  if (haveBaseInfo && millis() - lastBaseMs < 5000) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(baseInfo);
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print(haveBaseInfo ? "(stale)" : "--");
  }
}

// ===========================================================================
//  Setup / loop
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(2500);            // let the USB-CDC monitor reconnect so boot logs are visible

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  // determine this rover's address (MAC-derived unless pinned in config)
  roverAddr = MONMON_ROVER_ADDR;
  if (roverAddr == 0x0000) {
    uint16_t a = (uint16_t)(ESP.getEfuseMac() & 0xFFFF);
    roverAddr = (a == 0x0000 || a == 0xFFFF) ? 0x0001 : a;
  }

  // display
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(4);
  tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  tft.setCursor(6, 6);
  tft.print("monMon");
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(6, 36);
  tft.printf("rover 0x%04X", roverAddr);

  // radios — bootstrap the XBee from any state, then apply network params
  tft.setCursor(6, 36);
  tft.printf("rover 0x%04X  XBee...", roverAddr);
  if (xbeeBootstrap()) {
    xbeeDiag();
    configureXBeeNetwork();
  } else {
    Serial.println("[XBee] bootstrap FAILED — check wiring (DOUT->17, DIN->18)");
  }
  tft.fillRect(0, 34, tft.width(), 18, TFT_BLACK);
  tft.setCursor(6, 36);
  tft.printf("rover 0x%04X", roverAddr);
  beginF9P();

  // buttons -> stakeout mark
  btn1.attachClick(onMark);
  btn2.attachClick(onMark);

  Serial.printf("[monMon] rover 0x%04X ready\n", roverAddr);
}

void loop() {
  // ---- downlink: RTCM from base -> F9P ----
  xbee.readPacket();
  if (xbee.getResponse().isAvailable() &&
      xbee.getResponse().getApiId() == RX_16_RESPONSE) {
    Rx16Response rx;
    xbee.getResponse().getRx16Response(rx);
    downRssi = rx.getRssi();
    haveRssi = true;
    monmon::Frame f;
    if (monmon::decode(rx.getData(), rx.getDataLength(), f)) {
      if (f.type == monmon::PKT_RTCM) {
        Serial2.write(f.data, f.len);    // F9P parses/validates RTCM itself
        lastRtcmMs = millis();
      } else if (f.type == monmon::PKT_TELEM) {
        uint8_t n = f.len < sizeof(baseInfo) - 1 ? f.len : sizeof(baseInfo) - 1;
        memcpy(baseInfo, f.data, n);
        baseInfo[n] = 0;
        haveBaseInfo = true;
        lastBaseMs = millis();
      }
    }
  }

  // ---- read F9P: feed parser + capture latest GGA ----
  while (Serial2.available()) {
    char c = Serial2.read();
    gps.encode(c);
    if (c == '\n' || c == '\r') {
      if (lineLen > 6 && lineBuf[0] == '$') {
        lineBuf[lineLen] = 0;
        if (strstr(lineBuf, "GGA")) {
          strncpy(lastGga, lineBuf, sizeof(lastGga) - 1);
          lastGga[sizeof(lastGga) - 1] = 0;
          haveGga = true;
        }
      }
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }

  // ---- uplink: position to base at 1 Hz ----
  if (millis() - lastPosMs >= POS_REPORT_MS) {
    lastPosMs = millis();
    if (haveGga) {
      char out[126];
      int n = snprintf(out, sizeof(out), "%s\r\n", lastGga);
      sendPacket(monmon::PKT_NMEA, (const uint8_t*)out, n);
    }
  }

  // ---- stakeout mark ----
  if (markPending) {
    markPending = false;
    char m[80];
    int n = gps.location.isValid()
              ? snprintf(m, sizeof(m), "MARK,%.7f,%.7f,%d",
                         gps.location.lat(), gps.location.lng(), atoi(ggaQuality.value()))
              : snprintf(m, sizeof(m), "MARK,,,0");
    sendPacket(monmon::PKT_CONTROL, (const uint8_t*)m, n);
    Serial.printf("[monMon] MARK sent: %s\n", m);
  }

  btn1.tick();
  btn2.tick();

  // ---- display ----
  if (millis() - lastDispMs >= DISPLAY_MS) {
    lastDispMs = millis();
    drawStatus();
  }
}

// ---- build-time guards (keep the TFT_eSPI setup + ESP core sane) -----------
#if PIN_LCD_WR != TFT_WR || PIN_LCD_RD != TFT_RD || PIN_LCD_CS != TFT_CS || \
    PIN_LCD_DC != TFT_DC || PIN_LCD_RES != TFT_RST || \
    170 != TFT_WIDTH || 320 != TFT_HEIGHT
#error "Select <User_Setups/Setup206_LilyGo_T_Display_S3.h> in TFT_eSPI/User_Setup_Select.h"
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#error "Use Arduino-ESP32 < 3.0 (this project targets espressif32@6.5.0)"
#endif
