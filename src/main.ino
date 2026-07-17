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

// Set to 1 to bypass everything and just spam ASCII out Serial2 TX (GPIO43) so
// an FTDI on that pin can confirm the UART actually drives it. Set back to 0.
#define F9P_TX_TEST 0

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
uint32_t lastDiagMs  = 0;
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
// Returns true if every AT write was acknowledged OK.
static bool configureXBeeNetwork() {
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

  return bMM && bAO && bAP && bID && bCH && bMY && bAC;
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
// True if a checksum-valid NMEA sentence arrives within the window. (Mere byte
// presence isn't enough — garbage at the wrong baud also "has bytes".)
static bool f9pValidNmea(uint32_t ms) {
  uint32_t t0 = millis();
  char line[100];
  int len = -1;                                  // -1 = not inside a sentence
  while (millis() - t0 < ms) {
    while (Serial2.available()) {
      char c = Serial2.read();
      if (c == '$') {
        len = 0;
      } else if (len >= 0 && (c == '\r' || c == '\n')) {
        if (len > 4 && line[len - 3] == '*') {   // ...*HH
          uint8_t cs = 0;
          for (int i = 0; i < len - 3; i++) cs ^= (uint8_t)line[i];
          char hex[3] = { line[len - 2], line[len - 1], 0 };
          if ((uint8_t)strtol(hex, nullptr, 16) == cs) return true;
        }
        len = -1;
      } else if (len >= 0 && len < (int)sizeof(line) - 1) {
        line[len++] = c;
      }
    }
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

// UBX-CFG-VALSET (RAM): switch UART1 to 115200. Applied blind at the current baud.
static void f9pSetBaud115200() {
  uint8_t p[12] = { 0x00, 0x01, 0x00, 0x00,          // version, layer=RAM, reserved
                    0x01, 0x00, 0x52, 0x40,          // CFG-UART1-BAUDRATE (0x40520001)
                    0x00, 0xC2, 0x01, 0x00 };        // = 115200
  ubxSend(0x06, 0x8A, p, sizeof(p));
}

// UBX-CFG-VALSET (RAM): make any F9P plug-and-play — NMEA out + UBX/NMEA/RTCM3 in
// on UART1, and GGA emitted every nav epoch (regardless of prior config).
static void f9pConfigure() {
  const uint8_t cfg[] = {
    0x00, 0x01, 0x00, 0x00,                          // version, layer=RAM, reserved
    0x01, 0x00, 0x73, 0x10, 0x01,                    // CFG-UART1INPROT-UBX    = 1
    0x02, 0x00, 0x73, 0x10, 0x01,                    // CFG-UART1INPROT-NMEA   = 1
    0x04, 0x00, 0x73, 0x10, 0x01,                    // CFG-UART1INPROT-RTCM3X = 1 (corrections in)
    0x01, 0x00, 0x74, 0x10, 0x01,                    // CFG-UART1OUTPROT-UBX   = 1
    0x02, 0x00, 0x74, 0x10, 0x01,                    // CFG-UART1OUTPROT-NMEA  = 1
    0xBB, 0x00, 0x91, 0x20, 0x01,                    // CFG-MSGOUT-NMEA_ID_GGA_UART1 = 1
  };
  ubxSend(0x06, 0x8A, cfg, sizeof(cfg));
}

// Returns true if the F9P is producing valid NMEA after configuration.
static bool beginF9P() {
  Serial2.begin(F9P_BAUD, SERIAL_8N1, F9P_RX_PIN, F9P_TX_PIN);
  if (!f9pValidNmea(1500)) {
    // No valid NMEA at 115200: the F9P may be at its 38400 default and/or have
    // NMEA disabled. Blindly set baud + config from 38400, then reopen at 115200.
    Serial.println("[F9P] no valid NMEA at 115200; configuring from 38400 default");
    Serial2.begin(F9P_BAUD_DEFAULT, SERIAL_8N1, F9P_RX_PIN, F9P_TX_PIN);
    delay(100);
    f9pSetBaud115200();
    f9pConfigure();
    delay(300);
    Serial2.begin(F9P_BAUD, SERIAL_8N1, F9P_RX_PIN, F9P_TX_PIN);
    delay(100);
  }
  f9pConfigure();                                    // ensure messages are on at 115200
  delay(400);
  bool ok = f9pValidNmea(1500);
  if (ok)
    Serial.println("[F9P] ready — valid NMEA flowing");
  else
    Serial.println("[F9P] WARNING: no valid NMEA — check wiring (GPIO44<-F9P TX, "
                   "GPIO43->F9P RX, shared GND) and power");
  return ok;
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

// Boot-status line, updated step by step so you can watch XBee configure, then
// F9P. State per slot: 2=configuring, 1=OK, 0=FAIL, -1=waiting. Drawn during
// setup in the y22..42 band; drawStatus() never touches it, so it stays put.
enum { CFG_WAIT = -1, CFG_FAIL = 0, CFG_OK = 1, CFG_BUSY = 2 };

static void drawCfgSlot(int state) {
  switch (state) {
    case CFG_BUSY: tft.setTextColor(TFT_YELLOW, TFT_BLACK);   tft.print("CFG");  break;
    case CFG_OK:   tft.setTextColor(TFT_GREEN, TFT_BLACK);    tft.print("OK");   break;
    case CFG_FAIL: tft.setTextColor(TFT_RED, TFT_BLACK);      tft.print("FAIL"); break;
    default:       tft.setTextColor(TFT_DARKGREY, TFT_BLACK); tft.print("--");   break;
  }
}

static void drawCfgLine(int xbee, int f9p) {
  tft.fillRect(0, 22, tft.width(), 20, TFT_BLACK);
  tft.setTextFont(2);
  tft.setCursor(6, 24);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.print("XBee ");
  drawCfgSlot(xbee);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.print("   F9P ");
  drawCfgSlot(f9p);
}

// Split s into up to two lines of <= maxc chars, breaking on a space if possible.
static void wrapTwoLines(const char* s, char* l1, char* l2, size_t maxc) {
  size_t n = strlen(s);
  if (n <= maxc) {
    memcpy(l1, s, n);
    l1[n] = 0;
    l2[0] = 0;
    return;
  }
  size_t split = maxc;
  for (size_t i = maxc; i > 0; i--) {
    if (s[i] == ' ') { split = i; break; }
  }
  memcpy(l1, s, split);
  l1[split] = 0;
  const char* rest = s + split;
  while (*rest == ' ') rest++;
  size_t rlen = strlen(rest);
  if (rlen > maxc) rlen = maxc;
  memcpy(l2, rest, rlen);
  l2[rlen] = 0;
}

static void drawStatus() {
  int q = atoi(ggaQuality.value());
  const char* qtxt; uint16_t qcol;
  qualityLabel(q, qtxt, qcol);

  // fix quality (big)
  drawField(56, 30);
  tft.setTextFont(4);
  tft.setTextColor(qcol, TFT_BLACK);
  tft.setCursor(6, 58);
  tft.print(qtxt);

  // sats + hdop
  drawField(90, 18);
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(6, 90);
  tft.printf("Sat %lu  HDOP %.1f",
             (unsigned long)gps.satellites.value(),
             gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);

  // position: lat / lon / alt
  drawField(108, 58);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (gps.location.isValid()) {
    tft.setCursor(6, 108); tft.printf("Lat %.7f", gps.location.lat());
    tft.setCursor(6, 126); tft.printf("Lon %.7f", gps.location.lng());
    tft.setCursor(6, 144);
    if (gps.altitude.isValid()) tft.printf("Alt %.1f m", gps.altitude.meters());
    else                        tft.print("Alt --");
  } else {
    tft.setCursor(6, 108); tft.print("Lat --");
    tft.setCursor(6, 126); tft.print("Lon --");
    tft.setCursor(6, 144); tft.print("Alt --");
  }

  // link RSSI + bar
  drawField(168, 40);
  tft.setCursor(6, 168);
  if (haveRssi) {
    uint16_t rc = (downRssi >= RSSI_BAD) ? TFT_RED
                : (downRssi >= RSSI_WARN) ? TFT_ORANGE : TFT_GREEN;
    tft.setTextColor(rc, TFT_BLACK);
    tft.printf("Link %d dBm", -downRssi);
    float frac = (100.0f - downRssi) / 60.0f;   // -40 dBm full .. -100 dBm empty
    if (frac < 0) frac = 0;
    if (frac > 1) frac = 1;
    int bw = tft.width() - 12;
    tft.drawRect(6, 188, bw, 12, TFT_DARKGREY);
    tft.fillRect(8, 190, (int)((bw - 4) * frac), 8, rc);
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("Link --");
  }

  // corrections age
  drawField(206, 18);
  tft.setCursor(6, 206);
  if (lastRtcmMs == 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("RTCM --");
  } else {
    uint32_t age = millis() - lastRtcmMs;
    tft.setTextColor(age > RTCM_STALE_MS ? TFT_RED : TFT_GREEN, TFT_BLACK);
    tft.printf("RTCM %.1fs", age / 1000.0f);
  }

  // base info (telemetry text), wrapped to two lines
  drawField(228, 58);
  tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  tft.setCursor(6, 228);
  tft.print("Base:");
  if (haveBaseInfo && millis() - lastBaseMs < 5000) {
    char l1[24], l2[24];
    wrapTwoLines(baseInfo, l1, l2, 22);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(6, 248); tft.print(l1);
    tft.setCursor(6, 266); tft.print(l2);
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(6, 248); tft.print(haveBaseInfo ? "(stale)" : "--");
  }
}

// ===========================================================================
//  Setup / loop
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(2500);            // let the USB-CDC monitor reconnect so boot logs are visible

#if F9P_TX_TEST
  // Pin sanity check: drive GPIO43 (Serial2 TX) with readable ASCII forever.
  // On the FTDI (RX <- GPIO43, GND shared) at 115200 you should see clean lines.
  Serial2.begin(F9P_BAUD, SERIAL_8N1, F9P_RX_PIN, F9P_TX_PIN);
  Serial.printf("[TXtest] spamming ASCII on GPIO%d (Serial2 TX) @ %d baud\n",
                F9P_TX_PIN, F9P_BAUD);
  for (uint32_t n = 0;; n++) {
    Serial2.printf("monMon GPIO%d TX test #%lu @ %d baud\r\n", F9P_TX_PIN, n, F9P_BAUD);
    Serial.printf("[TXtest] sent #%lu\n", n);
    delay(500);
  }
#endif

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  // determine this rover's address (MAC-derived unless pinned in config)
  roverAddr = MONMON_ROVER_ADDR;
  if (roverAddr == 0x0000) {
    uint16_t a = (uint16_t)(ESP.getEfuseMac() & 0xFFFF);
    roverAddr = (a == 0x0000 || a == 0xFFFF) ? 0x0001 : a;
  }

  // display header: "monMon 0xADDR" + a boot-status line
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  tft.setCursor(6, 2);
  tft.printf("monMon  0x%04X", roverAddr);

  // radios — configure the XBee first, then the F9P, updating the status line
  // at each step so progress is visible.
  drawCfgLine(CFG_BUSY, CFG_WAIT);            // XBee configuring, F9P waiting
  bool xbeeOk = false;
  if (xbeeBootstrap()) {
    xbeeDiag();
    xbeeOk = configureXBeeNetwork();
  } else {
    Serial.println("[XBee] bootstrap FAILED — check wiring (DOUT->17, DIN->18)");
  }

  drawCfgLine(xbeeOk ? CFG_OK : CFG_FAIL, CFG_BUSY);   // XBee done, F9P configuring
  bool f9pOk = beginF9P();

  drawCfgLine(xbeeOk ? CFG_OK : CFG_FAIL, f9pOk ? CFG_OK : CFG_FAIL);   // both done

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

  // ---- F9P health diagnostic (1 Hz) ----
  if (millis() - lastDiagMs >= 1000) {
    lastDiagMs = millis();
    Serial.printf("[gps] chars=%lu fixSentences=%lu cksumErr=%lu | q=%d sats=%lu ",
                  gps.charsProcessed(), gps.sentencesWithFix(), gps.failedChecksum(),
                  atoi(ggaQuality.value()), (unsigned long)gps.satellites.value());
    if (gps.location.isValid())
      Serial.printf("lat=%.7f lon=%.7f alt=%.1fm\n",
                    gps.location.lat(), gps.location.lng(), gps.altitude.meters());
    else
      Serial.println("LLA=--");
  }

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
