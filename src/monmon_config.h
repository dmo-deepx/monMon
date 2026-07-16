#pragma once
#include <stdint.h>

// ============================================================================
//  monMon shared configuration (rover firmware)
//  Force-applied to the XBee on every boot, so a module only needs API mode
//  (AP=1, non-escaped) enabled once via XCTU — everything else is set here.
// ============================================================================

// ---- XBee 802.15.4 network ----
#define MONMON_PAN_ID     0x3332   // ID  — must match every node in the swarm
#define MONMON_CHANNEL    0x0C     // CH  — 802.15.4 channel (0x0B..0x1A)
#define MONMON_BASE_ADDR  0x0000   // base MY, and the rover's unicast destination

// Rover's own 16-bit address (MY). 0x0000 => derive a unique one from the ESP
// MAC (recommended: no per-unit build). Set nonzero to pin a specific address.
#define MONMON_ROVER_ADDR 0x0000

// ---- XBee serial link (ESP Serial1) ----
#define XBEE_BAUD    115200
#define XBEE_RX_PIN  17            // ESP RX  <- XBee DOUT
#define XBEE_TX_PIN  18            // ESP TX  -> XBee DIN

// ---- F9P serial link (ESP Serial2) ----
#define F9P_BAUD          115200   // target baud
#define F9P_BAUD_DEFAULT  38400    // u-blox factory default (used to bootstrap)
#define F9P_RX_PIN        44       // ESP RX  <- F9P TX
#define F9P_TX_PIN        43       // ESP TX  -> F9P RX

// ---- Behaviour ----
#define POS_REPORT_MS  1000        // rover -> base position cadence (1 Hz)
#define RTCM_STALE_MS  5000        // corrections considered stale after this
#define DISPLAY_MS     250         // TFT refresh period

// RSSI warning thresholds, expressed as positive magnitude (dBm = -value)
#define RSSI_WARN  85              // >= -85 dBm : getting weak (orange)
#define RSSI_BAD   95              // >= -95 dBm : near edge of range (red)
