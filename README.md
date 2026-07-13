# monMon — XBee RF monitor for GPS/RTCM

A remote radio monitor built on the **LilyGO T-Display-S3** (ESP32-S3 + 1.9" 170×320 TFT).
An **XBee XB3-24AUT-J** on `Serial1` receives packets over the air; the sketch parses them
and renders live status on the display. The current sketch is an XBee **RSSI monitor**
(signal-strength dashboard) — the skeleton that GPS/RTCM parsing is being built onto.

## Hardware

| | |
|---|---|
| Board | LilyGO T-Display-S3 (ESP32-S3R8, 16MB flash, 8MB PSRAM) |
| Display | 170×320 ST7789 TFT via TFT_eSPI (pre-configured, `Setup206`) |
| Radio | XBee XB3-24AUT-J on `Serial1` — **RX = GPIO 17, TX = GPIO 18**, 115200 baud |
| Buttons | GPIO 0 and GPIO 14 |
| Touch (Touch variant only) | CST816S / CST328 capacitive, pins 16 / 21 |

Pin map lives in [`src/pin_config.h`](src/pin_config.h).

## Project layout

```
src/            main.ino + pin_config.h + bundled fonts
lib/            vendored libraries (no downloads needed — builds offline)
boards/         lilygo-t-displays3 board definition
platformio.ini  single build env: monMon
datasheet/ dimensions/ firmware/ schematic/   hardware reference
```

Vendored libraries: `TFT_eSPI` (display/draw/sprites/fonts), `xbee-arduino`,
`TinyGPSPlus` (NMEA), `GFX Library for Arduino`, `OneButton`, `lvgl`, `TouchLib`,
`SensorLib`. Only the ones a sketch actually `#include`s get compiled.

## Build & run

This project uses [PlatformIO](https://platformio.org/). The toolchain
(`espressif32@6.5.0`) is cached locally, so builds work fully offline.

> **Note:** the `pio` CLI is not on `PATH` on this machine — use the full path
> `~/.platformio/penv/bin/pio`, or add it to your shell:
> `export PATH="$HOME/.platformio/penv/bin:$PATH"`

```bash
# Compile
pio run

# Flash the board over USB (auto-detects the port)
pio run -t upload

# Flash and then open the serial monitor
pio run -t upload -t monitor

# Just open the serial monitor (115200 baud)
pio device monitor
```

If the upload port isn't detected, plug in the board via USB-C and pass it
explicitly, e.g. `pio run -t upload --upload-port /dev/cu.usbmodemXXXX`
(`pio device list` shows attached ports).

## Notes

- Requires **Arduino-ESP32 < 3.0** — the sketch `#error`s on ESP-IDF ≥ 5.
- Based on the LilyGO T-Display-S3 example repo, trimmed down to a minimal
  single-project layout.
