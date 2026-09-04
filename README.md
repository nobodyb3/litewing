# LiteWing on ESP8266 — port notes

This is a port of `LiteWing-Arduino-Drone-V1.0` (originally ESP32-S3 +
ESP-NOW joystick) to ESP8266, using WiFi SoftAP + UDP instead of ESP-NOW so
it can talk to the **LiteWing Android app** (and other ESP-Drone/Crazyflie
CRTP clients), the same way the *real* LiteWing firmware does — that real
firmware is the ESP-IDF code in this repo (`components/`, compiled to the
`.bin`s), which is a fork of Espressif's **ESP-Drone**, itself a fork of
Bitcraze's **Crazyflie** firmware. The Arduino sketch you gave me never
talked to the app at all; it talked to a second ESP32 running
`ESPnow_joystick.ino`. This port swaps that link out.

## What I copied straight from the ESP-IDF firmware

Read from `components/drivers/general/wifi/wifi_esp32.c` and
`main/Kconfig.projbuild`, so the app's defaults still line up:

- SoftAP SSID: `LiteWing_XXXXXX` (last 3 MAC bytes), password `12345678`, channel 6
- Static IP `192.168.43.42`, DHCP on for clients
- UDP server on port **2390**
- Every UDP packet is CRTP payload + **1 trailing checksum byte** (sum of
  the preceding bytes, truncated to uint8)
- Flight control uses CRTP port 3 (`SETPOINT`), the "legacy RPYT" packet:
  `float roll; float pitch; float yaw; uint16_t thrust;` (14 bytes)

## Files

- `LiteWing_ESP8266.ino` — main sketch
- `WiFi_CRTP.h` — SoftAP + UDP CRTP transport (replaces `ESP_NOW.h`)
- `motors.h` — software-PWM motor driver (replaces `ledc`-based original)
- `PID.h`, `read_YPR.h` — **unchanged**, copied as-is (pure math/I2C, no
  ESP32-specific calls, so they needed no porting)

## Wiring (Wemos D1 Mini / NodeMCU)

| Function      | Pin        |
|---------------|------------|
| MPU6050 SDA   | D2 (GPIO4) |
| MPU6050 SCL   | D1 (GPIO5) |
| Motor 1 (FR)  | D5 (GPIO14)|
| Motor 2 (BR)  | D6 (GPIO12)|
| Motor 3 (BL)  | D7 (GPIO13)|
| Motor 4 (FL)  | D8 (GPIO15)|

D8/GPIO15 must idle LOW at boot for the board to boot at all — that's
actually convenient here since it means motors default off. D0/GPIO16,
D3/GPIO0 and D4/GPIO2 are deliberately left unused (boot-strapping pins,
risky to drive a motor gate with them). Each motor pin drives a
MOSFET/ESC input, same as the original design — this assumes you're
building your own frame/motor driver board, since the official LiteWing
PCB is wired specifically for the ESP32-S3's pinout and won't drop an
ESP8266 in directly.

## Arduino IDE setup

1. Install the "ESP8266" board package (Boards Manager URL:
   `http://arduino.esp8266.com/stable/package_esp8266com_index.json`)
2. Install the `MPU6050` library (electronic cats / jrowberg i2cdevlib —
   same one the original sketch uses) and its `I2Cdev` dependency
3. Select your board (e.g. "LOLIN(WEMOS) D1 R2 & mini" or "NodeMCU 1.0")
4. Open `LiteWing_ESP8266.ino` and upload

## ⚠️ Known limitations / what's untested

I don't have this hardware to test against, and the LiteWing app is
closed-source, so I built this from the documented CRTP/ESP-Drone
protocol, not from a packet capture. Be the test pilot carefully — props
off for the first bench test:

1. **Flight control (throttle/roll/pitch/yaw) should work** once the
   app connects — that only needs CRTP port 3, which is implemented.
2. **Everything else the app does is NOT implemented yet**: battery
   voltage display, altitude hold, trim get/set persistence, arming
   state feedback, and the log/param "TOC" handshake some CRTP clients
   expect before they consider a connection healthy. If the app shows
   a permanent "connecting…" state or disconnects after a few seconds,
   it's very likely waiting on one of these (most likely CRTP_PORT_LOG
   TOC responses, or a periodic ping on CRTP_PORT_LINK). Capturing the
   app's traffic once (e.g. with Wireshark on the phone's hotspot, or
   `tcpdump` if you have a rooted/jailbroken test device) would tell us
   exactly which ports it insists on — happy to add handlers for
   whatever shows up.
3. **PWM on ESP8266 is software-generated**, not the ESP32's hardware
   `ledc`. I set `analogWriteFreq(20000)` to stay ultrasonic/quiet, but
   ESP8266 cores vary in how well they hold a clean 20kHz waveform
   under WiFi interrupt load — if motors sound rough or stutter, try
   dropping `FREQ` in `motors.h` to 1000–4000 Hz first.
4. **Single core, no RTOS**: the real firmware runs WiFi/UDP and flight
   control as separate FreeRTOS tasks on a dual-core chip. This sketch
   runs everything cooperatively in one `loop()`. `handleWiFiCRTP()` is
   non-blocking so it shouldn't stall the PID loop, but if you add more
   CRTP handling later, keep it non-blocking or your loop rate (and
   therefore flight stability) will suffer.
5. **No ESP8266 board is a drop-in replacement for the LiteWing PCB.**
   This assumes you're wiring your own motors/MOSFETs/MPU6050 to a
   NodeMCU/D1 Mini per the table above.

## Suggested next steps (Phase 2, not included here)

- Add `CRTP_PORT_LOG` handling: respond to TOC requests with an empty
  or minimal table, and stream battery voltage as a log block if the
  app polls for it.
- Add `CRTP_PORT_PARAM` handling for the trim sliders.
- Add a battery voltage read on `A0` (ESP8266 ADC is 0–1V, so you'll
  need a resistor divider) and report it.
