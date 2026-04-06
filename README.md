# Camera Slider Controller

A feature-complete Arduino/ESP32 firmware for a motorised camera slider rail, with a wired handheld controller, programmable motion sequences, and optional WiFi control.

---

## Overview

This project controls a stepper-motor camera slider using an Arduino or ESP32 microcontroller. The user interface lives in a **wired handheld enclosure** containing a Mini12864 OLED display, rotary encoder, and three buttons, connected to the main driver board via a multi-wire cable.

Programs define a sequence of positions with dwell timers at each stop. Up to four user programs can be stored and recalled, with settings surviving power cycles via EEPROM or flash storage.

---

## Hardware

| Component | Part |
|---|---|
| Stepper motor | NEMA 17 |
| Stepper driver | TMC2209 |
| Display + encoder | BIGTREETECH Mini12864 V2.0 |
| Endstops | 2× NC micro switch |
| Handheld buttons | LEFT jog, RIGHT jog, PLAY/PAUSE |
| Cable options | 16-way IDC ribbon / DB15 / DB25 |

---

## Repository Structure

```
camera-slider/
├── README.md
│
├── camera-slider/              Original Uno fork — SH1106 OLED + EC11 encoder
│   ├── camera-slider.ino
│   ├── camera-slider-bom.xlsx
│   ├── camera-slider-wiring.pdf
│   └── camera-slider-wiring.txt
│
├── 12864-screen-mod/           Mini12864 fork — Arduino Uno
│   ├── 12864-screen-mod.ino
│   ├── 12864-bom.xlsx
│   ├── 12864-pin-reference.txt
│   ├── 12864-wiring.pdf
│   ├── 12864-wiring.txt
│   └── menu-simulator.jsx      Interactive menu simulator (React)
│
└── esp32-wifi-mod/             ESP32 fork — WiFi web UI + REST API
    ├── esp32-wifi-mod.ino
    ├── esp32-bom.xlsx
    ├── esp32-pin-reference.txt
    └── esp32-wiring.txt
```

---

## Firmware Forks

### `camera-slider` — Original Uno  (v1.2)

The original design. Arduino Uno with SH1106 1.3" OLED over I2C and an external EC11 rotary encoder module.

**Hardware:**
- Arduino Uno
- SH1106 1.3" OLED (I2C)
- External EC11 encoder + Back + Confirm buttons
- LEFT / RIGHT / E-Stop panel buttons
- TMC2209 stepper driver, NEMA 17
- 2× NC endstops (separate pins)

**Libraries:** U8g2, AccelStepper

---

### `12864-screen-mod` — Mini12864 Uno  (v1.2)

Replaces the OLED + external encoder with the BTT Mini12864 V2.0 — a single board containing a UC1701 display, EC11 encoder, and WS2812 RGB backlight. Communicates via SPI. TMC2209 EN pin tied permanently to GND.

**Hardware:**
- Arduino Uno
- BTT Mini12864 V2.0 (SPI, UC1701, WS2812 backlight)
- LEFT / RIGHT / PLAY-PAUSE panel buttons
- Encoder long press = E-Stop
- TMC2209 (EN tied to GND)
- 2× NC endstops (shared single pin, direction-aware)

**Libraries:** U8g2, AccelStepper, Adafruit NeoPixel

**Backlight states:**

| State | Colour |
|---|---|
| Idle / menu | Green |
| Running | Blue |
| Paused | Amber |
| Homing | White |
| E-Stop | Red |

**Includes** an interactive menu simulator (`menu-simulator.jsx`) — a React component that replicates the full display and button interface for testing menu navigation without hardware.

---

### `esp32-wifi-mod` — ESP32 WiFi  (v1.2)

Ports the Mini12864 design to an ESP32 DevKit (38-pin), adding WiFi control via a web browser UI and REST API. No IR remote required.

**Hardware:**
- ESP32 DevKit v1 (38-pin)
- BTT Mini12864 V2.0
- LEFT / RIGHT / PLAY-PAUSE panel buttons
- TMC2209 (EN tied to GND)
- 2× NC endstops (separate pins)

**Libraries:** U8g2, AccelStepper, Adafruit NeoPixel, ArduinoJson, ESPAsyncWebServer, AsyncTCP

**WiFi behaviour:**
1. On boot, attempts to connect to saved home network (10 second timeout)
2. If successful → Station mode, web UI at `http://<assigned IP>/`
3. If failed → Access Point mode (`CameraSlider` / `slider123`)
4. AP opens captive portal at `http://192.168.4.1/setup` to enter credentials
5. Credentials saved to NVS flash, survive reboots
6. Reset via: **Menu → WiFi Status → Reset WiFi Creds**

**Architecture:** FreeRTOS dual-core — Core 0 handles WiFi/web server, Core 1 handles motor/display (time-critical).

**REST API endpoints:**

| Method | Endpoint | Action |
|---|---|---|
| GET | `/api/status` | Position, state, program, loop, WiFi info |
| GET | `/api/programs` | All slot summaries + Full Loop settings |
| POST | `/api/home` | Trigger homing |
| POST | `/api/start` | Start selected program |
| POST | `/api/stop` | Stop program |
| POST | `/api/pause` | Pause program |
| POST | `/api/resume` | Resume paused program |
| POST | `/api/estop` | Emergency stop |
| POST | `/api/jog` | `{"dir":"left"\|"right","fast":0\|1}` |
| POST | `/api/jog/stop` | Stop jog motion |
| POST | `/api/select` | `{"slot":0–5}` (4=Full Loop, 5=To Endpoint) |
| POST | `/api/fullloop` | `{"speed":mm/s,"loops":count}` |
| POST | `/api/wifi` | `{"ssid":"...","pass":"..."}` — saves and restarts |

> ⚠️ GPIO 34, 35, 36, 39 on the ESP32 are input-only and have no internal pull-up. External 10kΩ resistors to 3.3V are required on these pins (ENC_CLK, ENC_DT, both endstops).

---

## Features

### Programs

- **Full Loop** — travels the full rail length at set speed, repeats N times (0 = continuous)
- **Move to Endpoint** — moves to the far end and stops
- **Slots 1–4** — user-defined programs with:
  - Set speed (mm/s)
  - Start position
  - Up to 10 stop points, each with an individual dwell timer (seconds)
  - End position
  - Loop count (0 = continuous)

### Motor Control

- Home all motors (moves to left endstop, zeros position)
- Jog left / right: tap = 1 mm slow step, hold = fast continuous
- Endstop reversal: if an endstop is hit mid-loop and loops remain, direction reverses automatically
- Controlled deceleration on pause and cancel
- Cancel returns carriage to program start point

### Persistence

| Data | Uno forks | ESP32 fork |
|---|---|---|
| Program slots 1–4 | EEPROM (301 / 1024 bytes) | NVS flash |
| Full Loop settings | EEPROM | NVS flash |
| WiFi credentials | — | NVS flash |

### Handheld Cable

Three cable options are documented in the wiring files:

| Option | Type | Pins | Notes |
|---|---|---|---|
| A | 16-way IDC ribbon | 16 | DIY crimp, compact |
| B | DB15 (gameport/data cable) | 15 | Pre-made, locking screws — verify all pins wired |
| C | DB25 (serial/parallel cable) | 25 | All pins guaranteed, 9 spares, likely in parts bin |

> ⚠️ Do not use standard VGA cables for Option B — VGA cables frequently omit pins. Use gameport extension cables or dedicated DB15 data cables instead.

Signal integrity components required at the PCB end of the cable:
- **470Ω resistor** in series on the NeoPixel data line (Mini12864 forks)
- **100nF ceramic capacitors** from ENC_CLK and ENC_DT to GND

---

## Getting Started

### Arduino Uno forks

1. Install libraries via Arduino Library Manager:
   - `U8g2` by Oliver Kraus
   - `AccelStepper` by Mike McCauley
   - `Adafruit NeoPixel` *(12864 fork only)*

2. Open the `.ino` file in Arduino IDE

3. Select board: **Arduino Uno**

4. Upload

5. On first boot, EEPROM is initialised with defaults automatically

### ESP32 fork

1. Add ESP32 board support to Arduino IDE:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```

2. Install additional libraries:
   - `ArduinoJson` (v6.x)
   - `ESPAsyncWebServer`
   - `AsyncTCP`

3. Select board: **ESP32 Dev Module**

4. Upload

5. Connect to WiFi network `CameraSlider` (password: `slider123`)

6. Browser opens captive portal — enter your home WiFi credentials

7. Device restarts and connects to your network

---

## Motor Tuning

Adjust these defines in the sketch to match your hardware:

```cpp
#define STEPS_PER_MM    80    // tune to your lead-screw pitch
                              // T8 leadscrew at 16 microsteps = 400
#define MAX_TRAVEL_MM  200    // physical rail length in mm
```

**Microstepping** is set via MS1/MS2 pads on the TMC2209 module. Default in firmware is 16 microsteps. Adjust `STEPS_PER_MM` if you change this.

---

## Version History

| Version | Changes |
|---|---|
| v1.0 | Initial release — Uno + SH1106 OLED + IR remote |
| v1.1 | Mini12864 fork, ESP32 fork, EEPROM persistence |
| v1.2 | IR remote removed, wired handheld design, `resumeProgram()` fix, ESP32 loop count bug fix, `/api/programs` endpoint, cable documentation |

---

## Licence

MIT — free to use, modify, and distribute. Attribution appreciated but not required.
