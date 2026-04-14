# Satellite Tracking Ground Station
### Automated Rotator Control System for a 3U CubeSat Mission


 HAMZA SKIRD
 YASSIN KHATER
 ABDERRAZZAK EL HAMCHIOUI


[![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%203-c51a4a?logo=raspberry-pi)](https://www.raspberrypi.org/)
[![Controller](https://img.shields.io/badge/controller-Arduino%20Mega-00979D?logo=arduino)](https://www.arduino.cc/)
[![Python](https://img.shields.io/badge/python-3.8%2B-3776AB?logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

A fully automated ground station that tracks satellites in real time and controls a **Yaesu G-5500 antenna rotator** via transistor switching. A Raspberry Pi 3 fetches live TLE orbital data, computes azimuth/elevation, and sends GOTO commands over serial to an Arduino Mega, which drives the G-5500 control box directly.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
  - [Components](#components)
  - [Wiring](#wiring)
  - [Calibration](#calibration)
- [Software](#software)
  - [Arduino Firmware](#arduino-firmware)
  - [Raspberry Pi Tracker](#raspberry-pi-tracker)
  - [Serial Test Utility](#serial-test-utility)
- [Installation](#installation)
- [Usage](#usage)
- [Serial Command Reference](#serial-command-reference)
- [Known Limitations](#known-limitations)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This project automates antenna pointing for a 3U CubeSat mission. The system:

1. Downloads fresh Two-Line Element (TLE) orbital data from **CelesTrak** (with local 2-hour cache)
2. Computes real-time **azimuth and elevation** for any satellite using [Skyfield](https://rhodesmill.org/skyfield/)
3. Sends `GOTO:az:el` commands over UART to an Arduino Mega
4. The Arduino closes NPN transistor switches that mimic the G-5500 hand controller buttons, moving the rotator to the correct pointing angle with closed-loop feedback from the rotator's analog position sensors

The system is passive until a satellite rises above the horizon — or can be set to always track (useful for geostationary targets or testing).

---

## System Architecture

```
┌─────────────────────────────────┐        USB / UART        ┌──────────────────────────┐
│        Raspberry Pi 3           │ ─────────────────────── ▶ │     Arduino Mega          │
│                                 │                           │                           │
│  tracker.py                     │   GOTO:182.45:34.12\n    │  g5500_transistor_         │
│  ├── CelesTrak TLE fetch        │ ─────────────────────── ▶ │  controller.ino           │
│  ├── Skyfield orbit propagation │                           │                           │
│  └── Serial GOTO output         │ ◀ ─────────────────────  │  ├── Serial command parser │
│                                 │   POS AZ:182.1 EL:34.5   │  ├── PID-like GOTO loop   │
└─────────────────────────────────┘                           │  ├── AZ/EL ADC feedback   │
                                                              │  └── NPN transistor drive │
                                                              └──────────┬────────────────┘
                                                                         │ D4-D7 (HIGH/LOW)
                                                              ┌──────────▼────────────────┐
                                                              │   4× NPN Transistors       │
                                                              │   (BC547 / 2N2222)        │
                                                              └──────────┬────────────────┘
                                                                         │ Pins 2,3,4,5
                                                              ┌──────────▼────────────────┐
                                                              │   Yaesu G-5500            │
                                                              │   Control Box             │
                                                              └───────────────────────────┘
```

---

## Hardware

### Components

| Part | Purpose | Notes |
|------|---------|-------|
| Raspberry Pi 3 Model B/B+ | Main computer — orbit propagation, TLE fetch, serial host | Any Pi with Python 3.8+ works |
| Arduino Mega 2560 | Real-time rotator controller | Uno works too if pins are available |
| Yaesu G-5500 | Az/El antenna rotator | Az: 0–450°, El: 0–180° |
| 4× NPN transistor (BC547 or 2N2222) | Switch G-5500 control lines | One per direction: L / R / U / D |
| 4× 1 kΩ resistor | Base current limiting for transistors | |
| 2× 10 kΩ resistor + 2× 8.2 kΩ resistor | Voltage divider on EL feedback *(optional — see note)* | Required to read EL above 135° |
| USB-A to USB-B cable | Pi → Arduino serial | Or use GPIO UART with level-shifter |

### Wiring

**Transistor control (Arduino → G-5500 control box)**

Each direction uses one NPN transistor. Arduino pin HIGH → transistor saturates → control pin shorted to G-5500 Pin 8 (GND) → rotator moves.

```
Arduino D4/D5/D6/D7
        │
       [1kΩ]
        │
      Base ─── NPN (BC547)
     Emitter ── GND (Arduino GND + G-5500 Pin 8 GND shared)
    Collector ── G-5500 control pin
```

| Arduino Pin | Direction | G-5500 Pin |
|-------------|-----------|------------|
| D4 | AZ Left (CCW) | Pin 4 |
| D5 | AZ Right (CW) | Pin 2 |
| D6 | EL Down | Pin 5 |
| D7 | EL Up | Pin 3 |
| A0 (input) | AZ feedback | Pin 6 |
| A1 (input) | EL feedback | Pin 1 |
| GND | Common ground | Pin 8 |

> ⚠️ **Important:** G-5500 Pin 1 (EL feedback) can reach 5.5 V, exceeding the Arduino's 5 V ADC limit. Without the voltage divider on A1, the firmware caps EL readback at 135°. To unlock full 0–180° EL, add a 10 kΩ + 8.2 kΩ divider between Pin 1 and A1 (and update `EL_VOLTS[]` accordingly).

### Calibration

The firmware uses piecewise linear interpolation between measured voltage–angle pairs. Default values are from actual hardware measurements:

**Azimuth (A0)**

| Voltage | Angle |
|---------|-------|
| 0.16 V | 0° |
| 1.14 V | 90° |
| 2.20 V | 180° |
| 3.35 V | 270° |
| 4.40 V | 360° |

**Elevation (A1) — without voltage divider**

| Voltage | Angle |
|---------|-------|
| 0.00 V | 0° |
| 0.75 V | 22.5° |
| 1.50 V | 45° |
| 2.20 V | 67° |
| 2.90 V | 90° |
| 3.70 V | 112° |
| 4.40 V | 135° *(ADC limit)* |

To re-calibrate, measure the feedback voltage at known angles and update `AZ_VOLTS[]` / `EL_VOLTS[]` in the firmware.

---

## Software

### Arduino Firmware

**File:** `g5500_transistor_controler.ino`

Runs entirely on the Arduino. Key features:

- **Serial command parser** at 9600 baud — receives `GOTO:az:el` and manual commands
- **Closed-loop GOTO** — reads ADC feedback every loop iteration and drives motors until within tolerance (`AZ_TOL = 3°`, `EL_TOL = 2°`)
- **Auto-scan mode** — sweeps azimuth left↔right on a timer (useful for blind search)
- **Position broadcast** — prints `POS AZ: xxx EL: xxx` every 500 ms for logging

### Raspberry Pi Tracker

**File:** `tracker.py`

Runs on the Raspberry Pi. Key features:

- Fetches TLE data from CelesTrak with a 2-hour local cache (`/tmp/tle_cache.json`) — survives network outages
- Uses [Skyfield](https://rhodesmill.org/skyfield/) for precise orbit propagation
- Accepts any NORAD catalog ID
- Configurable refresh interval, observer coordinates, serial port, and baud rate
- Optional **visible-only mode** — sends GOTO only when the satellite is above the horizon

### Serial Test Utility

**File:** `g5500_serial_controller.py`

A standalone interactive terminal for manual testing and debugging, with an optional automated test sequence that exercises every command.

---

## Installation

### Arduino

1. Open `g5500_transistor_controler.ino` in the **Arduino IDE** (2.x recommended)
2. Select board: **Arduino Mega 2560** (or Uno)
3. Select the correct COM port
4. Upload

### Raspberry Pi

```bash
# Update and install system dependencies
sudo apt update && sudo apt install -y python3-pip python3-venv

# Clone the repo
git clone https://github.com/your-username/satellite-ground-station.git
cd satellite-ground-station

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install pyserial skyfield requests

# Allow serial port access without sudo
sudo usermod -aG dialout $USER
# Log out and back in for group change to take effect
```

**Dependencies summary**

| Package | Version | Purpose |
|---------|---------|---------|
| `pyserial` | ≥ 3.5 | UART communication with Arduino |
| `skyfield` | ≥ 1.46 | Satellite orbit propagation |
| `requests` | ≥ 2.28 | TLE download from CelesTrak |

---

## Usage

### 1. Test serial communication first

```bash
# Interactive terminal (manual control)
python3 g5500_serial_controller.py --port /dev/ttyUSB0 --baud 9600

# Run automated test sequence (exercises all commands)
python3 g5500_serial_controller.py --port /dev/ttyUSB0 --baud 9600 --test
```

### 2. Start satellite tracking

```bash
source venv/bin/activate
python3 tracker.py
```

You will be prompted for:

```
NORAD ID (e.g. 25544 for ISS): 25544
Refresh interval in seconds (e.g. 5): 5

📍 Observer position:
  Latitude  (e.g. 48.8566 for Paris): 48.8566
  Longitude (e.g. 2.3522 for Paris): 2.3522

🔌 Arduino serial port:
  Port (e.g. COM3, /dev/ttyUSB0, /dev/ttyACM0): /dev/ttyUSB0
  Baudrate (e.g. 9600): 9600

Only send GOTO when satellite is visible? (y/N): y
```

Example output during a pass:

```
2024-03-15T14:23:07Z
  Satellite : lat= 51.234°, lon= -12.567°, alt= 421.3 km
  Observer  : az=127.45°, elev= 32.18°, dist=  874.2 km
  Status    : ✅ VISIBLE
  [Serial] Sent → GOTO:127.45:32.18
  [Serial] ACK  ← CMD: GOTO AZ=127.5 EL=32.2
------------------------------------------------------------
```

### Common NORAD IDs

| Satellite | NORAD ID |
|-----------|----------|
| ISS (ZARYA) | 25544 |
| NOAA 19 | 33591 |
| FUNCUBE-1 (AO-73) | 39444 |
| FOX-1A (AO-85) | 40967 |
| LILACSAT-2 | 40908 |

Find any satellite at [celestrak.org](https://celestrak.org/NORAD/elements/) or [n2yo.com](https://www.n2yo.com/).

---

## Serial Command Reference

All commands are sent as ASCII strings terminated with `\n` at **9600 baud**.

| Command | Action | Example |
|---------|--------|---------|
| `L` | Move azimuth left (CCW) | `L` |
| `R` | Move azimuth right (CW) | `R` |
| `U` | Move elevation up | `U` |
| `D` | Move elevation down | `D` |
| `S` | Stop all movement | `S` |
| `POS` | Print current AZ/EL position | `POS` |
| `SCAN` | Start auto az sweep left↔right | `SCAN` |
| `STOPSCAN` | Stop auto sweep | `STOPSCAN` |
| `GOTO:az:el` | Go to target position | `GOTO:180:45` |

GOTO tolerances: **±3° azimuth**, **±2° elevation**. The Arduino confirms arrival with `GOTO reached — AZ: xxx EL: xxx`.

---

## Known Limitations

- **EL capped at 135°** without the voltage divider on A1 — the G-5500's EL feedback exceeds 5 V above 135°. The firmware warns and clamps automatically.
- **No wrap-around logic** — the G-5500 AZ range is 0–450° mechanical. The firmware uses 0–360° only; passes crossing the 360°/0° boundary may cause a full rotation. A future improvement would track mechanical angle.
- **No TLE age validation** — if CelesTrak is unreachable for more than 2 hours, stale TLEs are used. Pointing accuracy degrades with TLE age (typically < 1 km/day for LEO).
- **Single-threaded tracker** — the refresh loop blocks on serial I/O. For high-cadence tracking (< 2 s), consider threading the serial read.

---

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

1. Fork the repo
2. Create a feature branch: `git checkout -b feature/my-improvement`
3. Commit your changes: `git commit -m 'Add voltage divider support for full EL range'`
4. Push and open a Pull Request



