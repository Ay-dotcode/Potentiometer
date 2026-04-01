# Potentiometer Controller

An Arduino-based controller interface built with [PlatformIO](https://platformio.org/) that reads joystick and button inputs and transmits control data over serial.

## Overview

This project runs on an Arduino Uno and acts as a controller for a robot/vehicle. It reads multiple analog joystick axes and digital buttons, processes them, and sends structured serial output to a host device.

## Hardware

- **Board:** Arduino Uno
- **Analog inputs (A0–A4):**
  - `A0` — Speed (throttle)
  - `A1` — Arm Z axis (right joystick X)
  - `A2` — Steering (right joystick Y)
  - `A3` — Arm X axis (left joystick X)
  - `A4` — Arm Y axis (left joystick Y)
- **Digital inputs (D2–D12, INPUT_PULLUP):**
  | Pin | Label | Function |
  |-----|-------|----------|
  | 2   | Gripper | Toggle gripper open/close |
  | 3   | Stop | Momentary stop |
  | 4   | L1 | Toggle Arm / Car mode |
  | 5   | R2 | Toggle R2 |
  | 6   | R1 | Lock/unlock toggle |
  | 7   | Arm Home | Momentary arm home |
  | 8   | R3 | Toggle R3 |
  | 9   | Auto Release | Momentary auto release |
  | 10  | Left | Toggle left |
  | 11  | Dumper | Toggle dumper (requires unlock) |
  | 12  | L2 | Cycle speed limit mode |

## Features

- **Dual mode:** Switch between **Arm** mode and **Car** mode using L1.
- **Speed limits:** Three modes cycled via L2 — Slow (50), Medium (100), High (255).
- **Lock system:** R1 toggles a lock; the dumper can only be operated when unlocked. The system auto-locks after the dumper closes.
- **Change detection:** Serial output is only sent when values change, reducing noise.
- **Joystick deadzone:** Center deadzone on all joystick axes to prevent drift.

## Serial Output

Data is sent at **115200 baud** in comma-separated key:value format whenever a value changes:

```
speed:<n>,steering:<n>,armX:<n>,armY:<n>,armZ:<n>,armReset:<0|1>,stop:<0|1>,gripper:<0|1>,autoRelease:<0|1>,left:<0|1>,dumper:<0|1>,R2:<0|1>,R3:<0|1>
```

Status messages are also printed on mode/state changes (e.g. `Lock: ON`, `Active: Arm`, `Mode:Slow`).

## Getting Started

### Prerequisites

- [PlatformIO IDE](https://platformio.org/install/ide) (VS Code extension recommended) or PlatformIO CLI.

### Build & Upload

```bash
# Install dependencies and build
pio run

# Upload to connected Arduino Uno
pio run --target upload

# Open serial monitor at 115200 baud
pio device monitor
```

## Project Structure

```
├── src/
│   └── main.cpp        # Main controller logic
├── include/            # Header files (currently unused)
├── lib/                # Project-specific libraries
├── test/               # Unit tests
└── platformio.ini      # PlatformIO configuration
```
