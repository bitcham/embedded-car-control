# Integrated Car Control System

## Overview

This project implements an Arduino-based RC car control system that can be operated via a web interface or joystick. The car features motor control with encoder feedback, compass-based navigation using CMPS14 sensor, and real-time status display on an LCD. An ESP8266 module provides Wi-Fi connectivity and hosts a web server for remote control.

## Features

- **Dual Control Modes**: Switch between ESP (web) control and joystick control using an external button
- **Precise Distance Control**: Drive specific distances (5cm, 20cm) using wheel encoder feedback
- **Compass Navigation**: Turn to specific angles (0-360°) using CMPS14 digital compass sensor
- **Find North**: Automatically orient the car to magnetic north
- **Real-time LCD Display**: Shows current mode, heading, encoder pulses, and distance traveled
- **Web Interface**: Control the car remotely through a responsive web page hosted on ESP8266
- **Heartbeat System**: Safety feature that stops the car if communication with ESP is lost

## Hardware Requirements

- Arduino Mega 2560
- ESP8266 (ESP-01 or NodeMCU)
- CMPS14 Digital Compass Module
- DC Motors with Encoders (2x)
- H-Bridge Motor Driver
- 20x4 LCD Display
- Analog Joystick
- Push Button (for mode switching)
- Power Supply
- Jumper Wires

## Pin Configuration

### Arduino Mega

| Component | Pin |
|-----------|-----|
| Motor L Direction | 7 |
| Motor R Direction | 8 |
| Motor L PWM | 9 |
| Motor R PWM | 10 |
| Encoder L | 3 (INT) |
| Encoder R | 2 (INT) |
| Joystick X | A8 |
| Joystick Y | A9 |
| Mode Button | 4 |
| LCD RS | 53 |
| LCD E | 51 |
| LCD D4 | 35 |
| LCD D5 | 34 |
| LCD D6 | 33 |
| LCD D7 | 32 |
| I2C SDA | 20 |
| I2C SCL | 21 |

### ESP8266

| Connection | Description |
|------------|-------------|
| TX | Connect to Arduino Serial1 RX |
| RX | Connect to Arduino Serial1 TX |

## Software Requirements

### Arduino IDE

### Libraries

- `Wire.h` (for I2C communication with compass)
- `LiquidCrystal.h` (for LCD display)
- `ESP8266WiFi.h` (for ESP8266 Wi-Fi)
- `ESP8266WebServer.h` (for web server)
- `FS.h` (for SPIFFS file system)

## Project Structure

```
embedded-car-control-/
├── README.md
├── car-control/
│   └── car-control.ino      # Arduino Mega main code
└── ESP-lite/
    ├── ESP-lite.ino         # ESP8266 web server code
    └── data/
        ├── index.html       # Web interface
        ├── style.css        # Styling
        ├── script.js        # Client-side logic
        └── favicon.png      # Browser icon
```

## Setup Instructions

### Hardware Setup

1. Connect the DC motors to the H-bridge motor driver
2. Connect the motor driver to Arduino Mega (pins 7, 8, 9, 10)
3. Connect wheel encoders to interrupt pins (2, 3)
4. Connect CMPS14 compass module to I2C (SDA: 20, SCL: 21)
5. Connect joystick to analog pins (A8, A9)
6. Connect mode button to pin 4
7. Connect 20x4 LCD to specified pins
8. Connect ESP8266 TX/RX to Arduino Serial1 (pins 18, 19)

### Software Setup

1. Open Arduino IDE
2. Install required libraries via Library Manager
3. Upload `car-control.ino` to Arduino Mega
4. For ESP8266:
   - Select appropriate ESP8266 board
   - Upload `ESP-lite.ino`
   - Upload web files to SPIFFS using **Tools > ESP8266 Sketch Data Upload**

## Configuration

### Wi-Fi Settings (ESP-lite.ino)

```cpp
const char* ssid = "Your-WiFi-SSID";
const char* password = "Your-WiFi-Password";
```

### Calibration Values (car-control.ino)

```cpp
const float PULSES_PER_CM = 84.0755f;    // Encoder pulses per cm
const uint8_t DRIVE_SPEED = 35;          // Default driving speed (0-100%)
const uint8_t TURN_SPEED = 30;           // Default turning speed (0-100%)
const float HEADING_TOLERANCE = 5.0f;    // Compass tolerance in degrees
```

## Usage

### Power On

1. Power on the Arduino and ESP8266
2. The system initializes and calibrates the compass
3. LCD displays "Final Assignment" followed by current status

### Mode Selection

- **Button Pressed (LOW)**: ESP mode - control via web interface
- **Button Released (HIGH)**: Joystick mode - manual control

### Web Interface

1. Connect to the same Wi-Fi network as ESP8266
2. Open browser and navigate to ESP8266's IP address (shown in Serial Monitor)
3. Use the web interface to:
   - Move forward/backward (5cm or 20cm)
   - Turn to specific angle using compass slider (-360° to 360°)
   - Find North button to orient to magnetic north

### Joystick Control

- **Y-axis**: Forward/Backward movement
- **X-axis**: Left/Right turning
- Movement speed is proportional to joystick deflection

### Serial Commands (ESP to Arduino)

| Command | Description |
|---------|-------------|
| `dist:X` | Drive X cm (positive=forward, negative=backward) |
| `deg:X` | Turn to X degrees (0-360) |
| `north` | Turn to face north (0°) |
| `ping` | Heartbeat check (responds with `pong`) |

### LCD Display

```
Line 1: [MODE] [HEADING]°[DIRECTION]
Line 2: C:[Last ESP Command] or X:[JoyX] Y:[JoyY]
Line 3: P L:[Left Pulses] R:[Right Pulses]
Line 4: D L:[Left cm] R:[Right cm]cm
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "COMPASS ERROR!" on LCD | Check I2C connections, verify CMPS14 address (0x60) |
| Car not responding to web commands | Check Serial1 baud rate (115200), verify ESP connection |
| Inaccurate distance | Calibrate `PULSES_PER_CM` value for your wheels |
| Turning not accurate | Calibrate `HEADING_TOLERANCE` or check compass mounting |