# Implementation Plan - Smart Wheelchair ESP32 Board Integration

This plan details the implementation of the ESP32 board firmware (`smart_wheel_chair_esp32_bord.ino`) for the **Wifi ESP32 Wroom 30 Pin Module (Model XX5R69)**. It integrates Blynk IoT control, WiFi connectivity, video stream compatibility, MPU6050 gyro checks, battery sensing, and dual ultrasonic safety brakes.

## User Review Required

> [!IMPORTANT]
> **ESP32 Core Libraries**
> * **Blynk Library**: The code requires the `Blynk` library. Ensure you have installed the Blynk library in your Arduino IDE.
> * **ESP32Servo Library**: The standard Arduino `Servo.h` does not support ESP32. We will use the `#include <ESP32Servo.h>` library for servo control on the ESP32.
> * **LEDC PWM driver**: ESP32 uses the LEDC hardware block for PWM speed control instead of AVR's `analogWrite()`. We will configure LEDC channels `0` and `1` for the left and right motors.

> [!WARNING]
> **MPU6050 Enabler Switch**
> The gyroscope enabler flag `#define USE_MPU6050 false` is set by default. If your MPU6050 is connected and stable on the ESP32 I2C pins (SDA pin 22, SCL pin 23), you can toggle this flag to `true` to enable tilt protection.

---

## Proposed Changes

### ESP32 Firmware

#### [NEW] [smart_wheel_chair_esp32_bord.ino](file:///Users/pasindu/Document%20-%20Not%20Cloud/smart_wheel_chair/smart_wheel_chair_esp32_bord/smart_wheel_chair_esp32_bord.ino)
Create a new sketch containing:
1. **WiFi & Blynk IoT Integration**:
   * Connects to local WiFi.
   * Links to Blynk App using your Auth Token: `UH1kH3VfahOMHcsuB1hBfplnAWnL9e2T`.
   * Maps virtual pins:
     - `V0` / `V1`: Virtual Joystick X / Y inputs.
     - `V2`: Speed factor limit (0 to 255).
     - `V12`: System ON/OFF power toggle.
     - `V3` (Front Distance), `V4` (Back Distance), `V5` (Tilt Warning), `V6` (Obstacle Warning), `V7` (Roll Angle), `V8` (Battery V), `V11` (System Status String).
2. **Servo Sweeping & Obstacle Sector Memory**:
   * Uses `<ESP32Servo.h>`.
   * Sweeps from 0° to 180° in non-blocking steps.
   * Maintains `frontReadings[7]` to prevent obstacle detection flickering.
3. **Proportional Motor Driver & Safety Brakes**:
   * Implements proportional arcade mixing using `ledcWrite` on pins 32 and 33.
   * Forward brake stops drive speed if `frontObstacle` is active.
   * Backward brake stops drive speed if `rearObstacle` is active.
4. **Diagnostic Reports**:
   * Prints reactive `Action: <state>` on the Serial Monitor (`115200` baud) on state change.
   * Sends telemetry updates to Blynk every 500ms.

---

## Pin Connection Reference Table (ESP32 Wroom 30-Pin)

| Component | Pin Function | ESP32 Pin | Notes |
|---|---|---|---|
| **Blynk Virtual** | Joystick X (Steer) | `V0` | Virtual Pin |
| **Blynk Virtual** | Joystick Y (Drive) | `V1` | Virtual Pin |
| **Blynk Virtual** | Max Speed | `V2` | Virtual Pin |
| **Blynk Virtual** | Power Switch | `V12` | Virtual Pin |
| **MPU6050 Gyro** | SDA | `22` | I2C Data |
| **MPU6050 Gyro** | SCL | `23` | I2C Clock |
| **Buzzer** | Signal | `15` | Digital Output |
| **Servo** | PWM Signal | `4` | ESP32Servo Output |
| **Sweep Ultrasonic (Front)** | Trig | `5` | Digital Output |
| **Sweep Ultrasonic (Front)** | Echo | `18` | Digital Input |
| **Fixed Ultrasonic (Rear)** | Trig | `19` | Digital Output |
| **Fixed Ultrasonic (Rear)** | Echo | `21` | Digital Input |
| **Battery Sensor** | Analog Input | `36` | ADC Input (VP Pin) |
| **Motor Driver** | ENA (Speed Left) | `32` | LEDC PWM Ch 0 |
| **Motor Driver** | IN1 (Dir Left 1) | `25` | Digital Output |
| **Motor Driver** | IN2 (Dir Left 2) | `26` | Digital Output |
| **Motor Driver** | ENB (Speed Right) | `33` | LEDC PWM Ch 1 |
| **Motor Driver** | IN3 (Dir Right 1) | `27` | Digital Output |
| **Motor Driver** | IN4 (Dir Right 2) | `14` | Digital Output |

---

## Verification Plan

### Automated/Build Check
- Compile the sketch in Arduino IDE choosing "ESP32 Dev Module" as the target board.

### Manual Verification
1. **Blynk Connection**:
   - Check that ESP32 connects to WiFi and Blynk status shows "Online" or "RUNNING" on `V11`.
2. **Virtual Driving Control**:
   - Move virtual joystick on Blynk app $\rightarrow$ verify motor directions and speeds.
3. **Brakes & Telemetry**:
   - Verify that obstacle detections write to virtual pins `V3`, `V4`, and `V6`.
   - Verify forward/backward drive locks when obstacles are close.
