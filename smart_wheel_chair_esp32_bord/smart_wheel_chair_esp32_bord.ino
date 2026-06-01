/*
  SMART ASSISTIVE WHEELCHAIR — ESP32 FIRMWARE (WROOM 30-PIN EDITION)
  
  Integrates:
  1. Blynk IoT Control (Virtual Joystick, speed settings, and system power switch)
  2. Physical Analog Joystick (overrides Blynk when moved out of center)
  3. Motor driver (L298N) using ESP32 LEDC PWM driver
  4. MPU6050 Gyroscope (tilt safety check via I2C)
  5. Dual Ultrasonic Sensors (fixed rear sensor & sweeping front sensor on servo)
  6. Onboard buzzer alerts
  7. Battery voltage monitoring
  8. WiFi Connection Loss safety auto-stop
  
  INPUT PRIORITY:
    Physical joystick moved? → Physical joystick controls motors.
    Physical joystick centered? → Blynk virtual joystick controls motors.
  
  This code is non-blocking, using timers to ensure continuous telemetry,
  safety checking, and Blynk synchronization.
*/

#define BLYNK_TEMPLATE_ID "TMPL6JjPFyclp"
#define BLYNK_TEMPLATE_NAME "SmartWheelChair"
#define BLYNK_AUTH_TOKEN "IwopCGgd4P5rr3MgC0PkTU_mApa8B-86"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ==========================================
// MPU6050 ENABLER SWITCH
// ==========================================
// Set this to false to disable Gyroscope balance checking and prevent I2C lockups.
// Set to true once the MPU6050 is connected to ESP32 pins 22 (SDA) and 23 (SCL).
#define USE_MPU6050 false

// ==========================================
// WIFI SETTINGS
// ==========================================
// Configured to match the ESP32-CAM access point settings.
const char* ssid = "Hostel_WiFi";
const char* password = "wifi@HostRUSL";

// ==========================================
// PIN CONNECTIONS (ESP32 WROOM 30-PIN)
// ==========================================

// Physical Joystick Pins
// NOTE: GPIO34 and GPIO35 are input-only ADC pins (no internal pull-up) — ideal for analog reads.
const int JOY_X_PIN = 34;  // Analog Input — Joystick X axis (Forward/Backward)
const int JOY_Y_PIN = 35;  // Analog Input — Joystick Y axis (Left/Right steer)
const int JOY_SW_PIN = 13; // Digital Input — Joystick push button (Active Low)

// MPU6050 I2C Pins
#define SDA_PIN 22
#define SCL_PIN 23
const uint8_t MPU_ADDR = 0x68;

// Buzzer Pin
const int BUZZER_PIN = 15;

// Servo Pin
const int SERVO_PIN = 4;

// Sweeping Front Ultrasonic Sensor
const int TRIG_SWEEP = 5;
const int ECHO_SWEEP = 18;

// Fixed Rear Ultrasonic Sensor
const int TRIG_REAR = 19;
const int ECHO_REAR = 21;

// Battery Sensor Pin
const int BATT_PIN = 36; // ADC1 CH0 (VP Pin)

// Motor Driver (L298N)
const int ENA = 32;      // Left Motor Speed (LEDC PWM)
const int IN1 = 25;      // Left Motor Dir 1
const int IN2 = 26;      // Left Motor Dir 2
const int ENB = 33;      // Right Motor Speed (LEDC PWM)
const int IN3 = 27;      // Right Motor Dir 1
const int IN4 = 14;      // Right Motor Dir 2

// LEDC PWM Settings
#define PWM_CHANNEL_ENA 0
#define PWM_CHANNEL_ENB 1
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// ==========================================
// SYSTEM CONSTANTS & CALIBRATIONS
// ==========================================

// Blynk virtual joystick deadzone (Blynk sends -255 to 255)
const int JOY_DEADZONE = 25;

// Physical joystick calibration — values measured from real hardware.
// UNO test (0-1023): X_mid=504, Y_mid=516 → scaled ×4 for ESP32 12-bit ADC (0-4095).
// Direction mapping from measurements:
//   X axis → Drive:  X=0 (forward) … X=4095 (backward)
//   Y axis → Steer:  Y=0 (right)   … Y=4095 (left)
const int JOY_PHYS_MID_X    = 2016; // 504  × 4  (measured center X)
const int JOY_PHYS_MID_Y    = 2064; // 516  × 4  (measured center Y)
const int JOY_PHYS_DEADZONE = 400;  // ±400 ADC counts (~±100 in UNO scale) = center dead zone

const int SAFE_DISTANCE_CM = 20;
const float TILT_LIMIT_DEG = 45.0;
const float BATT_RATIO = 3.128;
const unsigned long WIFI_TIMEOUT_MS = 1500;

// ==========================================
// STATE VARIABLES
// ==========================================
Servo scanServo;

// Telemetry state
long rearDistance = 400;             // Fixed rear sensor reading
long frontSweepDistance = 400;       // Sweep sensor reading (current angle)
long frontDistance = 400;            // Calculated minimum distance in front cone
long frontReadings[7] = {400, 400, 400, 400, 400, 400, 400}; // Front sectors
float rollAngle = 0.0;
float pitchAngle = 0.0;

// Safety overrides
bool tiltEmergency = false;
bool frontObstacle = false;
bool rearObstacle = false;
bool obstacleEmergency = false;

// Blynk control states
bool systemON = false;
int speedValue = 100; // Speed limit (0-255). 100 = ~40% power. Adjust via Blynk V2 slider.
int joyX = 0;         // Blynk virtual joystick horizontal (Steer)
int joyY = 0;         // Blynk virtual joystick vertical (Drive)

// Physical joystick state
bool physJoyActive = false; // true when physical joystick is out of center

// Non-blocking timers
unsigned long lastFixedSensorTime = 0;
const unsigned long fixedSensorInterval = 100; // Read fixed sensor every 100ms

unsigned long lastServoTime = 0;
const unsigned long servoInterval = 100;       // Move servo every 100ms
int servoAngle = 90;
int servoDirection = 10; // Scan increment (10 degrees for responsive sector updates)

unsigned long lastBuzzerTime = 0;
bool buzzerState = false;

unsigned long lastWiFiOK = 0;

// ==========================================
// BLYNK INPUT HANDLERS
// ==========================================

BLYNK_CONNECTED() {
  Blynk.virtualWrite(V11, systemON ? "RUNNING" : "STOPPED");
  Blynk.virtualWrite(V12, systemON ? 1 : 0);
  lastWiFiOK = millis();
}

BLYNK_WRITE(V0) {
  joyX = param.asInt(); // Virtual Joystick X (steer)
}

BLYNK_WRITE(V1) {
  joyY = param.asInt(); // Virtual Joystick Y (drive)
}

BLYNK_WRITE(V2) {
  speedValue = constrain(param.asInt(), 0, 255); // Max speed limit
}

BLYNK_WRITE(V12) {
  systemON = (param.asInt() == 1);
  if (systemON) {
    Blynk.virtualWrite(V11, "RUNNING");
  } else {
    stopMotors();
    joyX = 0;
    joyY = 0;
    Blynk.virtualWrite(V11, "STOPPED");
  }
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

// MPU6050 Register writing
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

// MPU6050 Register reading
void readRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    data[i] = Wire.read();
  }
}

int16_t combineBytes(uint8_t msb, uint8_t lsb) {
  return (int16_t)((msb << 8) | lsb);
}

// Read ultrasonic distance in centimeters
long readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout (~340cm max)
  if (duration == 0) {
    return 400; // Return max range if no echo
  }
  return duration / 58; // Convert to cm
}

// Motor Control functions (abstracting polarity offsets)
void setLeftMotor(bool forward, int pwm) {
  // ESP32 motor polarity: LOW is forward, HIGH is backward
  digitalWrite(IN1, forward ? LOW : HIGH); 
  digitalWrite(IN2, forward ? HIGH : LOW);
  ledcWrite(PWM_CHANNEL_ENA, pwm);
}

void setRightMotor(bool forward, int pwm) {
  // ESP32 motor polarity: HIGH is forward, LOW is backward
  digitalWrite(IN3, forward ? HIGH : LOW); 
  digitalWrite(IN4, forward ? LOW : HIGH);
  ledcWrite(PWM_CHANNEL_ENB, pwm);
}

void stopMotors() {
  ledcWrite(PWM_CHANNEL_ENA, 0);
  ledcWrite(PWM_CHANNEL_ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("\n--- Smart Wheelchair ESP32: Starting Boot ---"));

  // Physical Joystick Setup
  pinMode(JOY_SW_PIN, INPUT_PULLUP); // Button uses internal pull-up; pressed = LOW
  // GPIO34 & GPIO35 are input-only, no pinMode needed for analog reads

  // Buzzer Setup
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Ultrasonic Sensors Setup
  pinMode(TRIG_SWEEP, OUTPUT);
  pinMode(ECHO_SWEEP, INPUT);
  pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);

  // Servo Setup
  scanServo.attach(SERVO_PIN);
  scanServo.write(servoAngle);

  // Motor Driver Setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // LEDC PWM Driver Configuration
  ledcSetup(PWM_CHANNEL_ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_ENA);
  ledcSetup(PWM_CHANNEL_ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENB, PWM_CHANNEL_ENB);
  
  stopMotors();

  // Initialize I2C and MPU6050
#if USE_MPU6050
  Serial.println(F("Connecting to MPU6050 Gyro via I2C..."));
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  // Wake up the MPU-6050
  writeRegister(0x6B, 0x00); 
  writeRegister(0x1C, 0x00); // Set accelerometer range to +/-2g
  writeRegister(0x1B, 0x00); // Set gyroscope range to +/-250 deg/s
#else
  Serial.println(F("MPU6050 Gyro features are DISABLED in code."));
#endif

  // Connect to WiFi (non-blocking — waits max 10 seconds then continues)
  Serial.print(F("Connecting to WiFi: "));
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("\nWiFi connected. IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("\nWiFi not available — running in offline mode."));
  }

  // Configure Blynk without blocking (it will connect in the background via Blynk.run())
  Blynk.config(BLYNK_AUTH_TOKEN);
  lastWiFiOK = millis();

  // Signal successful boot
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println(F("System Initialized Successfully."));
}

// ==========================================
// LOOP
// ==========================================
void loop() {
  unsigned long currentMillis = millis();

  // 1. RUN BLYNK INTERNAL LOOPS
  Blynk.run();

  // 2. WIFI CONNECTION LOSS SAFETY CHECK
  if (WiFi.status() == WL_CONNECTED) {
    lastWiFiOK = currentMillis;
  } else {
    if (currentMillis - lastWiFiOK > WIFI_TIMEOUT_MS) {
      stopMotors();
      // Log WiFi loss
      static unsigned long lastWiFiLog = 0;
      if (currentMillis - lastWiFiLog >= 1000) {
        lastWiFiLog = currentMillis;
        Serial.println(F("[SAFETY] WiFi connection lost! Motors stopped."));
      }
      return;
    }
  }

  // 3. READ FIXED REAR ULTRASONIC SENSOR
  if (currentMillis - lastFixedSensorTime >= fixedSensorInterval) {
    lastFixedSensorTime = currentMillis;
    rearDistance = readDistanceCm(TRIG_REAR, ECHO_REAR);
  }

  // 4. NON-BLOCKING SERVO SWEEP (FRONT SENSOR)
  if (currentMillis - lastServoTime >= servoInterval) {
    lastServoTime = currentMillis;
    
    servoAngle += servoDirection;
    if (servoAngle >= 180) {
      servoAngle = 180;
      servoDirection = -10; // Reverse direction
    } else if (servoAngle <= 0) {
      servoAngle = 0;
      servoDirection = 10;  // Forward direction
    }
    
    scanServo.write(servoAngle);
    frontSweepDistance = readDistanceCm(TRIG_SWEEP, ECHO_SWEEP);

    // Update front sector readings if servo is looking forward (60° to 120°)
    if (servoAngle >= 60 && servoAngle <= 120) {
      int idx = map(servoAngle, 60, 120, 0, 6);
      if (idx >= 0 && idx < 7) {
        frontReadings[idx] = frontSweepDistance;
      }
    }
  }

  // Calculate the minimum front distance from our front sector readings
  frontDistance = 400;
  for (int i = 0; i < 7; i++) {
    if (frontReadings[i] < frontDistance && frontReadings[i] > 0) {
      frontDistance = frontReadings[i];
    }
  }

  // Update obstacle states
  frontObstacle = (frontDistance > 0 && frontDistance < SAFE_DISTANCE_CM);
  rearObstacle = (rearDistance > 0 && rearDistance < SAFE_DISTANCE_CM);

#if USE_MPU6050
  // 5. READ GYROSCOPE & CHECK BALANCE
  uint8_t buffer[14] = {0};
  readRegisters(0x3B, 14, buffer);
  int16_t ax = combineBytes(buffer[0], buffer[1]);
  int16_t ay = combineBytes(buffer[2], buffer[3]);
  int16_t az = combineBytes(buffer[4], buffer[5]);

  // Calculate Roll & Pitch angles (degrees)
  rollAngle = atan2((float)ay, (float)az) * 180.0 / PI;
  pitchAngle = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;

  // Check if wheelchair tilt exceeds safety threshold
  if (abs(rollAngle) > TILT_LIMIT_DEG || abs(pitchAngle) > TILT_LIMIT_DEG) {
    tiltEmergency = true;
  }

  // Physical joystick SWITCH = tilt emergency reset button (same as UNO board)
  if (digitalRead(JOY_SW_PIN) == LOW) {
    if (abs(rollAngle) <= TILT_LIMIT_DEG && abs(pitchAngle) <= TILT_LIMIT_DEG) {
      if (tiltEmergency) {
        tiltEmergency = false;
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println(F("Tilt Emergency Cleared via joystick button."));
      }
    }
  }

  // Tilt lockout action
  if (tiltEmergency) {
    stopMotors();
    // Solid alarm on tilt emergency
    digitalWrite(BUZZER_PIN, HIGH);
    
    // Log tilt error
    static unsigned long lastLog = 0;
    if (currentMillis - lastLog >= 1000) {
      lastLog = currentMillis;
      Serial.print(F("[EMERGENCY TILT] Lockout Active. Roll: "));
      Serial.print(rollAngle);
      Serial.print(F(" | Pitch: "));
      Serial.println(pitchAngle);
    }
    return; // Block execution while tilted
  }
#endif

  // 6. READ PHYSICAL JOYSTICK (always, so diagnostics work even when system is OFF)
  int physX = analogRead(JOY_X_PIN);
  int physY = analogRead(JOY_Y_PIN);

  int physDrive = 0;
  int physSteer = 0;

  // X axis → Drive: X=0 is FORWARD (+), X=4095 is BACKWARD (-)
  if (physX < JOY_PHYS_MID_X - JOY_PHYS_DEADZONE) {
    physDrive = map(physX, JOY_PHYS_MID_X - JOY_PHYS_DEADZONE, 0, 0, speedValue);
  } else if (physX > JOY_PHYS_MID_X + JOY_PHYS_DEADZONE) {
    physDrive = map(physX, JOY_PHYS_MID_X + JOY_PHYS_DEADZONE, 4095, 0, -speedValue);
  }

  // Y axis → Steer: Y=0 is RIGHT (+), Y=4095 is LEFT (-)
  if (physY < JOY_PHYS_MID_Y - JOY_PHYS_DEADZONE) {
    physSteer = map(physY, JOY_PHYS_MID_Y - JOY_PHYS_DEADZONE, 0, 0, speedValue);
  } else if (physY > JOY_PHYS_MID_Y + JOY_PHYS_DEADZONE) {
    physSteer = map(physY, JOY_PHYS_MID_Y + JOY_PHYS_DEADZONE, 4095, 0, -speedValue);
  }

  physJoyActive = (abs(physDrive) > 0 || abs(physSteer) > 0);

  // X=0 is forward, X=4095 is backward (inverted axis)
  bool drivingForward  = physJoyActive ? (physX < JOY_PHYS_MID_X - JOY_PHYS_DEADZONE) : (joyY >  JOY_DEADZONE);
  bool drivingBackward = physJoyActive ? (physX > JOY_PHYS_MID_X + JOY_PHYS_DEADZONE) : (joyY < -JOY_DEADZONE);

  // Compute final drive/steer for diagnostics (will be used by motor section too)
  int drive = 0;
  int steer = 0;
  if (physJoyActive) {
    drive = physDrive;
    steer = physSteer;
  } else {
    if (abs(joyY) > JOY_DEADZONE) drive = map(joyY, -255, 255, -speedValue, speedValue);
    if (abs(joyX) > JOY_DEADZONE) steer = map(joyX, -255, 255, -speedValue, speedValue);
  }

  // Compute current action string (used in diagnostics AND reactive log)
  String currentAction = "STOPPED";
  if (!systemON) {
    currentAction = "SYSTEM OFF";
  }
#if USE_MPU6050
  else if (tiltEmergency) {
    currentAction = "EMERGENCY TILT";
  }
#endif
  else if (drivingForward && frontObstacle) {
    currentAction = "BRAKE FRONT";
  } else if (drivingBackward && rearObstacle) {
    currentAction = "BRAKE REAR";
  } else {
    if (abs(drive) <= 0 && abs(steer) <= 0) {
      currentAction = "STOPPED";
    } else if (abs(drive) >= abs(steer)) {
      currentAction = (drive > 0) ? "FORWARD" : "BACKWARD";
    } else {
      currentAction = (steer > 0) ? "RIGHT" : "LEFT";
    }
  }

  static String lastAction = "";
  if (currentAction != lastAction) {
    lastAction = currentAction;
    Serial.print(physJoyActive ? F("[PHYS JOY] Action: ") : F("[BLYNK] Action: "));
    Serial.println(currentAction);
  }

  // 7. PRINT ALL SENSOR DIAGNOSTICS (Every 500ms — always runs, even when system is OFF)
  static unsigned long lastDiagTime = 0;
  if (currentMillis - lastDiagTime >= 500) {
    lastDiagTime = currentMillis;

    Serial.println(F("========================================"));

    // --- Joystick ---
    if (physJoyActive) {
      Serial.print(F("  [JOYSTICK] Source: PHYSICAL"));
      Serial.print(F("  | Raw X: ")); Serial.print(physX);
      Serial.print(F("  | Raw Y: ")); Serial.println(physY);
      Serial.print(F("  [JOYSTICK] Drive: ")); Serial.print(physDrive);
      Serial.print(F("  | Steer: ")); Serial.println(physSteer);
    } else {
      Serial.print(F("  [JOYSTICK] Source: BLYNK"));
      Serial.print(F("  | JoyX: ")); Serial.print(joyX);
      Serial.print(F("  | JoyY: ")); Serial.println(joyY);
    }
    Serial.print(F("  [JOYSTICK] SW Button: "));
    Serial.println(digitalRead(JOY_SW_PIN) == LOW ? F("PRESSED") : F("RELEASED"));

    // --- Ultrasonic Sensors ---
    Serial.print(F("  [ULTRASONIC] Front: ")); Serial.print(frontDistance); Serial.print(F(" cm"));
    Serial.print(F("  | Rear: ")); Serial.print(rearDistance); Serial.println(F(" cm"));
    Serial.print(F("  [ULTRASONIC] Sweep angle: ")); Serial.print(servoAngle);
    Serial.print(F(" deg | Sweep reading: ")); Serial.print(frontSweepDistance); Serial.println(F(" cm"));

    // --- Battery ---
    int adcRawLocal = analogRead(BATT_PIN);
    float battVLocal = (adcRawLocal / 4095.0) * 3.3 * BATT_RATIO;
    Serial.print(F("  [BATTERY] ADC Raw: ")); Serial.print(adcRawLocal);
    Serial.print(F("  | Voltage: ")); Serial.print(battVLocal, 2); Serial.println(F(" V"));

    // --- MPU6050 Gyro ---
#if USE_MPU6050
    Serial.print(F("  [GYRO] Roll: ")); Serial.print(rollAngle, 1);
    Serial.print(F(" deg  | Pitch: ")); Serial.print(pitchAngle, 1); Serial.println(F(" deg"));
    Serial.print(F("  [GYRO] Tilt Emergency: ")); Serial.println(tiltEmergency ? F("YES !!") : F("No"));
#else
    Serial.println(F("  [GYRO] MPU6050 DISABLED"));
#endif

    // --- Obstacle / Safety Flags ---
    Serial.print(F("  [SAFETY] Front Obstacle: ")); Serial.print(frontObstacle ? F("YES") : F("No"));
    Serial.print(F("  | Rear Obstacle: ")); Serial.print(rearObstacle ? F("YES") : F("No"));
    Serial.print(F("  | Buzzer Alarm: ")); Serial.println(obstacleEmergency ? F("YES") : F("No"));

    // --- Motor Output ---
    Serial.print(F("  [MOTORS] Drive: ")); Serial.print(drive);
    Serial.print(F("  | Steer: ")); Serial.print(steer);
    Serial.print(F("  | Action: ")); Serial.println(currentAction);

    // --- WiFi / Blynk / System ---
    Serial.print(F("  [WIFI] Status: "));
    Serial.print(WiFi.status() == WL_CONNECTED ? F("Connected") : F("Disconnected"));
    Serial.print(F("  | Blynk: ")); Serial.println(Blynk.connected() ? F("Connected") : F("Disconnected"));
    Serial.print(F("  [SYSTEM] Power: ")); Serial.println(systemON ? F("ON") : F("OFF (turn on via Blynk V12)"));

    Serial.println(F("========================================"));
  }

  // 8. CHECK POWER STATE
  // Physical joystick is a manual override — it works even when systemON = false.
  // Motors only stop if BOTH: Blynk switch is OFF and joystick is centered.
  if (!systemON && !physJoyActive) {
    stopMotors();
    return;
  }

  // 9. APPLY MOTOR STEERING (drive/steer already computed above)
  // SAFETY COLLISION BRAKE:
  if (drive > 0 && frontObstacle) {
    drive = 0;
  } else if (drive < 0 && rearObstacle) {
    drive = 0;
  }

  if ((drivingForward && frontObstacle) || (drivingBackward && rearObstacle)) {
    obstacleEmergency = true;
  } else {
    obstacleEmergency = false;
  }

  if (obstacleEmergency) {
    // Slow warning beeps for obstacle warning
    if (currentMillis - lastBuzzerTime >= 250) {
      lastBuzzerTime = currentMillis;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
  } else {
    // Make sure buzzer is turned off if no alarm is active
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Mix drive and steer values for differential drive
  int leftSpeed = drive + steer;
  int rightSpeed = drive - steer;

  // Constrain motor speeds within boundaries
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Apply command speed to Left Motor
  if (abs(leftSpeed) > 40) { // Small deadzone to prevent motor hum
    setLeftMotor(leftSpeed > 0, abs(leftSpeed));
  } else {
    ledcWrite(PWM_CHANNEL_ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Apply command speed to Right Motor
  if (abs(rightSpeed) > 40) {
    setRightMotor(rightSpeed > 0, abs(rightSpeed));
  } else {
    ledcWrite(PWM_CHANNEL_ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // 8. SEND TELEMETRY TO BLYNK (Every 500ms)
  static unsigned long lastBlynkTime = 0;
  if (currentMillis - lastBlynkTime >= 500) {
    lastBlynkTime = currentMillis;
    if (Blynk.connected()) {
      Blynk.virtualWrite(V3, frontDistance);
      Blynk.virtualWrite(V4, rearDistance);
      Blynk.virtualWrite(V5, tiltEmergency ? 255 : 0);
      Blynk.virtualWrite(V6, obstacleEmergency ? 255 : 0);
      Blynk.virtualWrite(V9, physJoyActive ? 1 : 0); // V9: Physical joystick active indicator
#if USE_MPU6050
      Blynk.virtualWrite(V7, rollAngle);
#endif
      // Read battery voltage and publish
      int adcRaw = analogRead(BATT_PIN);
      float battV = (adcRaw / 4095.0) * 3.3 * BATT_RATIO;
      Blynk.virtualWrite(V8, battV);
    }
  }

}
