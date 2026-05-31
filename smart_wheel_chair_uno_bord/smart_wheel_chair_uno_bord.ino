/*
  SMART ASSISTIVE WHEELCHAIR — ARDUINO UNO FIRMWARE (ISOLATED SKETCH)
  
  Integrates:
  1. Joystick control (proportional arcade drive)
  2. Motor driver (L298N)
  3. MPU6050 Gyroscope (tilt safety check with joystick switch reset)
  4. Dual Ultrasonic Sensors (fixed rear sensor & sweeping front sensor on servo)
  5. Buzzer alerts
  
  This code runs entirely non-blocking, using timers to ensure continuous telemetry
  readings, joystick updates, and safety locks.
*/

#include <Wire.h>
#include <Servo.h>

// ==========================================
// MPU6050 ENABLER SWITCH
// ==========================================
// Set this to false to disable Gyroscope balance checking and prevent I2C lockups.
// Set to true once the MPU6050 is reconnected and wired correctly.
#define USE_MPU6050 false

// ==========================================
// PIN CONNECTIONS
// ==========================================

// Joystick
const int X_PIN = A0;   // Analog Input (X axis)
const int Y_PIN = A1;   // Analog Input (Y axis)
const int SW_PIN = 7;   // Joystick Button Input (Active Low)

// MPU6050 I2C Pins (Standard for UNO: SDA -> A4, SCL -> A5)
const uint8_t MPU_ADDR = 0x68;

// Buzzer
const int BUZZER_PIN = 12;

// Servo & Sweeping Front Ultrasonic Sensor
const int SERVO_PIN = 13;
const int TRIG_SWEEP = 2;
const int ECHO_SWEEP = 3;

// Fixed Rear Ultrasonic Sensor
const int TRIG_REAR = 4;
const int ECHO_REAR = A2; // Using A2 as a digital pin

// Motor Driver (L298N)
const int ENA = 5;      // Left Motor Speed (PWM)
const int IN1 = 8;      // Left Motor Dir 1
const int IN2 = 9;      // Left Motor Dir 2
const int ENB = 6;      // Right Motor Speed (PWM)
const int IN3 = 10;     // Right Motor Dir 1
const int IN4 = 11;     // Right Motor Dir 2

// ==========================================
// SYSTEM CONSTANTS & CALIBRATIONS
// ==========================================
const int JOY_DEADZONE_MIN = 450;
const int JOY_DEADZONE_MAX = 550;
const int JOY_MID_X = 498;
const int JOY_MID_Y = 505;

const int SAFE_DISTANCE_CM = 20;
const float TILT_LIMIT_DEG = 45.0;

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

// Non-blocking timers
unsigned long lastFixedSensorTime = 0;
const unsigned long fixedSensorInterval = 100; // Read fixed sensor every 100ms

unsigned long lastServoTime = 0;
const unsigned long servoInterval = 100;       // Move servo every 100ms
int servoAngle = 90;
int servoDirection = 10; // Scan increment (10 degrees for responsive sector updates)

unsigned long lastBuzzerTime = 0;
bool buzzerState = false;

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

// Motor A Control (Left Motor)
void setLeftMotor(bool forward, int pwm) {
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW : HIGH);
  analogWrite(ENA, pwm);
}

// Motor B Control (Right Motor)
void setRightMotor(bool forward, int pwm) {
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW : HIGH);
  analogWrite(ENB, pwm);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ==========================================
// ARDUINO SETUP
// ==========================================
void setup() {
  Serial.begin(9600); // 9600 baud rate matches your other Arduino Uno test codes
  Serial.println(F("\n--- Smart Wheelchair Uno: Starting Boot ---"));
  
  // Joystick Setup
  pinMode(SW_PIN, INPUT_PULLUP);

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
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  // Initialize I2C and MPU6050
#if USE_MPU6050
  Serial.println(F("Connecting to MPU6050 Gyro via I2C..."));
  Wire.begin();
  Wire.setClock(400000);
  
  // Wake up the MPU-6050
  writeRegister(0x6B, 0x00); 
  writeRegister(0x1C, 0x00); // Set accelerometer range to +/-2g
  writeRegister(0x1B, 0x00); // Set gyroscope range to +/-250 deg/s
#else
  Serial.println(F("MPU6050 Gyro features are DISABLED in code."));
#endif

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
// MAIN LOOP
// ==========================================
void loop() {
  unsigned long currentMillis = millis();

  // 1. READ FIXED REAR ULTRASONIC SENSOR
  if (currentMillis - lastFixedSensorTime >= fixedSensorInterval) {
    lastFixedSensorTime = currentMillis;
    rearDistance = readDistanceCm(TRIG_REAR, ECHO_REAR);
  }

  // 2. NON-BLOCKING SERVO SWEEP (FRONT SENSOR)
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
  // 3. READ GYROSCOPE & CHECK BALANCE
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

  // 4. READ JOYSTICK SWITCH (RESET BUTTON)
  int swVal = digitalRead(SW_PIN);
  if (swVal == LOW) { // Button is pressed
    // Reset tilt emergency ONLY if the wheelchair is back in balance
    if (abs(rollAngle) <= TILT_LIMIT_DEG && abs(pitchAngle) <= TILT_LIMIT_DEG) {
      if (tiltEmergency) {
        tiltEmergency = false;
        digitalWrite(BUZZER_PIN, LOW); // Silence the buzzer
        Serial.println(F("Tilt Emergency Cleared."));
      }
    }
  }

  // 5. PROCESS EMERGENCY STATUS & ALARM AUDIO (TILT)
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
    return; // Block motor steering while tilt lock is active
  }
#endif

  // 6. JOYSTICK MOTOR STEERING (ARCADE DRIVE MIXING)
  int xVal = analogRead(X_PIN);
  int yVal = analogRead(Y_PIN);

  int drive = 0; // Forward / Backward speed (-255 to 255)
  int steer = 0; // Turning adjustment (-255 to 255)

  // X Axis Mapping (Forward / Backward)
  if (xVal > JOY_DEADZONE_MAX) {
    // Map forward input to positive PWM speeds
    drive = map(xVal, JOY_DEADZONE_MAX, 1023, 0, 255);
  } else if (xVal < JOY_DEADZONE_MIN) {
    // Map backward input to negative PWM speeds
    drive = map(xVal, JOY_DEADZONE_MIN, 0, 0, -255);
  }

  // Y Axis Mapping (Left / Right)
  if (yVal > JOY_DEADZONE_MAX) {
    // Map right steer input
    steer = map(yVal, JOY_DEADZONE_MAX, 1023, 0, 255);
  } else if (yVal < JOY_DEADZONE_MIN) {
    // Map left steer input
    steer = map(yVal, JOY_DEADZONE_MIN, 0, 0, -255);
  }

  // SAFETY COLLISION BRAKE:
  // If moving forward and obstacle is too close, override drive speed to 0.
  // If moving backward and obstacle is too close, override drive speed to 0.
  if (drive > 0 && frontObstacle) {
    drive = 0;
  } else if (drive < 0 && rearObstacle) {
    drive = 0;
  }

  // Trigger buzzer warning only when trying to drive into an obstacle
  if ((xVal > JOY_DEADZONE_MAX && frontObstacle) || (xVal < JOY_DEADZONE_MIN && rearObstacle)) {
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
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Apply command speed to Right Motor
  if (abs(rightSpeed) > 40) {
    setRightMotor(rightSpeed > 0, abs(rightSpeed));
  } else {
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // 7. DETERMINE ACTION STATE & REACTIVE LOGGING
  String currentAction = "STOPPED";
  
#if USE_MPU6050
  if (tiltEmergency) {
    currentAction = "EMERGENCY TILT";
  } else
#endif
  if (xVal > JOY_DEADZONE_MAX && frontObstacle) {
    currentAction = "BRAKE FRONT";
  } else if (xVal < JOY_DEADZONE_MIN && rearObstacle) {
    currentAction = "BRAKE REAR";
  } else {
    // Determine direction based on joystick mapping relative to midpoint
    int driveIntention = 0;
    int steerIntention = 0;
    
    if (xVal > JOY_DEADZONE_MAX) driveIntention = xVal - JOY_MID_X;
    else if (xVal < JOY_DEADZONE_MIN) driveIntention = xVal - JOY_MID_X;
    
    if (yVal > JOY_DEADZONE_MAX) steerIntention = yVal - JOY_MID_Y;
    else if (yVal < JOY_DEADZONE_MIN) steerIntention = yVal - JOY_MID_Y;
    
    if (abs(driveIntention) < 40 && abs(steerIntention) < 40) {
      currentAction = "STOPPED";
    } else if (abs(driveIntention) >= abs(steerIntention)) {
      if (driveIntention > 0) {
        currentAction = "FORWARD";
      } else {
        currentAction = "BACKWARD";
      }
    } else {
      if (steerIntention > 0) {
        currentAction = "RIGHT";
      } else {
        currentAction = "LEFT";
      }
    }
  }

  static String lastAction = "";
  if (currentAction != lastAction) {
    lastAction = currentAction;
    Serial.print(F("Action: "));
    Serial.println(currentAction);
  }

  // 8. PRINT DIAGNOSTICS TO SERIAL (Every 500ms)
  static unsigned long lastDiagTime = 0;
  if (currentMillis - lastDiagTime >= 500) {
    lastDiagTime = currentMillis;
    Serial.print(F("JoyX: ")); Serial.print(xVal);
    Serial.print(F(" | JoyY: ")); Serial.print(yVal);
#if USE_MPU6050
    Serial.print(F(" | Roll: ")); Serial.print(rollAngle, 1);
    Serial.print(F(" | Pitch: ")); Serial.print(pitchAngle, 1);
#endif
    Serial.print(F(" | Rear: ")); Serial.print(rearDistance);
    Serial.print(F("cm | Front: ")); Serial.print(frontDistance);
    Serial.println(F("cm"));
  }
}
