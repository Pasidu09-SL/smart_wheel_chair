/*
 SMART ASSISTIVE WHEELCHAIR — FIXED v8
 ESP32 N4R4 | Blynk Only Control
*/

#define BLYNK_TEMPLATE_ID "TMPL6BfkQsEts"
#define BLYNK_TEMPLATE_NAME "SmartWheelchair"
#define BLYNK_AUTH_TOKEN "UH1kH3VfahOMHcsuB1hBfplnAWnL9e2T"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <MPU6050.h>
#include <math.h>

const char* ssid = "abc";
const char* password = "12345678";
const char* CAM_STREAM_URL = "http://172.20.10.9:81/stream";

// MOTOR PINS
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define ENA 32
#define ENB 33

// PWM CHANNELS
#define PWM_CHANNEL_ENA 0
#define PWM_CHANNEL_ENB 1
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// ULTRASONIC FRONT
#define TRIG_FRONT 5
#define ECHO_FRONT 18

// ULTRASONIC BACK
#define TRIG_BACK 19
#define ECHO_BACK 21

// SERVO
#define SERVO_PIN 4

// MPU6050
#define SDA_PIN 22
#define SCL_PIN 23

// BUZZER
#define BUZZER 15

// BATTERY
#define BATT_PIN 36
#define BATT_RATIO 3.128f

#define OBSTACLE_CM 20
#define TILT_LIMIT 45.0f
#define WIFI_TIMEOUT_MS 1500
#define BATT_LOW_V 6.4f
#define JOY_DEADZONE 25

Servo scanServo;
MPU6050 mpu;
BlynkTimer timer;

bool systemON = false;
int speedValue = 180;

int joyX = 0;
int joyY = 0;

bool mpuOK = false;
bool tiltActive = false;
bool obstacleActive = false;

int servoPos[] = {60, 90, 120};
int servoStep = 0;

long scanDist[] = {400,400,400};
long frontDist = 400;
long backDist = 400;

unsigned long lastWiFiOK = 0;

// ------------------------------------------------

BLYNK_CONNECTED() {
  Blynk.virtualWrite(V9, CAM_STREAM_URL);
  Blynk.virtualWrite(V11, systemON ? "RUNNING" : "STOPPED");
  Blynk.virtualWrite(V12, systemON ? 1 : 0);
  lastWiFiOK = millis();
}

BLYNK_WRITE(V0) { joyX = param.asInt(); }
BLYNK_WRITE(V1) { joyY = param.asInt(); }

BLYNK_WRITE(V2) {
  speedValue = constrain(param.asInt(),0,255);
}

BLYNK_WRITE(V12) {
  systemON = (param.asInt() == 1);

  if(systemON){
    beep(1,300,0);
    Blynk.virtualWrite(V11,"RUNNING");
  }else{
    stopMotors();
    joyX=0;
    joyY=0;
    beep(3,100,80);
    Blynk.virtualWrite(V11,"STOPPED");
  }
}

// ------------------------------------------------

void setup(){

  Serial.begin(115200);

  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  // PWM SETUP
  ledcSetup(PWM_CHANNEL_ENA,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(ENA,PWM_CHANNEL_ENA);

  ledcSetup(PWM_CHANNEL_ENB,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(ENB,PWM_CHANNEL_ENB);

  stopMotors();

  pinMode(TRIG_FRONT,OUTPUT);
  pinMode(ECHO_FRONT,INPUT);
  pinMode(TRIG_BACK,OUTPUT);
  pinMode(ECHO_BACK,INPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(90);
  delay(500);

  pinMode(BUZZER,OUTPUT);

  Wire.begin(SDA_PIN,SCL_PIN);
  mpu.initialize();

  if(mpu.testConnection()){
    mpuOK=true;
  }

  Blynk.begin(BLYNK_AUTH_TOKEN,ssid,password);

  timer.setInterval(250L,servoScanStep);
  timer.setInterval(500L,sendSensorData);

  beep(2,150,100);
}

// ------------------------------------------------

void loop(){

  Blynk.run();
  timer.run();

  if(WiFi.status()==WL_CONNECTED){
    lastWiFiOK=millis();
  }else{
    if(millis()-lastWiFiOK > WIFI_TIMEOUT_MS){
      stopMotors();
      return;
    }
  }

  if(!systemON){
    stopMotors();
    return;
  }

  if(tiltActive){
    stopMotors();
    return;
  }

  driveBlynk();
}

// ------------------------------------------------

void driveBlynk(){

  if(abs(joyX)<=JOY_DEADZONE && abs(joyY)<=JOY_DEADZONE){
    stopMotors();
    clearObstacle();
    return;
  }

  if(abs(joyY)>=abs(joyX)){

    if(joyY > JOY_DEADZONE){
      if(frontDist>OBSTACLE_CM){
        moveForward();
        clearObstacle();
      }else{
        stopMotors();
        triggerObstacle("Front");
      }
    }

    else if(joyY < -JOY_DEADZONE){
      if(backDist>OBSTACLE_CM){
        moveBackward();
        clearObstacle();
      }else{
        stopMotors();
        triggerObstacle("Back");
      }
    }

  }else{

    if(joyX > JOY_DEADZONE){
      turnRight();
      clearObstacle();
    }

    else if(joyX < -JOY_DEADZONE){
      turnLeft();
      clearObstacle();
    }
  }
}

// ------------------------------------------------

void servoScanStep(){

  scanServo.write(servoPos[servoStep]);

  long d = getDistance(TRIG_FRONT,ECHO_FRONT);

  if(d<=0 || d>400) d=400;

  scanDist[servoStep]=d;

  if(servoStep==2){
    frontDist=min(scanDist[0],min(scanDist[1],scanDist[2]));
    Blynk.virtualWrite(V3,frontDist);
  }

  servoStep++;
  if(servoStep>=3) servoStep=0;
}

// ------------------------------------------------

void sendSensorData(){

  backDist=getDistance(TRIG_BACK,ECHO_BACK);
  if(backDist<=0 || backDist>400) backDist=400;

  Blynk.virtualWrite(V4,backDist);

  if(mpuOK){

    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

    float roll = atan2((float)ay,(float)az)*180.0/PI;
    float pitch = atan2(-(float)ax,sqrt((float)ay*ay+(float)az*az))*180.0/PI;

    tiltActive = (abs(roll)>TILT_LIMIT || abs(pitch)>TILT_LIMIT);

    if(tiltActive){
      stopMotors();
      Blynk.virtualWrite(V5,255);
      beep(5,80,60);
    }else{
      Blynk.virtualWrite(V5,0);
    }

    Blynk.virtualWrite(V7,roll);
  }

  int adcRaw=analogRead(BATT_PIN);

  float battV=(adcRaw/4095.0)*3.3*BATT_RATIO;

  Blynk.virtualWrite(V8,battV);
}

// ------------------------------------------------

void triggerObstacle(const char* side){
  if(!obstacleActive){
    obstacleActive=true;
    beep(3,100,80);
    Blynk.virtualWrite(V6,255);
  }
}

void clearObstacle(){
  if(obstacleActive){
    obstacleActive=false;
    Blynk.virtualWrite(V6,0);
  }
}

// ------------------------------------------------

long getDistance(int trigPin,int echoPin){

  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin,LOW);

  long dur=pulseIn(echoPin,HIGH,20000);

  if(dur==0) return 400;

  return dur*0.034/2;
}

// ------------------------------------------------
// MOTOR FUNCTIONS
// ------------------------------------------------

void moveForward(){

  ledcWrite(PWM_CHANNEL_ENA,speedValue);
  ledcWrite(PWM_CHANNEL_ENB,speedValue);

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void moveBackward(){

  ledcWrite(PWM_CHANNEL_ENA,speedValue);
  ledcWrite(PWM_CHANNEL_ENB,speedValue);

  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void turnLeft(){

  ledcWrite(PWM_CHANNEL_ENA,speedValue);
  ledcWrite(PWM_CHANNEL_ENB,speedValue);

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void turnRight(){

  ledcWrite(PWM_CHANNEL_ENA,speedValue);
  ledcWrite(PWM_CHANNEL_ENB,speedValue);

  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void stopMotors(){

  ledcWrite(PWM_CHANNEL_ENA,0);
  ledcWrite(PWM_CHANNEL_ENB,0);

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

// ------------------------------------------------

void beep(int times,int onMs,int offMs){

  for(int i=0;i<times;i++){

    digitalWrite(BUZZER,HIGH);
    delay(onMs);

    digitalWrite(BUZZER,LOW);

    if(offMs>0) delay(offMs);
  }
}