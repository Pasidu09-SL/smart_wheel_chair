#include <Servo.h>

// Pin setup (change to match your wiring)
const int TRIG_SWEEP = 2;
const int ECHO_SWEEP = 3;
const int TRIG_FIXED = 4;
const int ECHO_FIXED = 5;
const int SERVO_PIN = 9;

Servo scanServo;

long readDistanceCm(int trigPin, int echoPin) {
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);

	long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
	if (duration == 0) {
		return -1; // no echo
	}
	long distance = duration / 58; // microseconds to cm
	return distance;
}

void setup() {
	pinMode(TRIG_SWEEP, OUTPUT);
	pinMode(ECHO_SWEEP, INPUT);
	pinMode(TRIG_FIXED, OUTPUT);
	pinMode(ECHO_FIXED, INPUT);

	scanServo.attach(SERVO_PIN);
	scanServo.write(90);

	Serial.begin(9600);
	Serial.println("Two ultrasonic sensors + servo sweep test");
}

void loop() {
	// Sweep 0 -> 180 with the ultrasonic on the servo
	for (int angle = 0; angle <= 180; angle += 5) {
		scanServo.write(angle);
		delay(200);

		long sweepCm = readDistanceCm(TRIG_SWEEP, ECHO_SWEEP);
		long fixedCm = readDistanceCm(TRIG_FIXED, ECHO_FIXED);

		Serial.print("Angle: ");
		Serial.print(angle);
		Serial.print(" deg | Sweep: ");
		Serial.print(sweepCm);
		Serial.print(" cm | Fixed: ");
		Serial.print(fixedCm);
		Serial.println(" cm");
	}

	// Sweep back 180 -> 0
	for (int angle = 180; angle >= 0; angle -= 5) {
		scanServo.write(angle);
		delay(200);

		long sweepCm = readDistanceCm(TRIG_SWEEP, ECHO_SWEEP);
		long fixedCm = readDistanceCm(TRIG_FIXED, ECHO_FIXED);

		Serial.print("Angle: ");
		Serial.print(angle);
		Serial.print(" deg | Sweep: ");
		Serial.print(sweepCm);
		Serial.print(" cm | Fixed: ");
		Serial.print(fixedCm);
		Serial.println(" cm");
	}
}
