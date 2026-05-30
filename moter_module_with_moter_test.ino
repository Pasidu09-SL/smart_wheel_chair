// Motor driver test for two DC motors (L298N/L293D style)
// Change pins to match your wiring
const int ENA = 5;  // PWM
const int IN1 = 8;
const int IN2 = 9;

const int ENB = 6;  // PWM
const int IN3 = 10;
const int IN4 = 11;

const int SPEED = 180; // 0-255

void setup() {
	pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(ENB, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

	stopMotors();
}

void loop() {
	forwardBoth();
	delay(2000);

	backwardBoth();
	delay(2000);

	spinLeft();
	delay(1500);

	spinRight();
	delay(1500);

	stopMotors();
	delay(1000);
}

void forwardBoth() {
	setMotorA(true, SPEED);
	setMotorB(true, SPEED);
}

void backwardBoth() {
	setMotorA(false, SPEED);
	setMotorB(false, SPEED);
}

void spinLeft() {
	setMotorA(false, SPEED);
	setMotorB(true, SPEED);
}

void spinRight() {
	setMotorA(true, SPEED);
	setMotorB(false, SPEED);
}

void stopMotors() {
	analogWrite(ENA, 0);
	analogWrite(ENB, 0);
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
}

void setMotorA(bool forward, int pwm) {
	digitalWrite(IN1, forward ? HIGH : LOW);
	digitalWrite(IN2, forward ? LOW : HIGH);
	analogWrite(ENA, pwm);
}

void setMotorB(bool forward, int pwm) {
	digitalWrite(IN3, forward ? HIGH : LOW);
	digitalWrite(IN4, forward ? LOW : HIGH);
	analogWrite(ENB, pwm);
}
