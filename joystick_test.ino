// Joystick test: reads X, Y analog values and switch state
const int X_PIN = A0;
const int Y_PIN = A1;
const int SW_PIN = 7; // joystick button to GND

void setup() {
	Serial.begin(9600);
	pinMode(SW_PIN, INPUT_PULLUP);
	Serial.println("Joystick test: X, Y, and switch");
}

void loop() {
	int xVal = analogRead(X_PIN);
	int yVal = analogRead(Y_PIN);
	int swVal = digitalRead(SW_PIN);

	Serial.print("X: ");
	Serial.print(xVal);
	Serial.print(" | Y: ");
	Serial.print(yVal);
	Serial.print(" | SW: ");
	Serial.println(swVal == LOW ? "PRESSED" : "RELEASED");

	delay(100);
}
