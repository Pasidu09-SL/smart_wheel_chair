// MPU-6050 raw test: prints accel, gyro, and temperature over Serial.
// Wiring (Arduino UNO/Nano): VCC->5V, GND->GND, SDA->A4, SCL->A5.
// For other boards, use the correct I2C pins.

#include <Wire.h>

static const uint8_t MPU_ADDR = 0x68; // AD0 low = 0x68, AD0 high = 0x69

void writeRegister(uint8_t reg, uint8_t value) {
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission(true);
}

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

void setup() {
	Serial.begin(115200);
	while (!Serial) {
		;
	}

	Wire.begin();
	Wire.setClock(400000);

	// Wake up the MPU-6050 (clear sleep bit).
	writeRegister(0x6B, 0x00);

	// Optional: set accel range to +/-2g (0x00) and gyro to +/-250 deg/s (0x00).
	writeRegister(0x1C, 0x00);
	writeRegister(0x1B, 0x00);

	Serial.println("MPU-6050 test started");
}

void loop() {
	uint8_t buffer[14] = {0};
	readRegisters(0x3B, 14, buffer);

	int16_t ax = combineBytes(buffer[0], buffer[1]);
	int16_t ay = combineBytes(buffer[2], buffer[3]);
	int16_t az = combineBytes(buffer[4], buffer[5]);
	int16_t temp = combineBytes(buffer[6], buffer[7]);
	int16_t gx = combineBytes(buffer[8], buffer[9]);
	int16_t gy = combineBytes(buffer[10], buffer[11]);
	int16_t gz = combineBytes(buffer[12], buffer[13]);

	float tempC = (temp / 340.0f) + 36.53f;

	Serial.print("AX:");
	Serial.print(ax);
	Serial.print(" AY:");
	Serial.print(ay);
	Serial.print(" AZ:");
	Serial.print(az);
	Serial.print(" | GX:");
	Serial.print(gx);
	Serial.print(" GY:");
	Serial.print(gy);
	Serial.print(" GZ:");
	Serial.print(gz);
	Serial.print(" | T:");
	Serial.println(tempC, 2);

	delay(200);
}
