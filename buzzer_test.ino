// Buzzer test: play a simple "We Wish You a Merry Christmas" melody
const int BUZZER_PIN = 8;

// Notes (Hz)
const int NOTE_C4 = 262;
const int NOTE_D4 = 294;
const int NOTE_E4 = 330;
const int NOTE_F4 = 349;
const int NOTE_G4 = 392;
const int NOTE_A4 = 440;
const int NOTE_B4 = 494;
const int NOTE_C5 = 523;

// Melody and durations (4 = quarter, 8 = eighth)
const int melody[] = {
	NOTE_G4, NOTE_C5, NOTE_C5, NOTE_D4, NOTE_C5, NOTE_B4, NOTE_A4,
	NOTE_A4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C5, NOTE_B4,
	NOTE_G4, NOTE_E4, NOTE_E4, NOTE_F4, NOTE_E4, NOTE_D4, NOTE_C5
};

const int noteDurations[] = {
	4, 4, 8, 8, 8, 8, 4,
	4, 4, 8, 8, 8, 8, 4,
	4, 4, 8, 8, 8, 8, 2
};

void setup() {
	pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
	int notes = sizeof(melody) / sizeof(melody[0]);
	for (int i = 0; i < notes; i++) {
		int duration = 1000 / noteDurations[i];
		tone(BUZZER_PIN, melody[i], duration);
		delay(duration * 1.3);
	}

	noTone(BUZZER_PIN);
	delay(2000);
}
