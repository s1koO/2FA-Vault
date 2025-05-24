#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <TOTP.h>
#include "Base32.h"

#define IR_SENSOR_PIN PD0      // Arduino D0
#define MG996_SERVO_PIN PB1     // Arduino D9 (OCR1A)
#define GS21G_SERVO_PIN PB2    // Arduino D10 (OCR1B)
#define BUZZER_PIN PB3         // Arduino D11
#define BUZZER 11
#define RED_LED PB4            // Arduino D12
#define GREEN_LED PB5          // Arduino D13

#define MG996_start 18
#define MG996_end 95
#define GS21_start 10
#define GS21_end 100

// Program variables
bool is_locked = true;
bool door_opened = false;
bool tried_code = false;

String input_code = "";
const char* base32Key = "MNWGC5LENF2S443UMVTGC3Q";
uint8_t hmacKey[20];
unsigned long lastPeriod = 0;

TOTP* totp;
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Keypad variables
#define ROWS 4
#define COLS 3

char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {7, 2, 3, 5};
byte colPins[COLS] = {6, 8, 4};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);


void print_utc_time(DateTime now) {
	static unsigned long lastPrint = 0;
	if (millis() - lastPrint >= 5000) {
			Serial.print("UTC Time: ");
			Serial.print(now.year()); Serial.print("-");
			Serial.print(now.month()); Serial.print("-");
			Serial.print(now.day()); Serial.print(" ");
			Serial.print(now.hour()); Serial.print(":");
			Serial.print(now.minute()); Serial.print(":");
			Serial.println(now.second());

			lastPrint = millis();
	}
}

void setRTC(int year, int month, int day, int hour, int minute, int second) {
	DateTime customTime(year, month, day, hour, minute, second);
	rtc.adjust(customTime);

	Serial.print("RTC set to: ");
	Serial.print(year); Serial.print("-");
	Serial.print(month); Serial.print("-");
	Serial.print(day); Serial.print(" ");
	Serial.print(hour); Serial.print(":");
	Serial.print(minute); Serial.print(":");
	Serial.println(second);
}

char* get_totp_code() {
	DateTime now = rtc.now();
	char *code;

	// Debug function for RTC calibration
	print_utc_time(now);

	unsigned long unixTime = now.unixtime();
	unsigned long currentPeriod = unixTime / 30;

	if (currentPeriod != lastPeriod) {
		code = totp->getCode(unixTime);
		Serial.print("TOTP Code: ");
		Serial.println(code);
		lastPeriod = currentPeriod;
	}

	return code;
}

void playSuccessTone() {
	tone(BUZZER, 1000, 300);
	delay(300);
	noTone(BUZZER);
}

void playErrorTone() {
	tone(BUZZER, 300, 600);
	delay(600);
	noTone(BUZZER);
}

void setServoAngle(uint8_t pin, int angle) {
	int pulse = map(angle, 0, 180, 544, 2400);  // Use extended pulse range
	int ticks = pulse * 2; // Convert to 0.5 µs ticks for 16MHz/8 Timer1

	if (pin == MG996_SERVO_PIN) {
		OCR1A = ticks;
	} else if (pin == GS21G_SERVO_PIN) {
		OCR1B = ticks;
	}
}

void smoothServoMove(uint8_t pin, int fromAngle, int toAngle, int stepDelay = 15) {
	if (fromAngle == toAngle) {
		setServoAngle(pin, toAngle);
		return;
	}

	int stepDirection = (toAngle > fromAngle) ? 1 : -1;

	for (int angle = fromAngle; angle != toAngle; angle += stepDirection) {
		setServoAngle(pin, angle);
		delay(stepDelay);
	}

	// Ensure the final angle is set precisely
	setServoAngle(pin, toAngle);
}

void unlock_door() {
	door_opened = true;

	// GS21G (on D10 -> OCR1B)
	smoothServoMove(GS21G_SERVO_PIN, GS21_end, GS21_start, 5);	// from locked to unlocked

	delay(500);

	// MG996 (on D9 -> OCR1A)
	smoothServoMove(MG996_SERVO_PIN, MG996_start, MG996_end, 10);  // from closed to open

	delay(1000);
	Serial.println("\nDoor has been opened\n");
}

void lock_door() {
	door_opened = false;

	// Close door
	smoothServoMove(MG996_SERVO_PIN, MG996_end, MG996_start, 10);  // from open to closed
	delay(100);
	setServoAngle(GS21G_SERVO_PIN, MG996_start);  // reinforce final position

	delay(1000);

	// Lock door
	smoothServoMove(GS21G_SERVO_PIN, GS21_start, GS21_end, 5);  // from unlocked to locked

	delay(1000);
	Serial.println("\nDoor has been closed!\n");
}

void start_led(uint8_t pin) {
	PORTB |= (1 << pin);
}

void stop_led(uint8_t pin) {
	PORTB &= ~(1 << pin);
}

int is_obstacle_detected() {
	return !(PIND & (1 << IR_SENSOR_PIN));
}

void clear_lcd() {
	input_code = "";
	lcd.clear();
	lcd.setCursor(0, 0);
}

void setup() {
	Serial.begin(9600);

	// Set LEDs, buzzer and servos as output
	DDRB |= (1 << RED_LED) | (1 << GREEN_LED) | (1 << BUZZER_PIN)
			| (1 << MG996_SERVO_PIN) | (1 << GS21G_SERVO_PIN);

	// Set IR sensor as input with pull-up
	DDRD &= ~(1 << IR_SENSOR_PIN);      // Set as input
	PORTD |= (1 << IR_SENSOR_PIN);      // Enable pull-up resistor

	start_led(RED_LED);

	// Set PWM pins as outputs
	pinMode(MG996_SERVO_PIN, OUTPUT);
	pinMode(GS21G_SERVO_PIN, OUTPUT);

	// Configure Timer1 for Fast PWM, 20ms period (50Hz)
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Non-inverting PWM A & B
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);    // Prescaler 8
	ICR1 = 39999; // TOP = 20ms at 16MHz/8 => 0.5 µs per tick

	DDRB |= (1 << PB1) | (1 << PB2);

	lcd.init();
	lcd.backlight();
	lcd.clear();
	lcd.setCursor(0, 0);

	if (!rtc.begin()) {
		Serial.println("RTC not found!");
		while (1);
	}

	// Helper function to adjust the time on RTC module
	// Usually it takes 22s - 26s to set the time
	setRTC(2025, 5, 24, 14, 50, 0);

	int keyLength = base32decode(base32Key, hmacKey, sizeof(hmacKey));
	if (keyLength <= 0) {
		Serial.println("Failed to decode Base32 key.");
		while (1);
	}

	totp = new TOTP(hmacKey, keyLength);  // dynamically create TOTP

	// Adjust starting positions for motors
	setServoAngle(GS21G_SERVO_PIN, GS21_start);	// lock up
	delay(1000);

	smoothServoMove(MG996_SERVO_PIN, 60, MG996_start, 10);  // from halfway open to closed
	delay(100);
	setServoAngle(GS21G_SERVO_PIN, MG996_start);  // reinforce final position

	delay(1500);
	setServoAngle(GS21G_SERVO_PIN, GS21_end);	// lock down
}

void loop() {
	char *code_2fa = get_totp_code();

	char key = keypad.getKey();
	if (key) {
		if (key == '*') {
			// Backspace
			if (input_code.length() > 0) {
				input_code.remove(input_code.length() - 1);

				// Re-print code
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(input_code);
			}	
		} else if (key == '#') {
			tried_code = true;

			if (is_locked) {
				if (input_code == code_2fa) {
					// If the code is correct grant access to the vault
					is_locked = false;

					clear_lcd();
					lcd.print("Access granted!");
					lcd.setCursor(0, 1);
					lcd.print("Press # to lock!");
					
					stop_led(RED_LED);
					start_led(GREEN_LED);
					playSuccessTone();
				} else {
					clear_lcd();
					lcd.print("Wrong code!");
					playErrorTone();
				}
			} else {
				clear_lcd();

				// Lock vault
				is_locked = true;
				stop_led(GREEN_LED);
				start_led(RED_LED);
				lock_door();
			}
		} else {
			if (is_locked && tried_code) {
				tried_code = false;
				clear_lcd();
			}

			if (is_locked && input_code.length() < 6 && isDigit(key)) {
				input_code += key;
				lcd.setCursor(input_code.length() - 1, 0);
				lcd.print(key);
			}
		}
	}

	// If hand detected open the door
	if (!door_opened && !is_locked && is_obstacle_detected()) {
		unlock_door();
	}

	delay(100);
}
