//Works on board WEMOS D1 R32 and ESP32 DEV MODULE
#define VERSION "ESP32Motors_encoders_Rtos_OTA_Telnet_NVM_241118"
//#define PULSE_COUNT_VS_WIDTH_METHOD //Chose motors speed meters method comment / uncomment appriopriate
#include <Arduino.h>
#include <Preferences.h>  //For Non Volatile Memory to store preferences.
#define ESP32_RTOS        // Uncomment this line if you want to use the code with freertos only on the ESP32
// Has to be done before including "OTA.h"
//OTA & Telnet statements
#define OLED
#define TELNET
#if defined OLED
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TEXTFIRSTROW 4 
#include <spi.h>
#include <wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <fonts/Picopixel.h>        // Font Picopixel
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Adafruit_SSD1306 display(OLED_RESET);
#endif
//I2C SCL 22  
//I2C SDA 21
#include "OTA.h"
#include <credentials.h>
#define OTAPASSWORD "robot1"
//const char* otapassword = "robot1";
uint32_t entry;
//*********************
// Pin definitions
//Jtag pins 12 13 14 15
//I2C SCL 22  
//I2C SDA 21
const int leftMotorPWM = 26;            // GPIO for Left Motor PWM
const int rightMotorPWM = 33;           // GPIO for Right Motor PWM
const int leftMotorDIR = 2;             // GPIO for Left Motor DIR
const int rightMotorDIR = 27;           // GPIO for Right Motor DIR
const int leftPot = 34;                 // GPIO for Left Potentiometer
const int rightPot = 35;                // GPIO for Right Potentiometer
const int leftEncoder = 32;             // GPIO for Left Encoder
const int rightEncoder = 25;            // GPIO for Right Encoder
const int equalSpeed = 19;              // GPIO for right speed = left speed adjusted by left speed potentiometer
const int batteryPin = 36;              //ADC Port for battery voltage measurement
const float batteryDivider = 0.00371;   //Resistor divider for battery measurements
const float batteryLow = 7.0;			//Battery emergency levev
const float batteryMin = 6.0;           //Minimal battery voltage. Below motors stop
bool potOn = false; //If true target speed set by potentiometers instead of commands.
//For PID function
struct PID {
	float kp;  //Proportionality factor. To be adjusted
	float ki;  //Integration factor.  To be adjusted
	float kd;  //Derivation factor.  To be adjusted
};

Preferences preferences;
bool PIDon = true;              //If true speed PID regulation is on otherwise off
PID pidPreferences = { 1.0, 0.01, 0.01 };  // Default PID parameters
float integralLeft = 0;        // Accumulated error (integral)
float integralRight = 0;       // Accumulated error (integral)
float previousErrorLeft = 0;   // Last error (for derivative)
float previousErrorRight = 0;  // Last error (for derivative)
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int speedLeft;   //Current wheel speed in pulses / s
int speedRight;  //Current wheel speed in pulses / s
volatile long leftWay = 0;
volatile long rightWay = 0;
long currentLeftWay;
long currentRightWay;
const uint32_t minPulse = 5000ul;        //Minimal accepted pulse lebgth or minimal time between interrupts in uS
const uint32_t maxPulse = 200000ul;	   //Maximal zccepted pulse length in uS
// LEDC configuration
const int pwmFreq = 5000;    // PWM frequency in Hz
const int pwmResolution = 8;  // 8-bit resolution (0-255)

// Variables for motor speed and stabilization
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
int leftMotorPwm = 0;
int rightMotorPwm = 0;
#define FORWARD LOW
#define BACKWARD HIGH
#define PULSES 3 //Number of encoder pulses to calculate average
int leftMotorDirection = LOW;   //LOW means forward HI means backward
int rightMotorDirection = LOW;  //LOW means forward HI means backward
unsigned long lastTime;
//Maxspeed = 300 RPM Encoder 20 pulses per revolution (RISING or FALLING) 40 pulses (CHANGE) gives 100 or 200 pulses/s. 5000us or 10000us
//Minimal speed 30 RPM gives 10 or 20 pulses/s pulse width 50000us or 100000us 
const int maxSpeed = 110;              // Maximum speed in pulses per second

const unsigned long commPeriod = 500;  //Telnet communication period in miliseconds
bool printToTelnet = false;
bool printIntervalsToTelnet = false;
float batteryVoltage = 0;  //Stores battery voltage

TaskHandle_t MotorControlTaskHandle;  //Handle for MotorControlTask
TaskHandle_t CommunicationTaskHandle;  //Handle for MotorControlTask

#if defined PULSE_COUNT_VS_WIDTH_METHOD //Speed measure using pulse counting during set time.
const unsigned long countPeriod = 100000ul;  //Counting period in microseconds
//Interrupts handlers for optical encoders

volatile uint32_t lastInterruptTimeLeft = 0; // for debouncing last interrupt time
static void IRAM_ATTR handleStateChangeMotorLeft() {
	uint32_t currentTime = micros(); // Take actual time in uS
	if (currentTime - lastInterruptTimeLeft > minPulse) {
		lastInterruptTimeLeft = currentTime;
		if (leftMotorDirection == LOW)
			leftWay++;
		else
			leftWay--;
		leftEncoderCount++;
	}
}

volatile uint32_t lastInterruptTimeRight = 0; // for debouncing last interrupt time
static void IRAM_ATTR handleStateChangeMotorRight() {
	uint32_t currentTime = micros(); // Take actual time in uS
	if (currentTime - lastInterruptTimeRight > minPulse) {
		lastInterruptTimeRight = currentTime;
		if (rightMotorDirection == LOW)
			rightWay++;
		else
			rightWay--;
		rightEncoderCount++;
	}
}

void speedMeasure() {
	unsigned long currentTime = millis();
	unsigned long elapsedTime = currentTime - lastTime;
	if (elapsedTime >= countPeriod) {
		speedLeft = (leftEncoderCount * (1000.0 / elapsedTime));
		speedRight = (rightEncoderCount * (1000.0 / elapsedTime));
		lastTime = currentTime;
	}
}
#elif !defined PULSE_COUNT_VS_WIDTH_METHOD
//Measuring speeds by measuring time of PULSES
//#define PULSES 100
//#define PULSES 40
//#define PULSES 20
//#define PULSES 10
//#define PULSES 5
volatile uint32_t timestampsMotorLeft[PULSES + 1]; // Array to store timestamps for Motor Left
volatile uint32_t timestampsMotorRight[PULSES + 1]; // Array to store timestamps for Motor Right
volatile uint32_t timestampsMotorLeftStart; // Start pulse for Motor Left
volatile uint32_t timestampsMotorLeftEnd;  // End pulse for Motor Left
volatile uint32_t timestampsMotorRightStart; // Start pulse for Motor Left
volatile uint32_t timestampsMotorRightEnd;  // End pulse for Motor Left
volatile int countMotorLeft = 0;            // Counter for state changes for Motor Left
volatile int countMotorRight = 0;           // Counter for state changes for Motor Right

TaskHandle_t EncodersTaskHandle;
volatile bool leftSpeedMeasured = false;
volatile bool rightSpeedMeasured = false;
//volatile bool leftSpeedMeasure = true;
//volatile bool rightSpeedMeasure = false;
//const int measureCount = 100;

//ISR for Motor Left
static void IRAM_ATTR handleStateChangeMotorLeft() {
	if (leftMotorDirection == LOW)
		leftWay++;
	else
		leftWay--;
	if (countMotorLeft < PULSES) {
		leftSpeedMeasured = false;
		timestampsMotorLeft[countMotorLeft] = micros();
		countMotorLeft++;
	}
	else {
		countMotorLeft = 0;
		leftSpeedMeasured = true;
	}
}

//ISR for Motor Right
static void IRAM_ATTR handleStateChangeMotorRight() {
	if (rightMotorDirection == LOW)
		rightWay++;
	else
		rightWay--;
	if (countMotorRight < PULSES) {
		rightSpeedMeasured = false;
		timestampsMotorRight[countMotorRight] = micros();
		countMotorRight++;
	}
	else {
		countMotorRight = 0;
		rightSpeedMeasured = true;
	}
}

void speedMeasure() {
	const uint32_t falsePeriod = 1000000ul;
	//static bool measureSwitch = true;
	static int measuresCounterLeft = 0;
	static int measuresCounterRight = 0;
	//Left motor speed measure
	//if (measureSwitch) {
		//attachInterrupt(digitalPinToInterrupt(leftEncoder), handleStateChangeMotorLeft, RISING);
	if (leftSpeedMeasured) {
		//detachInterrupt(digitalPinToInterrupt(leftEncoder));
		unsigned long leftPulseAverageTime = 0;
		int acceptedCount = PULSES - 1;
		for (int i = 1; i < PULSES; i++) {
			uint32_t pulseTime = (timestampsMotorLeft[i] - timestampsMotorLeft[i - 1]); //Ignore to short or to long pulses
			if (pulseTime >= minPulse && pulseTime <= maxPulse)
				leftPulseAverageTime += pulseTime;
			else
				acceptedCount--;
		}
		if (acceptedCount > 0)
			leftPulseAverageTime /= acceptedCount; //Calculation of average pulse time
		else leftPulseAverageTime = falsePeriod; // If no accepted pulses assumes motor stopped.
		if (leftPulseAverageTime > 0 && leftPulseAverageTime < falsePeriod) {
			speedLeft = (int)(1000000ul / leftPulseAverageTime);
		}
		else
		{
			speedLeft = 0;
		}
		leftSpeedMeasured = false;
	}
	//else speedLeft = 0;
	//Right motor speed measure
	//attachInterrupt(digitalPinToInterrupt(rightEncoder), handleStateChangeMotorRight, RISING);
	if (rightSpeedMeasured)
	{
		//detachInterrupt(digitalPinToInterrupt(rightEncoder));
		unsigned long rightPulseAverageTime = 0;
		int acceptedCount = PULSES - 1;
		for (int i = 1; i < PULSES; i++) {
			uint32_t pulseTime = (timestampsMotorRight[i] - timestampsMotorRight[i - 1]); //Ignore to short or to long pulses
			if (pulseTime >= minPulse && pulseTime <= maxPulse)
				rightPulseAverageTime += pulseTime;
			else
				acceptedCount--;
		}
		if (acceptedCount > 0)
			rightPulseAverageTime /= acceptedCount;   //Calculation of average pulse time
		else rightPulseAverageTime = falsePeriod; // If no accepted pulses assumes motor stopped.
		if (rightPulseAverageTime > 0 && rightPulseAverageTime < falsePeriod) {
			speedRight = (int)(1000000ul / rightPulseAverageTime);
		}
		else
		{
			speedRight = 0;
		}
		rightSpeedMeasured = false;
	}
	//else speedRight = 0;
} //End of speedMeasure()
#endif


	// PID-based function to stabilize motor speed
int stabilizeSpeed(int targetSpeed, int currentSpeed, float* integral, float* previousError) {
	// Calculate the error
	float error = targetSpeed - currentSpeed;

	// Proportional term
	float proportional = pidPreferences.kp * error;

	// Integral term
	*integral += error;
	float integralTerm = pidPreferences.ki * *integral;

	// Derivative term
	float derivative = error - *previousError;
	float derivativeTerm = pidPreferences.kd * derivative;

	// Combine terms
	float correction = proportional + integralTerm + derivativeTerm;

	// Update previous error for next iteration
	*previousError = error;

	// Apply correction and constrain output
	return constrain(targetSpeed + correction, 0, maxSpeed);
}

void MotorControlTask(void* pvParameters) {
	const int batteryMeasures = 10;
	int batteryMeasuresCount = batteryMeasures;
	unsigned int batteryVolt = 0;
	for (;;) {
		speedMeasure();
		if (batteryMeasuresCount == batteryMeasures) {
			batteryVolt = 0;
		}
		if (batteryMeasuresCount > 0) {
			batteryVolt += analogRead(batteryPin);  //Battery voltage in volts
			batteryMeasuresCount--;
		}
		else {
			batteryMeasuresCount = batteryMeasures;
			batteryVoltage = ((float)batteryVolt / (float)batteryMeasures) * batteryDivider;
			batteryVolt = 0;

		}
		if (batteryVoltage < batteryMin) stopRobot();              // Stop robot if voltage is to low.

		// Read potentiometer values (0-4095 for 12-bit ADC)
		int leftPotValue = analogRead(leftPot);
		int rightPotValue = analogRead(rightPot);

		//     Map potentiometer values to target speeds in pulses / second
		if (potOn) {
			targetLeftSpeed = map(leftPotValue, 0, 4095, 0, 1000); //1000 pulses per second is maximum
			targetRightSpeed = map(rightPotValue, 0, 4095, 0, 1000); //1000 pulses per second is maximum
			if (digitalRead(equalSpeed) == LOW) targetRightSpeed = targetLeftSpeed; //Both motors run targetLeftSpeed if jumper is set
		}
		int targetLeftSpeedCorrected = targetLeftSpeed;
		int targetRightSpeedCorrected = targetRightSpeed;
		if (PIDon) {
			//Stabilize motor speeds using encoder feedback
			targetLeftSpeedCorrected = stabilizeSpeed(targetLeftSpeed, speedLeft, &integralLeft, &previousErrorLeft);
			targetRightSpeedCorrected = stabilizeSpeed(targetRightSpeed, speedRight, &integralRight, &previousErrorRight);
		}
		//Write directions to direction ports
		digitalWrite(rightMotorDIR, rightMotorDirection);
		digitalWrite(leftMotorDIR, leftMotorDirection);

		// Map target speeds values to motor speeds (0-255 or 255 - 0 for 8-bit PWM)
		if (leftMotorDirection == FORWARD)
			leftMotorPwm = map(targetLeftSpeedCorrected, 0, maxSpeed, 0, 255);
		else
			leftMotorPwm = map(targetLeftSpeedCorrected, 0, maxSpeed, 255, 0);
		if (rightMotorDirection == FORWARD)
			rightMotorPwm = map(targetRightSpeedCorrected, 0, maxSpeed, 0, 255);
		else
			rightMotorPwm = map(targetRightSpeedCorrected, 0, maxSpeed, 255, 0);

		// Set motor speeds
		ledcWrite(leftMotorPWM, leftMotorPwm);
		ledcWrite(rightMotorPWM, rightMotorPwm);
		leftEncoderCount = 0;
		rightEncoderCount = 0;
		vTaskDelay(10 / portTICK_PERIOD_MS);  //Task delay 5ms
	}
}  //End of MotorControlTask


#if defined TELNET
char* floatToString(float value, int przed_przecinkiem, int po_przecinku) {
	static char buffer[20]; // Statyczny bufor na wynik (20 znaków wystarczy w większości przypadków)
	dtostrf(value, przed_przecinkiem, po_przecinku, buffer);
	return buffer; // Zwracamy wskaźnik do stringa
}

//void displayTelnet() {
//	telnetClient.print(F("tgtLtSpeed: "));
//	telnetClient.print(targetLeftSpeed);
//	telnetClient.print(F("\tLtPwm: "));
//	//telnetClient.print(leftMotorPwm);
//	telnetClient.print(leftMotorDirection == FORWARD ? leftMotorPwm : 255 - leftMotorPwm);
//	telnetClient.print(F("\tLtDir: "));
//	telnetClient.print(leftMotorDirection);
//	telnetClient.print(F("\tSpeed Lt: "));
//	telnetClient.print(speedLeft);
//	telnetClient.print(F("\tWay Lt: "));
//	telnetClient.print(leftWay);
//	telnetClient.print(F("\t| tgtRtSpeed: "));
//	telnetClient.print(targetRightSpeed);
//	telnetClient.print(F("\tRtPwm: "));
//	//telnetClient.print(rightMotorPwm);
//	telnetClient.print(rightMotorDirection == FORWARD ? rightMotorPwm : 255 - rightMotorPwm);
//	telnetClient.print(F("\tRtDir: "));
//	telnetClient.print(rightMotorDirection);
//	telnetClient.print(F("\tSpeed Rt: "));
//	telnetClient.print(speedRight);
//	telnetClient.print(F("\tWay Rt: "));
//	telnetClient.print(rightWay);
//	telnetClient.print(F("\tkp= "));
//	telnetClient.print(floatToString(pidPreferences.kp,1,3));
//	telnetClient.print(F(" ki= "));
//	telnetClient.print(floatToString(pidPreferences.ki,1,3));
//	telnetClient.print(F(" kd= "));
//	telnetClient.print(floatToString(pidPreferences.kd,1,3));
//	//telnetClient.printf("\tkp= %1.3f3 ki= %1.3f3 kd= %1.3f", pidPreferences.kp, pidPreferences.ki, pidPreferences.kd);
//	//char* buffer[80];
//	//sprintf(buffer,"\tkp= %1.3f3 ki= %1.3f3 kd= %1.3f\r\n", pidPreferences.kp, pidPreferences.ki, pidPreferences.kd);
//	//telnetClient.print(buffer);
//	//telnetClient.printf(F("\tkp= %s ki= %s kd= %s"), floatToString(pidPreferences.kp,1,3), floatToString(pidPreferences.ki,1,3), floatToString(pidPreferences.kd,1,3));
//	telnetClient.print(F(F(" PID: ")));
//	telnetClient.print(PIDon);
//	telnetClient.print(F(" Pot: "));
//	telnetClient.print(potOn);
//	telnetClient.print(F(" Bat Volts: "));
//	telnetClient.print(batteryVoltage);
//	telnetClient.print(F("\r\n"));
//}

//bool displayPaused = false;  // Zmienna kontrolująca wyświetlanie tabeli

void displayStaticTable() {
	telnetClient.print(F("\033[2J"));  // Clears screen
	telnetClient.print(F("\033[H"));   // Move cursor to start

	// Górna ramka
	telnetClient.print(F("\033[32m+------------------------------------------------------+\r\n"));
	telnetClient.print(F("|                    SYSTEM STATUS TABLE               |\r\n"));
	telnetClient.print(F("+------------------------------------------------------+\r\n"));

	// Left Motor - ŻÓŁTY
	telnetClient.print(F("\033[33m"));  // Yellow color
	telnetClient.print(F("| Left Motor:   tgtSpeed:       Pwm:        Dir:       |\r\n"));
	telnetClient.print(F("|               Speed:          Way:                   |\r\n"));
	telnetClient.print(F("\033[0m"));   // Color reset

	// Podział
	telnetClient.print(F("\033[32m+------------------------------------------------------+\r\n"));

	// Right Motor - CYJAN
	telnetClient.print(F("\033[36m"));  // Cyyan color
	telnetClient.print(F("| Right Motor:  tgtSpeed:       Pwm:        Dir:       |\r\n"));
	telnetClient.print(F("|               Speed:          Way:                   |\r\n"));
	telnetClient.print(F("\033[0m"));   // Color reset

	// Podział
	telnetClient.print(F("\033[32m+------------------------------------------------------+\r\n"));

	// PID Values - ZIELONY
	telnetClient.print(F("\033[32m"));  // Green color
	telnetClient.print(F("| PID:          kp:             ki:         kd:        |\r\n"));
	telnetClient.print(F("+------------------------------------------------------+\r\n"));
	telnetClient.print(F("\033[33m"));  // Yellow color
	telnetClient.print(F("| Battery:      Vbat:                                  |\r\n"));
	telnetClient.print(F("\033[32m"));  // Green color
	telnetClient.print(F("+------------------------------------------------------+\r\n"));
	telnetClient.print(F("\033[0m"));   // Color reset
	telnetClient.flush();
}

void updateDynamicValues() {
	// Left Motor - values
	telnetClient.print(F("\033[4;28H"));  // Row 4, column 28
	telnetClient.printf("%3d", targetLeftSpeed);
	telnetClient.print(F("\033[4;39H"));  // Row 4, column 39
	telnetClient.printf("%3d", leftMotorDirection == FORWARD ? leftMotorPwm : 255 - leftMotorPwm);
	telnetClient.print(F("\033[4;51H"));  // Row 4, column 51
	telnetClient.print(leftMotorDirection == FORWARD ? "FWD" : "REV");

	telnetClient.print(F("\033[5;28H"));  // Row 5, column 28
	telnetClient.printf("%3d", speedLeft);
	telnetClient.print(F("\033[5;39H"));  // Row 5, column 39
	telnetClient.printf("%15d", leftWay);

	// Right Motor - values
	telnetClient.print(F("\033[7;28H"));  // Row 7, column 28
	telnetClient.printf("%3d", targetRightSpeed);
	telnetClient.print(F("\033[7;39H"));  // Row 7, column 39
	telnetClient.printf("%3d", rightMotorDirection == FORWARD ? rightMotorPwm : 255 - rightMotorPwm);
	telnetClient.print(F("\033[7;51H"));  // Row 7, column 51
	telnetClient.print(rightMotorDirection == FORWARD ? "FWD" : "REV");

	telnetClient.print(F("\033[8;28H"));  // Row 8, column 28
	telnetClient.printf("%3d", speedRight);
	telnetClient.print(F("\033[8;39H"));  // Row 8, column 39
	telnetClient.printf("%15d", rightWay);

	// PID - wartości
	telnetClient.print(F("\033[10;26H"));  // Row 10, column 26
	telnetClient.print(floatToString(pidPreferences.kp, 1, 3));
	telnetClient.print(F("\033[10;37H"));  // Row 10, column 37
	telnetClient.print(floatToString(pidPreferences.ki, 1, 3));
	telnetClient.print(F("\033[10;49H"));  // Row 10, column 49
	telnetClient.print(floatToString(pidPreferences.kd, 1, 3));
	//Battery values
	telnetClient.print(F("\033[12;26H"));   //Row 12, column 26
	telnetClient.print(batteryVoltage > batteryLow ? F("\033[32m") : F("\033[31m"));  //Green or red colour
	telnetClient.print(floatToString(batteryVoltage, 1, 2));
	telnetClient.print(F(" V \033[32m"));
	telnetClient.print(F("\033[0m"));   // Reset koloru
	telnetClient.flush();

	// Przesuń kursor poniżej tabeli
	telnetClient.print(F("\033[14;1H"));  // Rowz 14, column 1
	telnetClient.print(F("> "));          // Command cursor
	telnetClient.flush();
}

//void loop() {
//	if (telnetClient) {
//		static bool tableDrawn = false;
//
//		if (!tableDrawn) {
//			displayStaticTable();  // Wyświetl statyczną tabelę raz
//			tableDrawn = true;
//		}
//
//		updateDynamicValues();  // Aktualizuj tylko wartości
//		vTaskDelay(pdMS_TO_TICKS(500));  // Odświeżanie co 500 ms
//	}
//}

//void displayTelnet() {
//	if (!printToTelnet) return;  // Nie wyświetlaj tabeli, jeśli wprowadzana jest komenda
//
//	telnetClient.print("\033[H");     // Przesunięcie kursora na początek ekranu
//	telnetClient.print("\033[2J");    // Czyści cały ekran
//
//	// Górna ramka
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//	telnetClient.print(F("|                  SYSTEM STATUS TABLE                      |\r\n"));
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//
//	// Left Motor - ŻÓŁTY
//	telnetClient.print("\033[33m");  // Żółty kolor
//	telnetClient.print(F("| Left Motor:   tgtSpeed: "));
//	telnetClient.print(targetLeftSpeed);
//	telnetClient.print(F("   Pwm: "));
//	telnetClient.print(leftMotorDirection == FORWARD ? leftMotorPwm : 255 - leftMotorPwm);
//	telnetClient.print(F("   Dir: "));
//	telnetClient.print(leftMotorDirection == FORWARD ? "FWD" : "REV");
//	telnetClient.print(F("             |\r\n"));
//	telnetClient.print(F("|               Speed: "));
//	telnetClient.print(speedLeft);
//	telnetClient.print(F("    Way: "));
//	telnetClient.print(leftWay);
//	telnetClient.print(F("                          |\r\n"));
//	telnetClient.print("\033[0m");  // Reset koloru
//
//	// Podział
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//
//	// Right Motor - CYJAN
//	telnetClient.print("\033[36m");  // Cyjan kolor
//	telnetClient.print(F("| Right Motor:  tgtSpeed: "));
//	telnetClient.print(targetRightSpeed);
//	telnetClient.print(F("   Pwm: "));
//	telnetClient.print(rightMotorDirection == FORWARD ? rightMotorPwm : 255 - rightMotorPwm);
//	telnetClient.print(F("   Dir: "));
//	telnetClient.print(rightMotorDirection == FORWARD ? "FWD" : "REV");
//	telnetClient.print(F("             |\r\n"));
//	telnetClient.print(F("|               Speed: "));
//	telnetClient.print(speedRight);
//	telnetClient.print(F("    Way: "));
//	telnetClient.print(rightWay);
//	telnetClient.print(F("                          |\r\n"));
//	telnetClient.print("\033[0m");  // Reset koloru
//
//	// Podział
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//
//	// PID Values - ZIELONY
//	telnetClient.print("\033[32m");  // Zielony kolor
//	telnetClient.print(F("| PID: "));
//	telnetClient.print(PIDon ? "ON" : "OFF");
//	telnetClient.print(F("   kp: "));
//	telnetClient.print(floatToString(pidPreferences.kp, 1, 3));
//	telnetClient.print(F("   ki: "));
//	telnetClient.print(floatToString(pidPreferences.ki, 1, 3));
//	telnetClient.print(F("   kd: "));
//	telnetClient.print(floatToString(pidPreferences.kd, 1, 3));
//	telnetClient.print(F("               |\r\n"));
//	telnetClient.print("\033[0m");  // Reset koloru
//
//	// Dolna ramka
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//
//	telnetClient.flush();  // Wymuszenie wysłania bufora Telnet
//}

//void handleTelnetInput() {
//	if (telnetClient.available()) {
//		displayPaused = true;  // Zatrzymaj wyświetlanie tabeli
//		telnetClient.print(F("\r\n> "));  // Wskaźnik do wpisywania komendy
//
//		String command = telnetClient.readStringUntil('\n');  // Pobranie komendy
//		command.trim();  // Usuń białe znaki
//
//		if (command == "M") {
//			telnetClient.print(F("Menu:\r\n"));
//			telnetClient.print(F("[F]ORWARD <speed> <distance>\r\n"));
//			telnetClient.print(F("[B]ACKWARD <speed> <distance>\r\n"));
//			telnetClient.print(F("[S]TOP\r\n"));
//			telnetClient.print(F("[D]ISPLAY STATUS\r\n"));
//		}
//		else if (command == "S") {
//			telnetClient.print(F("Robot Stopped\r\n"));
//		}
//		else {
//			telnetClient.print(F("Unknown command\r\n"));
//		}
//
//		telnetClient.flush();
//		displayPaused = false;  // Wznów wyświetlanie tabeli
//	}
//}

//void loop() {
//	if (telnetClient) {
//		displayTelnet();  // Wyświetl tabelę (jeśli nie jest wstrzymana)
//		handleTelnetInput();  // Obsługa wpisywanych komend
//		vTaskDelay(pdMS_TO_TICKS(500));  // Odświeżaj co 500 ms
//	}
//}


//void displayTelnet() {
//	//telnetClient.print("\033c");      // Reset terminala
//	telnetClient.print("\033[H");     // Ustawienie kursora na początek
//
//	// Górna ramka
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//	telnetClient.print(F("|                  SYSTEM STATUS TABLE                     |\r\n"));
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//
//	// Left Motor - ŻÓŁTY
//	telnetClient.print("\033[33m");  // Żółty kolor
//	telnetClient.print(F("| Left Motor:   tgtSpeed: "));
//	telnetClient.print(targetLeftSpeed);
//	telnetClient.print(F("   Pwm: "));
//	telnetClient.print(leftMotorDirection == FORWARD ? leftMotorPwm : 255 - leftMotorPwm);
//	telnetClient.print(F("   Dir: "));
//	telnetClient.print(leftMotorDirection == FORWARD ? "FWD" : "REV");
//	telnetClient.print(F("   |\r\n"));
//	telnetClient.print(F("|               Speed: "));
//	telnetClient.print(speedLeft);
//	telnetClient.print(F("    Way: "));
//	telnetClient.print(leftWay);
//	telnetClient.print(F("                              |\r\n"));
//	telnetClient.print("\033[0m");  // Reset koloru
//
//	// Podział
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//
//	// Right Motor - CYJAN
//	telnetClient.print("\033[36m");  // Cyjan kolor
//	telnetClient.print(F("| Right Motor:  tgtSpeed: "));
//	telnetClient.print(targetRightSpeed);
//	telnetClient.print(F("   Pwm: "));
//	telnetClient.print(rightMotorDirection == FORWARD ? rightMotorPwm : 255 - rightMotorPwm);
//	telnetClient.print(F("   Dir: "));
//	telnetClient.print(rightMotorDirection == FORWARD ? "FWD" : "REV");
//	telnetClient.print(F("   |\r\n"));
//	telnetClient.print(F("|               Speed: "));
//	telnetClient.print(speedRight);
//	telnetClient.print(F("    Way: "));
//	telnetClient.print(rightWay);
//	telnetClient.print(F("                              |\r\n"));
//	telnetClient.print("\033[0m");  // Reset koloru
//
//	// Podział
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//
//	// PID Values - ZIELONY
//	telnetClient.print("\033[32m");  // Zielony kolor
//	telnetClient.print(F("| PID: "));
//	telnetClient.print(PIDon ? "ON" : "OFF");
//	telnetClient.print(F("   kp: "));
//	telnetClient.print(floatToString(pidPreferences.kp, 1, 3));
//	telnetClient.print(F("   ki: "));
//	telnetClient.print(floatToString(pidPreferences.ki, 1, 3));
//	telnetClient.print(F("   kd: "));
//	telnetClient.print(floatToString(pidPreferences.kd, 1, 3));
//	telnetClient.print(F("                  |\r\n"));
//	telnetClient.print("\033[0m");  // Reset koloru
//
//	// Dolna ramka
//	telnetClient.print(F("+-----------------------------------------------------------+\r\n"));
//
//	// Menu
//	telnetClient.print(F("\r\n"));
//	telnetClient.print(F("Robot Commands:\r\n"));
//	telnetClient.print(F("[F]ORWARD <speed> <distance>      [B]ACKWARD <speed> <distance>\r\n"));
//	telnetClient.print(F("[S]TOP                            [D]ISPLAY STATUS\r\n"));
//	telnetClient.print(F("[E]NCODERS                        [M]ENU - print this menu\r\n"));
//	telnetClient.print(F("[P]ID <kp> <ki> <kd>              [P]OT <0/1> - Potentiometer Control\r\n"));
//	telnetClient.print(F("[P]I <0/1> - PID regulation\r\n"));
//	telnetClient.print(F("\r\n> "));  // Wskaźnik do wpisania komendy
//	telnetClient.flush();  // Wymuszenie wysłania danych
//}
//

#else
void displaySerial() {
	Serial.print(F("tgtLtSpeed: "));
	Serial.print(targetLeftSpeed);
	Serial.print(F("\tLtPwm: "));
	Serial.print(leftMotorPwm);
	Serial.print(F("\tLtDir: "));
	Serial.print(leftMotorDirection);
	Serial.print(F("\tSpeed Lt: "));
	Serial.print(speedLeft);
	Serial.print(F("\tWay Lt: "));
	Serial.print(leftWay);
	Serial.print(F("\t| tgtRtSpeed: "));
	Serial.print(targetRightSpeed);
	Serial.print(F("\tRtPwm: "));
	Serial.print(rightMotorPwm);
	Serial.print(F("\tRtDir: "));
	Serial.print(rightMotorDirection);
	Serial.print(F("\tSpeed Rt: "));
	Serial.print(speedRight);
	Serial.print(F("\tWay Rt: "));
	Serial.print(rightWay);
	Serial.printf(F("\tkp= %f ki= %f kd= %f"), pidPreferences.kp, pidPreferences.ki, pidPreferences.kd);
	Serial.print(F("\tBat Volts: "));
	Serial.print(batteryVoltage);
	Serial.print(F("\r\n"));
}
#endif

void CommunicationTask(void* pvParameters) {
	for (;;) {
		if (currentLeftWay == leftWay) speedLeft = 0;
		if (currentRightWay == rightWay) speedRight = 0;
#if defined TELNET
		static bool tableDrawn;
		// Accept a new client
		if (telnetServer.hasClient()) {
			if (!telnetClient || !telnetClient.connected()) {
				if (telnetClient) telnetClient.stop();    // Disconnect previous client
				telnetClient = telnetServer.available();  // Accept new client
				Serial.println("New Telnet client connected");
				telnetClient.print("\033c");      // Reset terminala
				printMenu();
				telnetClient.flush(); //Prevents false menu command after connect.
				printToTelnet = false;
				tableDrawn = false;
			}
			else {
				telnetServer.available().stop();  // Reject additional clients
			}
		}
		orders();
		if (printToTelnet) {
			if (!tableDrawn) {
				displayStaticTable();  // Wyświetl tabelę tylko raz
				tableDrawn = true;
			}
			else {
				updateDynamicValues();  // Nadpisuj tylko zmieniające się wartości
			}
			//displayTelnet();
		}
		else {
			tableDrawn = false;
		}
#else
		displaySerial();
#endif
#if defined OLED
		display.clearDisplay();
		display.setFont(&Picopixel);
		display.setTextSize(1);
		display.setTextColor(SSD1306_WHITE);
		display.setCursor(0, TEXTFIRSTROW);
		display.print(WiFi.SSID());
		display.print(" ");
		display.println(WiFi.localIP());
		//             12345678901234567890123456
		display.println(F("Left dir          Right dir"));
		if (leftMotorDirection == FORWARD) display.print(F("FORWARD ")); else display.print(F("BACKWARD"));
		display.print(F("     "));
		if (rightMotorDirection == FORWARD) display.println(F("FORWARD ")); else display.println(F("BACKWARD"));
		display.println(F("Left speed    Right speed"));
		display.print(speedLeft);
		display.print(F("                    "));
		display.println(speedRight);
		display.println(F("Left way       Right way"));
		display.print(leftWay);
		display.print(F("                    "));
		display.println(rightWay);
		display.print(F("Battery [V]     "));
		display.println(batteryVoltage);
		display.display();
#endif
		currentLeftWay = leftWay;
		currentRightWay = rightWay;
		vTaskDelay(commPeriod / portTICK_PERIOD_MS);  //Task delay defined as commPeriod
	}
}


struct CommandData {
	String command;
	float arguments[5];  // Array to hold up to 10 numeric arguments
	int argCount;        // Number of arguments parsed
};

struct CommandData parseCommand(String input) {
	struct CommandData result;
	result.command = "";  // Default command
	result.argCount = 0;  // Initialize argument count

	input.trim();         // Remove leading and trailing spaces
	input.toUpperCase();  // Convert to uppercase for consistency

	int spaceIndex = input.indexOf(' ');
	if (spaceIndex > 0) {
		// Extract the command
		result.command = input.substring(0, spaceIndex);

		// Extract the arguments
		String argsPart = input.substring(spaceIndex + 1);
		argsPart.trim();
		while (argsPart.length() > 0 && result.argCount < 10) {
			int nextSpace = argsPart.indexOf(' ');
			if (nextSpace == -1) {  // Last argument
				result.arguments[result.argCount] = argsPart.toFloat();
				result.argCount++;
				break;
			}
			else {
				// Extract next argument
				String arg = argsPart.substring(0, nextSpace);
				result.arguments[result.argCount] = arg.toFloat();
				result.argCount++;
				argsPart = argsPart.substring(nextSpace + 1);
				argsPart.trim();
			}
		}
	}
	else {
		// No space, assume only the command
		result.command = input;
	}

	return result;
}  //End of parseCommand

void moveRobot(int motorDirection, int motorSpeed, int distance = 0) {
	leftMotorDirection = motorDirection;
	rightMotorDirection = motorDirection;
	targetLeftSpeed = motorSpeed;
	targetRightSpeed = motorSpeed;
	if (distance) {
		//Add logic to move given distance
	}
}

void stopRobot() {
	integralLeft = 0;        // Accumulated error (integral)
	integralRight = 0;       // Accumulated error (integral)
	previousErrorLeft = 0;   // Last error (for derivative)
	previousErrorRight = 0;  // Last error (for derivative)
	leftMotorDirection = FORWARD;
	rightMotorDirection = FORWARD;
	potOn = false; //Stop robot regardless on potentiometers setting
	targetLeftSpeed = 0;
	targetRightSpeed = 0;
	digitalWrite(leftMotorDIR, FORWARD);
	digitalWrite(rightMotorDIR, FORWARD);
	leftMotorPwm = 0;
	leftMotorPwm = 0;
	ledcWrite(leftMotorPWM, 0);
	ledcWrite(rightMotorPWM, 0);
}

void printMenu() {
#if defined TELNET
	telnetClient.print(F("Robot commands\n\r[F]ORWARD <speed> <distance>\n\r[B]ACKWARD <speed> <distance>\n\r[S]TOP\n\r[D]ISPLAY\n\r[E]NCODERS\n\r"));
	telnetClient.print(F("[M]ENU - print menu\n\r[P]ID <kp> <ki> <kd> - Enter PID factors\n\rP[O]T <0/1> - Potentiometers speeds control Off / On\n\r"));
	telnetClient.print(F("P[I]D <0/1> - PID regulation Off / On\n\r> "));  // Command cursor
#else
	Serial..print(F("Robot commands\n\r[F]ORWARD <speed> <distance>\n\r[B]ACKWARD <speed> <distance>\n\r[S]TOP\n\r[D]ISPLAY\n\r"));
	Serial.print(F("[M]ENU - print menu\n\r[P]ID <kp> <ki> <kd> - Enter PID factors\n\rP[O]T <0/1> - Potentiometers speeds control Off / On\n\r"));
	Serial, print(F("P[I]D <0/1> - PID regulation Off / On\n\r"));
#endif
}
//Function processing commands received from telnet terminal
#if defined TELNET
void orders() {
	if (telnetClient && telnetClient.connected() && telnetClient.available()) {
		printToTelnet = false; //Stop displaying until command is entered.
		String input = telnetClient.readStringUntil('\n');
		struct CommandData cmdData = parseCommand(input);
		Serial.print(F("Command: "));
		Serial.println(cmdData.command);
		Serial.print(F("Arguments: "));
		for (int i = 0; i < cmdData.argCount; i++) {
			Serial.print(cmdData.arguments[i]);
			Serial.print(" ");
		}
		Serial.println();

		// Process the command
		if (cmdData.command == "FORWARD" || cmdData.command == "F") {
			if (cmdData.argCount == 1) {
				int mSpeed = cmdData.arguments[0];
				if (mSpeed > maxSpeed) mSpeed = maxSpeed;
				if (mSpeed < 0) mSpeed = 0;
				telnetClient.printf(F("Moving forward indefinitely at %d mm/s.\n\r"), mSpeed);
				moveRobot(FORWARD, mSpeed);
			}
			else if (cmdData.argCount == 2) {
				int mSpeed = cmdData.arguments[0];
				int distance = cmdData.arguments[1];
				if (mSpeed > maxSpeed) mSpeed = maxSpeed;
				if (mSpeed < 0) mSpeed = 0;
				telnetClient.printf(F("Moving forward at %d mm/s for a distance of %d mm.\n\r"), mSpeed, distance);
				moveRobot(FORWARD, mSpeed, distance);
			}
			else {
				telnetClient.println(F("Invalid number of arguments for FORWARD command."));
			}
		}
		else if (cmdData.command == "BACKWARD" || cmdData.command == "B") {
			if (cmdData.argCount == 1) {
				int mSpeed = cmdData.arguments[0];
				if (mSpeed > maxSpeed) mSpeed = maxSpeed;
				if (mSpeed < 0) mSpeed = 0;
				telnetClient.printf(F("Moving backward indefinitely at %d mm/s.\n\r"), mSpeed);
				moveRobot(BACKWARD, mSpeed);
			}
			else if (cmdData.argCount == 2) {
				int mSpeed = cmdData.arguments[0];
				int distance = cmdData.arguments[1];
				if (mSpeed > maxSpeed) mSpeed = maxSpeed;
				if (mSpeed < 0) mSpeed = 0;
				telnetClient.printf(F("Moving backward at %d mm/s for a distance of %d mm.\n\r"), mSpeed, distance);
				moveRobot(BACKWARD, mSpeed, distance);
			}
			else {
				telnetClient.println(F("Invalid number of arguments for BACKWARD command."));
			}
		}
		else if (cmdData.command == "STOP" || cmdData.command == "S") {
			telnetClient.println(F("Stop robot movement!"));
			stopRobot();
		}
		else if (cmdData.command == "DISPLAY" || cmdData.command == "D") {
			printToTelnet = true;
			printIntervalsToTelnet = false;
		}
		else if ((cmdData.command == "MENU" || cmdData.command == "M")) {
			printToTelnet = false;
			printIntervalsToTelnet = false;
			printMenu();
		}
		else if ((cmdData.command == "ENCODERS" || cmdData.command == "E")) {
		}
		else if (cmdData.command == "PID" || cmdData.command == "P") {
			if (cmdData.argCount == 1) {
				pidPreferences.kp = cmdData.arguments[0];
				preferences.putBytes("pidPreferences", &pidPreferences, sizeof(pidPreferences));
				telnetClient.print(F("Stored to NVM new kp = "));
				telnetClient.print(pidPreferences.kp);
				telnetClient.print(F("\n\r"));
			}
			else if (cmdData.argCount == 2) {
				pidPreferences.kp = cmdData.arguments[0];
				pidPreferences.ki = cmdData.arguments[1];
				preferences.putBytes("pidPreferences", &pidPreferences, sizeof(pidPreferences));
				telnetClient.print(F("Stored to NVM new kp = "));
				telnetClient.print(pidPreferences.kp);
				telnetClient.print(F(" ki = "));
				telnetClient.print(pidPreferences.ki);
				telnetClient.print(F("\n\r"));
			}
			else if (cmdData.argCount == 3) {
				pidPreferences.kp = cmdData.arguments[0];
				pidPreferences.ki = cmdData.arguments[1];
				pidPreferences.kd = cmdData.arguments[2];
				preferences.putBytes("pidPreferences", &pidPreferences, sizeof(pidPreferences));
				//telnetClient.print(F("Stored to NVM new kp = "));
				//telnetClient.print(pidPreferences.kp);
				//telnetClient.print(F(" ki = "));
				//telnetClient.print(pidPreferences.ki);
				//telnetClient.print(F(" kd = "));
				//telnetClient.print(pidPreferences.kd);
				//telnetClient.print(F("\n\r"));
				//telnetClient.printf(F("Stored to NVM new kp= %s ki= %s kd= %s\r\n"), floatToString(pidPreferences.kp, 1, 3), floatToString(pidPreferences.ki, 1, 3), floatToString(pidPreferences.kd, 1, 3));
				telnetClient.print(F("Stored to NVM new kp= "));
				telnetClient.print(floatToString(pidPreferences.kp, 1, 3));
				telnetClient.print(F(" ki= "));
				telnetClient.print(floatToString(pidPreferences.ki, 1, 3));
				telnetClient.print(F(" kd= "));
				telnetClient.print(floatToString(pidPreferences.kd, 1, 3));
			}
			else {
				telnetClient.print(F("Invald parameters for PID command\n\r"));
			}
		}
		else if (cmdData.command == "POT" || cmdData.command == "O") {
			if (cmdData.argCount == 1) {
				potOn = cmdData.arguments[0];
				if (potOn) {
					potOn = true;
					telnetClient.println(F("Potentiometer speed control set to On\n\r"));
				}
				else {
					telnetClient.println(F("Potentiometer speed control set to Off\n\r"));
				}
			}
			else {
				telnetClient.print(F("Invald parameters for POT command\n\r"));
			}
		}
		else if (cmdData.command == "PID" || cmdData.command == "I") {
			if (cmdData.argCount == 1) {
				PIDon = cmdData.arguments[0];
				if (PIDon) {
					PIDon = true;
					telnetClient.println(F("PID speed regulation set to On\n\r"));
				}
				else
					telnetClient.println(F("PID speed regulation set to Off\n\r"));
			}
			else {
				telnetClient.println(F("Invald parameters for PID command\n\r"));
			}
		}
		else {
			telnetClient.println(F("Unknown command "));
		}
	}
}
#endif

// Function to stabilize motor speed simple proportional
//int stabilizeSpeed(int targetSpeed, int currentSpeed)
//{
//  int speedDelta = targetSpeed - currentSpeed;
//  int correction = speedDelta / 1.5;
//  return constrain(targetSpeed + correction, 0, maxSpeed);
//}

//****************************SETUP***********************************************************//
void setup() {
	Serial.begin(115200);
#if defined OLED
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display.clearDisplay();
	display.setFont(&Picopixel);
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0, TEXTFIRSTROW);
	display.println(F(VERSION));
	display.display();
#endif
	Serial.println(F(VERSION));
	preferences.begin("pidPreferences", false);  // Namespace
	// Load PID parameters
	if (preferences.getBytes("pidPreferences", &pidPreferences, sizeof(pidPreferences)) == 0) {
		Serial.println("No stored PID data, using defaults.");
	}
	else {
		Serial.printf("Loaded PID params: kp=%.2f, ki=%.2f, kd=%.2f\n", pidPreferences.kp, pidPreferences.ki, pidPreferences.kd);
	}
	setupOTA("Motor_Control", mySSID, myPASSWORD, OTAPASSWORD);
	Serial.println(F("OTA initialized"));

	//configure motors pins outputs
	pinMode(leftMotorDIR, OUTPUT);
	pinMode(rightMotorDIR, OUTPUT);
	pinMode(leftMotorPWM, OUTPUT);
	pinMode(rightMotorPWM, OUTPUT);
	digitalWrite(leftMotorPWM, LOW);
	digitalWrite(rightMotorPWM, LOW);
	digitalWrite(leftMotorDIR, LOW);
	digitalWrite(rightMotorDIR, LOW);


	// Configure potentiometers as inputs
	pinMode(leftPot, INPUT);
	pinMode(rightPot, INPUT);

	//Configure eqyalSpeed jumper as input
	pinMode(equalSpeed, INPUT_PULLUP);

	// Configure encoders
	//pinMode(leftEncoder, INPUT_PULLUP);  
	//pinMode(rightEncoder, INPUT_PULLUP); 
	pinMode(leftEncoder, INPUT_PULLDOWN);
	pinMode(rightEncoder, INPUT_PULLDOWN);
	//attachInterrupt(leftEncoder, leftEncoderISR, CHANGE);
	//attachInterrupt(rightEncoder, rightEncoderISR, CHANGE);
	lastTime = millis();

	// Attach interrupts for both sensors
	attachInterrupt(digitalPinToInterrupt(leftEncoder), handleStateChangeMotorLeft, RISING);
	attachInterrupt(digitalPinToInterrupt(rightEncoder), handleStateChangeMotorRight, RISING);


	// Confogure ADC
	pinMode(batteryPin, INPUT);

	// Configure PWM channels in one step
	ledcAttachChannel(leftMotorPWM, pwmFreq, pwmResolution, 0);   // Left Motor PWM: Channel 0
	ledcAttachChannel(rightMotorPWM, pwmFreq, pwmResolution, 1);  // Right Motor PWM: Channel 1


	// Creating FreeRTOS task to control dc motors speed
	xTaskCreate(
		MotorControlTask,        // Task function
		"Motor Control",         // Task name
		2048,                    // Stock assigned for task in words - 4 bytes for 32 bits processors
		NULL,                    // Task parameters
		2,                       // Task priority
		//    NULL               // Task Handle
		&MotorControlTaskHandle  // Task Handle
	);

	// Creating FreeRTOS task for telnet communication
	xTaskCreate(
		CommunicationTask,  // Task function
		"Communication",   // Task name
		2048,              // Stock assigned for task in words - 4 bytes for 32 bits processors
		NULL,              // Task parameters
		1,                 // Task priority
		//    NULL                 // Task Handle
		&CommunicationTaskHandle  // Task Handle
	);

}  //*****************End of setup**********************************

//*****************loop*********************************************
void loop() {
	//Nothing here. Everything in Rtos task.
}
//*****************End of loop**************************************
