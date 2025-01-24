//Works on board WEMOS D1 R32 and ESP32 DEV MODULE
//Install library ESP32_Network_Services from //https://github.com/aimeiz/ESP32_Network_Services.git

#define VERSION "ESP32Motors_encoders_Rtos_OTA_Telnet_NVM_250124"
//#define PULSE_COUNT_VS_WIDTH_METHOD //Chose motors speed meters method comment / uncomment appriopriate
//#define ESP32CAM
#define ESP32WROVERDEV //ESP32 Wroover Module Default 4MB with SPIFFS 1.2MB App 1.5MB SPIFFS
#include <Arduino.h>
#include <Preferences.h>  //For Non Volatile Memory to store preferences.
#include"OutputHandler.h"
//#include <Wire.h>
OutputHandler output;
#define ENABLE_OLED
#define ENABLE_OTA
#define ENABLE_FTP
#define ENABLE_SPIFFS
#define ENABLE_NTP
#define GMTOFFSET_SEC 3600 				//Default UTC time
#define DAYLIGHTOFFSET_SEC 3600 	//Default summer time offset 1 hour
//#define ENABLE_TELNETSTREAM
#define ENABLE_WEBSERVER
#define ENABLE_WEBSOCKETS
#define WEBSOCKETS_PORT 83
//#define NETWORK_SERVICES_CORE 1 //By default service runs on core 0
//#define FASTNET_SERVICE_CORE 1  //By default service runs on core 0
//#define COMMUNICATION_TASK_CORE 0  //Default Communication task core is 1
//#define MOTOR_CONTROL_TASK_CORE 0  //Default Motor Control task core is 1
#ifdef ENABLE_OLED
#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TEXTFIRSTROW 4 
//#include <spi.h>
//#include <wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <fonts/Picopixel.h>        // Font Picopixel
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
#define OTA_PASSWORD "robot1"
#define OTA_HOSTNAME VERSION
#include <credentials.h>
//const char* wifiSSID PROGMEM = MYSSID;
//const char* wifiPassword PROGMEM = MYPASSWORD;
//const char* ftpUser PROGMEM = FTPUSER;
//const char* ftpPassword PROGMEM = FTPPASSWORD;
//Library for network services like OTA, FTP, SPIFFS, NTP etc. use #define ENABLE_OTA, ENABLE_FTP, ENABLE_SPIFFS, ENABLE_NTP to incorporate services to your code.
#include<ESP32_Network_Services.h> //https://github.com/aimeiz/ESP32_Network_Services.git
uint32_t entry;
//*********************
// Pin definitions
//Jtag pins 12 13 14 15
#ifdef ESP32CAM //AI Thinker
#include"Camera.h"
bool sendPhotos = false; // if troe sends video stream over websocket
#include <PCF8574.h>
PCF8574 pcf8574(0x20); //Default address 0x20
//1-TXD, 3- RXD UART
const int sda PROGMEM = 3;// 0;					//Pins 0 and 16 blocks camera. 16 drives PSRAM
const int scl PROGMEM = 2;
const int leftMotorPWM PROGMEM = 12;			// GPIO for Left Motor PWM
const int rightMotorPWM PROGMEM = 13;			// GPIO for Right Motor PWM
const int leftEncoder PROGMEM = 14;				// GPIO INT for Left Encoder
const int rightEncoder PROGMEM = 15;			// GPIO INT for Right Encoder
const int batteryPin = 3;// 0; //3;// 16;			//ADC Port for battery voltage measurement shred with (RX)
const int flashLed = 4;					//Flash LED hardwired at ESP32CAM
//Expender I2C PCF8754
const int leftMotorDIR PROGMEM = 0;			// GPIO for Left Motor DIR
const int rightMotorDIR PROGMEM = 1;			// GPIO for Right Motor DIR

#elif defined ESP32WROVERDEV						//WROVER DEV
#include"Camera.h"
bool sendPhotos = false; // if troe sends video stream over websocket
//TXD 1
// RXD 3
const int scl PROGMEM = 0;  						// I2C SCL Shared with Boot button
const int sda PROGMEM = 3;							// I2C SDAShared with UART RX
const int leftMotorPWM PROGMEM = 32;				// GPIO for Left Motor PWM
const int rightMotorPWM PROGMEM = 33;				// GPIO for Right Motor PWM
const int leftMotorDIR PROGMEM = 13;				// GPIO for Left Motor DIR
const int rightMotorDIR PROGMEM = 14;				// GPIO for Right Motor DIR
const int leftEncoder PROGMEM = 12;					// GPIO for Left Encoder
const int rightEncoder PROGMEM = 13;				// GPIO for Right Encoder
const int batteryPin PROGMEM = 15;					//ADC Port for battery voltage measurement
const int flashLed PROGMEM = 2;								//Flash LED
//const int leftPot PROGMEM = ;						// GPIO for Left Potentiometer
//const int rightPot PROGMEM = ;					// GPIO for Right Potentiometer
//const int equalSpeed PROGMEM = ;					// GPIO for right speed = left speed adjusted by left speed potentiometer

#else
//I2C SCL 22  
//I2C SDA 21
const int leftMotorPWM PROGMEM = 26;				// GPIO for Left Motor PWM
const int rightMotorPWM PROGMEM = 33;				// GPIO for Right Motor PWM
const int leftMotorDIR PROGMEM = 4;					// GPIO for Left Motor DIR
const int rightMotorDIR PROGMEM = 27;				// GPIO for Right Motor DIR
const int leftPot PROGMEM = 34;						// GPIO for Left Potentiometer
const int rightPot PROGMEM = 35;					// GPIO for Right Potentiometer
const int leftEncoder PROGMEM = 32;					// GPIO for Left Encoder
const int rightEncoder PROGMEM = 25;				// GPIO for Right Encoder
const int equalSpeed PROGMEM = 19;					// GPIO for right speed = left speed adjusted by left speed potentiometer
const int batteryPin PROGMEM = 36;					//ADC Port for battery voltage measurement
const int flashLed = 2;								//Flash LED
#endif
const float batteryDivider PROGMEM = 0.00371;		//Resistor divider for battery measurements
const float batteryNorm PROGMEM = 7.6;				//Nominal battery voltage 7.6V - 2s packet
//const float batteryNorm PROGMEM = 11.4;				//Nominal battery voltage 7.6V - 3s packet
const float batteryLow PROGMEM = batteryNorm * 0.92;//Battery emergency levev 7.0V
const float batteryMax PROGMEM = batteryNorm * 1.105;//Maximal battery voltage 8.4V
const float batteryMin PROGMEM = batteryNorm * 0.79; //Minimal battery voltage 6.0V. Below motors stop
#define FORWARD LOW
#define BACKWARD HIGH
#define STOP LOW
bool stopped = true;
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
float errorLeft = 0;			//Error of left motor
float errorRight = 0;			//Error of right motor
float integralLeft = 0;        // Accumulated error (integral)
float integralRight = 0;       // Accumulated error (integral)
float derivativeLeft = 0;      //Derivative error
float derivativeRight = 0;      //Derivative error
float previousErrorLeft = 0;   // Last error (for derivative)
float previousErrorRight = 0;  // Last error (for derivative)
int targetSpeed = 0;			//Overall target speed. If no turn targetLeftSpeed = targetSpeed and targetRightSpeed = tergetSpeed
int turn = 0;					// -100 to 100 differentiates left / right motor speed to make turn. targetLeftspped = targetSpeed + targetSpeed * turn / 100
int direction = FORWARD;		//Overal movment direction
//targetRightspped = targetSpeed - targetSpeed * turn / 100 for backward dir the oposite.
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int targetLeftSpeedCorrected = 0;
int targetRightSpeedCorrected = 0;
int speedLeft;   //Current wheel speed in pulses / s
int speedRight;  //Current wheel speed in pulses / s
volatile long leftWay = 0;
volatile long rightWay = 0;
long currentLeftWay;
long currentRightWay;
const uint32_t minPulse PROGMEM = 8000ul; // 5000ul;        //Minimal accepted pulse lebgth or minimal time between interrupts in uS
const uint32_t maxPulse PROGMEM = 200000ul;	   //Maximal zccepted pulse length in uS
// LEDC configuration
const int motorsPwmFreq PROGMEM = 5000;    //  Motors PWM frequency in Hz
const int ledPwmFreq PROGMEM = 20000;    //  Led PWM channel
const int pwmResolution PROGMEM = 8;  // 8-bit resolution (0-255)
const int pwmMin PROGMEM = 75; //Minimal pwm value during run
const int pwmMax PROGMEM = 255; //Minimal pwm value during run
// Variables for motor speed and stabilization
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;
int leftMotorPwm = 0;
int rightMotorPwm = 0;
//Measuring speeds by measuring time of PULSES
//#define PULSES 100
//#define PULSES 40
//#define PULSES 20
//#define PULSES 10
//#define PULSES 5
#define PULSES 4 //3 //Number of encoder pulses to calculate average
int leftMotorDirection = direction;   //LOW means forward HI means backward
int rightMotorDirection = direction;  //LOW means forward HI means backward
unsigned long lastTime;
//Maxspeed = 300 RPM Encoder 20 pulses per revolution (RISING or FALLING) 40 pulses (CHANGE) gives 100 or 200 pulses/s. 5000us or 10000us
//Minimal speed 30 RPM gives 10 or 20 pulses/s pulse width 50000us or 100000us 
const int maxSpeed PROGMEM = 110;              // Maximum speed in pulses per second

const unsigned long commPeriod PROGMEM = 100;// 33;// 500;  //Network communication period in miliseconds 33 means 30 frames / sec
bool printToTelnet = false;
bool printIntervalsToTelnet = false;
float batteryVoltage = 0;  //Stores battery voltage
bool simulationEnable = false; //enables simulation work without motors hardware
#define DEFAULTLOGTIME = 5000ul; //default loging time 5 sec.
TaskHandle_t MotorControlTaskHandle;  //Handle for MotorControlTask
TaskHandle_t CommunicationTaskHandle;  //Handle for MotorControlTask

//Logowanie
const char logHeader[] PROGMEM = "Time\ttgtSpd\tturn\LtTgtSpd\tLtSpeed\tLtTgSpdCor\tLtPWM\tLtWay\tLtErr\tLtInt\tLtDer\tRtTgtSpd\tRtSpeed\tRtTgSpdCor\tRtPWM\tRtWay\tRtErr\tRtInt\tRtDer\tVbat\tKp\tKi\tKd\tPWMMin\tPWMMax\r\n";
const char logFormat[] PROGMEM = "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%s\t%s\t%s\t%d\t%d\t%d\t%d\t%d\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%d\t%d\r\n";
String logData;
bool logOn = false; //Global variable controlling loging state
String ordersResponse;
//File logFile;

void logWrite(unsigned long interval = 5, char* fileName = "/log.txt") {
	//void logWrite(unsigned long interval = 5) {
	static unsigned long startTime = 0;
	static unsigned long logEndTime = 0;
	static File logFile;
	static bool first = true;
	static char Name[30];		 // Filename buffer
	// If interval == 0 log variables in loop
	if (interval == 0) {
		if (logOn) {
			if (first) {
				first = false;
				startTime = millis();
			}
			unsigned long currentTime = millis() - startTime;
			logFile.printf(logFormat, currentTime, targetSpeed, turn, targetLeftSpeed, speedLeft, targetLeftSpeedCorrected, leftMotorPwm, leftWay, String(errorLeft, 3), String(integralLeft, 3), String(derivativeLeft, 3),
				targetRightSpeed, speedRight, targetRightSpeedCorrected, rightMotorPwm, rightWay, String(errorRight, 3), String(integralRight, 3), String(derivativeRight, 3),
				String(batteryVoltage, 2), String(pidPreferences.kp, 3), String(pidPreferences.kp, 3), String(pidPreferences.kd, 3), pwmMin, pwmMax);
			if (millis() >= logEndTime) {
				logFile.close();
				ordersResponse = F("Logfile closed. Recording complete");
				Serial.println(ordersResponse);
				logOn = false;
				startTime = 0;
			}
		}
		return; // return if interval == 0
	}

	// Loging init. Preparing filename, opening file.
	if (interval > 30) interval = 30; // 30s Max recording time
	logEndTime = millis() + (interval * 1000);
	strftime(Name, sizeof(Name), "/Log_%Y%m%d-%H%M%S.txt", &timeinfo);
	logFile = SPIFFS.open(Name, FILE_WRITE);
	if (!logFile) {
		Serial.println(F("Failed to open log file"));
		logOn = false;
		return;
	}
	output.printf(F("Log %s open for %ds recording"), Name, interval);
	ordersResponse = output.getBuffer();
	Serial.println(ordersResponse);
	logFile.println(Name);
	logFile.print(logHeader); // Save log header
	first = true;
	logOn = true;
}


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
//const int measureCount PROGMEM = 100;

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

void simulation(bool enable = false) {
	if (!enable) return;
	batteryVoltage = (float)random(batteryNorm * 90, batteryNorm * 110) / 100.0;
	speedLeft = (int)(((float)random(targetLeftSpeed * 70, targetLeftSpeed * 130)) / 100.0);
	speedRight = (int)(((float)random(targetRightSpeed * 70, targetRightSpeed * 130)) / 100.0);
	leftWay += (leftMotorDirection == FORWARD) ? speedLeft / 20 : speedLeft / -20;
	rightWay += (rightMotorDirection == FORWARD) ? speedRight / 20 : speedRight / -20;
}
void speedMeasure() {
	const uint32_t falsePeriod PROGMEM = 1000000ul;
	static int measuresCounterLeft = 0;
	static int measuresCounterRight = 0;
	if (leftSpeedMeasured) {
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
	if (rightSpeedMeasured)
	{
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
} //End of speedMeasure()
#endif


	// PID-based function to stabilize motor speed
int stabilizeSpeed(int targetSpeed, int currentSpeed, float* error, float* integral, float* previousError, float* derivative) {
	// Calculate the error
	*error = (float)(targetSpeed - currentSpeed);

	// Proportional term
	float proportional = pidPreferences.kp * *error;

	// Integral term
	*integral += *error;
	float integralTerm = pidPreferences.ki * *integral;

	// Derivative term
	*derivative = *error - *previousError;
	float derivativeTerm = pidPreferences.kd * *derivative;

	// Combine terms
	float correction = proportional + integralTerm + derivativeTerm;

	// Update previous error for next iteration
	*previousError = *error;

	// Apply correction and constrain output
	return constrain(targetSpeed + correction, 0, maxSpeed);
}

void MotorControlTask(void* pvParameters) {
	const int batteryMeasures PROGMEM = 10;
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
		simulation(simulationEnable); //If enabled overrides speed and battery measurements by random values

		if (batteryVoltage < batteryMin) stopRobot();              // Stop robot if voltage is to low.
#if !defined(ESP32CAM) && !defined(ESP32WROVERDEV)
		// Read potentiometer values (0-4095 for 12-bit ADC)
		int leftPotValue = analogRead(leftPot);
		int rightPotValue = analogRead(rightPot);

		//     Map potentiometer values to target speeds in pulses / second
		if (potOn) {
			targetLeftSpeed = map(leftPotValue, 0, 4095, 0, 1000); //1000 pulses per second is maximum
			targetRightSpeed = map(rightPotValue, 0, 4095, 0, 1000); //1000 pulses per second is maximum
			if (digitalRead(equalSpeed) == LOW) targetRightSpeed = targetLeftSpeed; //Both motors run targetLeftSpeed if jumper is set
		}
#endif
		targetLeftSpeedCorrected = targetLeftSpeed;
		targetRightSpeedCorrected = targetRightSpeed;
		if (PIDon) {
			//Stabilize motor speeds using encoder feedback
			targetLeftSpeedCorrected = stabilizeSpeed(targetLeftSpeed, speedLeft, &errorLeft, &integralLeft, &previousErrorLeft, &derivativeLeft);
			targetRightSpeedCorrected = stabilizeSpeed(targetRightSpeed, speedRight, &errorRight, &integralRight, &previousErrorRight, &derivativeRight);
		}
#ifdef ESP32CAM
		//Write directions to direction ports via expander PCF8754
		pcf8574.write(leftMotorDIR, leftMotorDirection);
		pcf8574.write(rightMotorDIR, rightMotorDirection);
#else
		//Write directions to direction ports
		digitalWrite(rightMotorDIR, rightMotorDirection);
		digitalWrite(leftMotorDIR, leftMotorDirection);
#endif
		// Map target speeds values to motor speeds (0-255 or 255 - 0 for 8-bit PWM)
		if (leftMotorDirection == FORWARD)
			//leftMotorPwm = map(targetLeftSpeedCorrected, 0, maxSpeed, 0, 255);
			leftMotorPwm = map(targetLeftSpeedCorrected, 0, maxSpeed, pwmMin, pwmMax);
		else
			//leftMotorPwm = map(targetLeftSpeedCorrected, 0, maxSpeed, 255, 0);
			leftMotorPwm = map(targetLeftSpeedCorrected, 0, maxSpeed, pwmMax - pwmMin, 0);
		if (rightMotorDirection == FORWARD)
			//rightMotorPwm = map(targetRightSpeedCorrected, 0, maxSpeed, 0, 255);
			rightMotorPwm = map(targetRightSpeedCorrected, 0, maxSpeed, pwmMin, pwmMax);
		else
			//rightMotorPwm = map(targetRightSpeedCorrected, 0, maxSpeed, 255, 0);
			rightMotorPwm = map(targetRightSpeedCorrected, 0, maxSpeed, pwmMax - pwmMin, 0);

		// Set motor speeds
		if (stopped) { //set zero voltage to both motors
			//digitalWrite(rightMotorDIR, FORWARD);
			//digitalWrite(leftMotorDIR, FORWARD);
			(leftMotorDirection == FORWARD) ? leftMotorPwm = 0 : leftMotorPwm = 255;
			(rightMotorDirection == FORWARD) ? rightMotorPwm = 0 : rightMotorPwm = 255;
		}
		ledcWrite(leftMotorPWM, leftMotorPwm);
		ledcWrite(rightMotorPWM, rightMotorPwm);
		leftEncoderCount = 0;
		rightEncoderCount = 0;
		//if (logOn) {
		logWrite(0); // Writing variables to log on SPIFFS if enabled.
		//}
		vTaskDelay(pdMS_TO_TICKS(10));  //Task delay 10ms
	}
}  //End of MotorControlTask

String floatToString(float value, int przed_przecinkiem, int po_przecinku) {
	static char buffer[20]; // Static buffer for result (20 characters is enough for cases majoroty)
	dtostrf(value, przed_przecinkiem, po_przecinku, buffer);
	return String(buffer); // Return pointer to string
}
#ifdef ENABLE_TELNETSTREAM
#include"static_table_TelnetStream2.h";
#endif
#if defined TELNET
char* floatToString(float value, int przed_przecinkiem, int po_przecinku) {
	static char buffer[20];  // Static buffer for result (20 characters is enough for cases majoroty)
	dtostrf(value, przed_przecinkiem, po_przecinku, buffer);
	return buffer; // Return pointer to string
}
#include "static_table.h"

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
#ifdef ENABLE_WEBSERVER
#include "Web_table.h"
#endif
#ifdef ENABLE_TELNET
void processTelnet() {
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
#endif

#ifdef ENABLE_TELNETSTREAM
	void processTelnetStream() {
	}
#endif

	void CommunicationTask(void* pvParameters) {
		for (;;) {
			if (currentLeftWay == leftWay) speedLeft = 0;
			if (currentRightWay == rightWay) speedRight = 0;
#if defined TELNET
			processTelnet();
			//static bool tableDrawn;
			//if (printToTelnet) {
			//	if (!tableDrawn) {
			//		displayStaticTable();  // Wyświetl tabelę tylko raz
			//		//displayDynamicTable(TABLE_WIDTH, TABLE_HEIGHT,TABLE_TITTLE,labels,labelCount );
			//		tableDrawn = true;
			//	}
			//	else {
			//		updateDynamicValues();  // Nadpisuj tylko zmieniające się wartości
			//	}
			//	//displayTelnet();
			//}
			//else {
			//	tableDrawn = false;
			//}
#endif
#ifdef ENABLE_TELNETSTREAM
			processTelnetStream();
			updateDynamicValuesTelnetStream();
#endif
#ifdef ENABLE_WEBSOCKETS
			if (webSocket.connectedClients()) {
				processWebsockets();
				websocketsUpdateDynamicValues(); //Send variables to webpage using websockets stream protocol
#if defined(ESP32CAM) || defined(ESP32WROVERDEV)
				if (sendPhotos) {
					sendPhotoOverWebSocket(0);
				}
#endif
			}
#endif
#if defined ENABLE_OLED
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
	}//end of Commmunication task


	//typedef struct {
	//	String command;
	//	float arguments[5];  // Array to hold up to 10 numeric arguments
	//	int argCount;        // Number of arguments parsed
	//} struct CommandData;
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
			result.argCount = 0;
		}

		return result;
	}  //End of parseCommand

	void moveRobot(int motorDirection, int motorSpeed, int turn = 0, int distance = 0) {
		if (turn > 100) turn = 100;
		if (turn < -100) turn = -100;
		leftMotorDirection = motorDirection;
		rightMotorDirection = motorDirection;
		targetLeftSpeed = (motorDirection == FORWARD) ? motorSpeed + motorSpeed * turn / 100 : motorSpeed - motorSpeed * turn / 100;
		targetRightSpeed = (motorDirection == FORWARD) ? motorSpeed - motorSpeed * turn / 100 : motorSpeed + motorSpeed * turn / 100;
		integralLeft = 0; //if target speed changes, PID integral is zeroed to avoid excessive error acumulation
		integralRight = 0; //if target speed changes, PID integral is zeroed to avoid exscessive error acumulation
		if (distance) {
			//Add logic to move given distance
		}
	}

	void stopRobot() {
		stopped = true;
		integralLeft = 0;        // Accumulated error (integral)
		integralRight = 0;       // Accumulated error (integral)
		previousErrorLeft = 0;   // Last error (for derivative)
		previousErrorRight = 0;  // Last error (for derivative)
		potOn = false; //Stop robot regardless on potentiometers setting
		targetSpeed = 0;
		targetLeftSpeed = 0;
		targetRightSpeed = 0;
		if (leftMotorDirection == FORWARD) {
#ifdef ESP32CAM
			pcf8574.write(leftMotorDIR, FORWARD);
#else
			digitalWrite(leftMotorDIR, FORWARD);
#endif
			leftMotorPwm = 0;
			ledcWrite(leftMotorPWM, 0);
		}
		else if (leftMotorDirection == BACKWARD)
		{
#ifdef ESP32CAM
			pcf8574.write(leftMotorDIR, BACKWARD);
#else
			digitalWrite(leftMotorDIR, BACKWARD);
#endif
			leftMotorPwm = 0;
			ledcWrite(leftMotorPWM, 255);
		}
		if (rightMotorDirection == FORWARD) {
#ifdef ESP32CAM
			pcf8574.write(rightMotorDIR, FORWARD);
#else
			digitalWrite(rightMotorDIR, FORWARD);
#endif
			rightMotorPwm = 0;
			ledcWrite(rightMotorPWM, 0);
		}
		else if (rightMotorDirection == BACKWARD)
		{
#ifdef ESP32CAM
			pcf8574.write(rightMotorDIR, BACKWARD);
#else
			digitalWrite(rightMotorDIR, BACKWARD);
#endif
			rightMotorPwm = 0;
			ledcWrite(rightMotorPWM, 255);
		}
	}

	void printMenu() {
#if defined TELNET
		displayStaticTable();
		//displayDynamicTable(TABLE_WIDTH, TABLE_HEIGHT, TABLE_TITTLE, labels, labelCount);
		updateDynamicValues();
		telnetClient.print(F("System commands\r\n[F]ORWARD <speed> <distance>\r\n[B]ACKWARD <speed> <distance>\r\n[S]TOP\r\n[D]ISPLAY\r\n[E]NCODERS\r\n"));
		telnetClient.print(F("[M]ENU - print menu\r\n[P]ID <kp> <ki> <kd> - Enter PID factors\r\nP[O]T <0/1> - Potentiometers speeds control Off / On\r\n[L]OG <sec> - Loging for sec seconds 1 - 30\r\n"));
		telnetClient.print(F("P[I]D <0/1> - PID regulation Off / On\r\n> "));  // Command cursor
#elif defined ENABLE_TELNETSTREAM
		displayStaticTable();
		//displayDynamicTable(TABLE_WIDTH, TABLE_HEIGHT, TABLE_TITTLE, labels, labelCount);
		updateDynamicValues();
		TelnetStream2.print(F("System commands\r\n[F]ORWARD <speed> <distance>\r\n[B]ACKWARD <speed> <distance>\r\n[S]TOP\r\n[D]ISPLAY\r\n[E]NCODERS\r\n"));
		TelnetStream2.print(F("[M]ENU - print menu\r\n[P]ID <kp> <ki> <kd> - Enter PID factors\r\nP[O]T <0/1> - Potentiometers speeds control Off / On\r\n[L]OG <sec> - Loging for sec seconds 1 - 30to /log.txt\r\n"));
		TelnetStream2.print(F("P[I]DON <0/1> - PID regulation Off / On\r\n> "));  // Command cursor
#else
		Serial.print(F("System commands\r\n[F]ORWARD <speed> <distance>\r\n[B]ACKWARD <speed> <distance>\n\r[S]TOP\n\r[D]ISPLAY\r\n"));
		Serial.print(F("[M]ENU - print menu\r\n[P]ID <kp> <ki> <kd> - Enter PID factors\r\nP[O]T <0/1> - Potentiometers speeds control Off / On\r\n"));
		Serial.print(F("P[I]DON <0/1> - PID regulation Off / On\r\n"));
		Serial.print(F("[L]OG <sec> - Loging for sec seconds 1 - 30 to /log.txt\r\n"));
#endif
	}

	void led(uint8_t light = 0) {

		ledcWrite(flashLed, light);
	}

	//Function processing commands received from telnet terminal
	//Struct struct CommandData cmdData;
	String orders(struct CommandData cmdData) {
		char resp[64];
		// Process the command
		if (cmdData.command == "FORWARD" || cmdData.command == "F") {
			if (cmdData.argCount == 0) {
				direction = FORWARD;
				output.printf(F("Direstion set to forward\r\n"));
				moveRobot(direction, targetSpeed, turn);
			}
			else if (cmdData.argCount == 1) {
				stopped = false;
				//int mSpeed = cmdData.arguments[0];
				//if (mSpeed > maxSpeed) mSpeed = maxSpeed;
				//if (mSpeed < 0) mSpeed = 0;
				targetSpeed = cmdData.arguments[0];
				direction = FORWARD;
				if (targetSpeed > maxSpeed) targetSpeed = maxSpeed;
				if (targetSpeed < 0) targetSpeed = 0;
				output.printf(F("Moving forward indefinitely at %d mm/s.\r\n"), targetSpeed);
				moveRobot(direction, targetSpeed, turn);
			}
			else if (cmdData.argCount == 2) {
				stopped = false;
				int distance = cmdData.arguments[1];
				//int mSpeed = cmdData.arguments[0];
				//if (mSpeed > maxSpeed) mSpeed = maxSpeed;
				//if (mSpeed < 0) mSpeed = 0;
				targetSpeed = cmdData.arguments[0];
				direction = FORWARD;
				if (targetSpeed > maxSpeed) targetSpeed = maxSpeed;
				if (targetSpeed < 0) targetSpeed = 0;
				output.printf(F("Moving forward at %d mm/s for a distance of %d mm.\r\n"), targetSpeed, distance);
				moveRobot(direction, targetSpeed, turn, distance);
			}
			else {
				output.println(F("Invalid number of arguments for FORWARD command."));
			}
		}
		else if (cmdData.command == "BACKWARD" || cmdData.command == "B") {
			if (cmdData.argCount == 0) {
				direction = BACKWARD;
				output.printf(F("Direstion set to backward\r\n"));
				moveRobot(direction, targetSpeed, turn);
			}
			else if (cmdData.argCount == 1) {
				stopped = false;
				//int mSpeed = cmdData.arguments[0];
				//if (mSpeed > maxSpeed) mSpeed = maxSpeed;
				//if (mSpeed < 0) mSpeed = 0;
				targetSpeed = cmdData.arguments[0];
				direction = BACKWARD;
				if (targetSpeed > maxSpeed) targetSpeed = maxSpeed;
				if (targetSpeed < 0) targetSpeed = 0;
				output.printf(F("Moving backward indefinitely at %d mm/s.\r\n"), targetSpeed);
				moveRobot(direction, targetSpeed, turn);
			}
			else if (cmdData.argCount == 2) {
				stopped = false;
				int distance = cmdData.arguments[1];
				//int mSpeed = cmdData.arguments[0];
				//if (mSpeed > maxSpeed) mSpeed = maxSpeed;
				//if (mSpeed < 0) mSpeed = 0;
				targetSpeed = cmdData.arguments[0];
				direction = BACKWARD;
				if (targetSpeed > maxSpeed) targetSpeed = maxSpeed;
				if (targetSpeed < 0) targetSpeed = 0;
				output.printf(F("Moving backward at %d mm/s for a distance of %d mm.\r\n"), targetSpeed, distance);
				moveRobot(direction, targetSpeed, turn, distance);
			}
			else {
				output.print(F("Invalid number of arguments for BACKWARD command. \r\n"));
			}
		}
		else if (cmdData.command == "SPEED" || cmdData.command == "SPE") {
			if (cmdData.argCount == 1) {
				targetSpeed = cmdData.arguments[0];
				if (targetSpeed > maxSpeed) targetSpeed = maxSpeed;
				if (targetSpeed < 0) targetSpeed = 0;
				stopped = false;
				output.printf(F("Set tgtSpeed to %d mm/s.\r\n"), targetSpeed);
				moveRobot(direction, targetSpeed, turn);
				if (targetSpeed == 0) {
					stopRobot();
				}
			}
			else {
				output.print(F("Invalid number of arguments for SPEED command. \r\n"));

			}
		}
		else if (cmdData.command == "TURN" || cmdData.command == "T") {
			if (cmdData.argCount == 1) {
				turn = cmdData.arguments[0];
				moveRobot(direction, targetSpeed, turn);
				if (turn > 0) {
					output.printf(F("Turn right %d \%"), turn);
				}
				else if (turn < 0) {

					output.printf(F("Turn left %d \%"), -1 * turn);
				}
				else {
					output.print(F("Go straight"));
				}
			}
			else {
				output.print(F("Invalid number of arguments for TURN command. \r\n"));

			}
		}
		else if (cmdData.command == "STOP" || cmdData.command == "S") {
			output.print(F("Stop robot movement!                               \r\n"));
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
				output.printf(F("Stored to NVM new kp = %s\r\n"), floatToString(pidPreferences.kp, 1, 3));
			}
			else if (cmdData.argCount == 2) {
				pidPreferences.kp = cmdData.arguments[0];
				pidPreferences.ki = cmdData.arguments[1];
				preferences.putBytes("pidPreferences", &pidPreferences, sizeof(pidPreferences));
				output.printf(F("Stored to NVM new kp = %s Ki = %s\r\n"), floatToString(pidPreferences.kp, 1, 3), floatToString(pidPreferences.ki, 1, 3));
			}
			else if (cmdData.argCount == 3) {
				pidPreferences.kp = cmdData.arguments[0];
				pidPreferences.ki = cmdData.arguments[1];
				pidPreferences.kd = cmdData.arguments[2];
				preferences.putBytes("pidPreferences", &pidPreferences, sizeof(pidPreferences));
				output.printf(F("Stored to NVM new kp = %s ki = %s kd = %s\r\n"), floatToString(pidPreferences.kp, 1, 3), floatToString(pidPreferences.kp, 1, 3), floatToString(pidPreferences.kd, 1, 3));
			}
			else {
				output.print(F("Invald parameters for PID command\r\n"));
			}
		}
		else if (cmdData.command == "POT" || cmdData.command == "O") {
			if (cmdData.argCount == 1) {
				potOn = cmdData.arguments[0];
				if (potOn) {
					potOn = true;
					output.print(F("Potentiometer speed control set to On\r\n"));
				}
				else {
					output.printf(F("Potentiometer speed control set to Off\r\n"));
				}
			}
			else {
				output.print(F("Invald parameters for POT command\r\n"));
			}
		}
		else if (cmdData.command == "SIMULATION" || cmdData.command == "SIM") {
			if (cmdData.argCount == 1) {
				simulationEnable = cmdData.arguments[0];
				if (simulationEnable) {
					simulationEnable = true;
					output.print(F("Simulation mode set On\r\n"));
				}
				else {
					simulationEnable = false;
					output.printf(F("Simulation mode set to Off\r\n"));
				}
			}
			else {
				output.print(F("Invald parameters for SIMULATION command\r\n"));
			}
		}
		else if (cmdData.command == "PID" || cmdData.command == "I") {
			if (cmdData.argCount == 1) {
				PIDon = cmdData.arguments[0];
				if (PIDon) {
					PIDon = true;
					output.print(F("PID speed regulation set to On\r\n"));
				}
				else
					output.printf(F("PID speed regulation set to Off\r\n"));
			}
			else {
				output.print(F("Invald parameters for PID command\r\n"));
			}
		}
		else if (cmdData.command == "LOG" || cmdData.command == "L") {
			if (cmdData.argCount == 1) {
				logWrite((int)cmdData.arguments[0]);
			}
			else if (cmdData.argCount == 0) {
				logWrite();
			}
			else {
				output.print(F("Invald parameters for LOG command\r\n"));
			}
		}
		else if (cmdData.command == "LED" || cmdData.command == "LE") {
			if (cmdData.argCount == 1) {
				led((uint8_t)cmdData.arguments[0]);
				(cmdData.arguments[0] == 0) ? output.printf(F("Flash LED light Off")) : output.printf(F("Flash LED light set to: %d\r\n"), (uint8_t)cmdData.arguments[0]);

			}
			else if (cmdData.argCount == 0) {
				led();
				output.print(F("Flash LED light Off\r\n"));
			}
			else {
				output.print(F("Invald parameters for LED command\r\n"));
			}
		}
#if defined(ESP32CAM) || defined(ESP32WROVERDEV)
		else if (cmdData.command == "VIDEO" || cmdData.command == "V") {
			if (cmdData.argCount == 1) {
				discardPhoto();
				if (cmdData.arguments[0] == 0) {
					sendPhotos = true;
					output.printf(F("Viede0 stream ON"));
				}
				else {
					output.printf(F("Sending %d Photos\r\n"), (uint8_t)cmdData.arguments[0]);
					sendPhotos = false;
					sendPhotoOverWebSocket((uint8_t)cmdData.arguments[0]);
				}
			}
			else if (cmdData.argCount == 0) {
				discardPhoto();
				sendPhotos = false;
				output.print(F("Video stream Off\r\n"));
			}
			else {
				output.print(F("Invald parameters forPHOTO command\r\n"));
			}
		}
#endif
		else {
			output.print(F("Unknown command \r\n"));
		}
		return output.getBuffer();
	} //end of orders

#ifdef ENABLE_WEBSOCKETS
	void processWebsockets() {
		if (newWebSocketMessage) {
			String input = lastWebSocketMessage;
			Serial.print(F("Received from websocket "));
			Serial.println(input);
			struct CommandData cmdData = parseCommand(input);
			Serial.print(F("Command: "));
			Serial.println(cmdData.command);
			Serial.print(F("Arguments: "));
			for (int i = 0; i < cmdData.argCount; i++) {
				Serial.print(cmdData.arguments[i]);
				Serial.print(" ");
			}
			Serial.println();
			ordersResponse = orders(cmdData);
			output.clear();
			newWebSocketMessage = false; // Reset the flag
			printToTelnet = true;
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
//#if defined(ESP32CAM) || defined(ESP32WROVERDEV)	//extern void setupCamera();
	//extern void captureAndSavePhoto();
	//extern void handleWebSocketMessage(uint8_t* payload, size_t length);
//#endif

	//****************************SETUP***********************************************************//
	void setup() {

#if defined(ESP32CAM) || defined(ESP32WROVERDEV)
		Serial.begin(115200, SERIAL_8N1, -1, 1); // TX active, RX inactive
		Wire.begin(sda, scl);
#ifdef ESP32CAM
		// Inicjalizacja PCF8574
		pcf8574.begin();

		// Setting all PCF8574 LOW
		for (int i = 0; i < 8; i++) {
			pcf8574.write(i, LOW);
		}
#endif
#else
		Serial.begin(115200);
#endif

#ifdef ENABLE_OLED
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
		startNetworkServices();
		//configure motors pins outputs
		pinMode(leftMotorPWM, OUTPUT);
		pinMode(rightMotorPWM, OUTPUT);
		digitalWrite(leftMotorPWM, LOW);
		digitalWrite(rightMotorPWM, LOW);
#if !defined(ESP32CAM) && !defined(ESP32WROVERDEV)
		pinMode(leftMotorDIR, OUTPUT);
		pinMode(rightMotorDIR, OUTPUT);
		digitalWrite(leftMotorDIR, LOW);
		digitalWrite(rightMotorDIR, LOW);

		// Configure potentiometers as inputs
		pinMode(leftPot, INPUT);
		pinMode(rightPot, INPUT);
		//Configure eqyalSpeed jumper as input
		pinMode(equalSpeed, INPUT_PULLUP);
#endif

		// Configure encoders
				//pinMode(leftEncoder, INPUT_PULLDOWN);
				//pinMode(rightEncoder, INPUT_PULLDOWN);
		pinMode(leftEncoder, INPUT_PULLUP);
		pinMode(rightEncoder, INPUT_PULLUP);
		lastTime = millis();

		// Attach interrupts for both sensors
		//attachInterrupt(leftEncoder, leftEncoderISR, CHANGE);
		//attachInterrupt(rightEncoder, rightEncoderISR, CHANGE);
		attachInterrupt(leftEncoder, handleStateChangeMotorLeft, RISING);
		attachInterrupt(leftEncoder, handleStateChangeMotorRight, RISING);

#if defined(ESP32CAM) || defined(ESP32WROVERDEV)
		setupCamera();
		delay(500);
#endif

		// Confogure ADC
		pinMode(batteryPin, INPUT);

		// Configure PWM channels in one step
		// Configure PWM channels in one step
		ledcAttachChannel(leftMotorPWM, motorsPwmFreq, pwmResolution, 1);   // Left Motor PWM: Channel 1
		ledcAttachChannel(rightMotorPWM, motorsPwmFreq, pwmResolution, 2);  // Right Motor PWM: Channel 2
		ledcAttachChannel(flashLed, ledPwmFreq, pwmResolution, 3);  // Flash LED PWM: Channel 3

		// Creating FreeRTOS task to control dc motors speed
#ifndef MOTOR_CONTROL_TASK_CORE
#define MOTOR_CONTROL_TASK_CORE 1
#endif
		xTaskCreatePinnedToCore(
			MotorControlTask,        // Task function
			"Motor Control",         // Task name
			8192,                    // Stock assigned for task in words - 4 bytes for 32 bits processors
			//4096,                    // Stock assigned for task in words - 4 bytes for 32 bits processors
			//2048,                    // Stock assigned for task in words - 4 bytes for 32 bits processors
			NULL,                    // Task parameters
			2,                       // Task priority
			//    NULL,               // Task Handle
			&MotorControlTaskHandle,  // Task Handle
			MOTOR_CONTROL_TASK_CORE
		);

		// Creating FreeRTOS task for telnet communication
#ifndef COMMUNICATION_TASK_CORE 
#define COMMUNICATION_TASK_CORE 1
#endif

		xTaskCreatePinnedToCore(
			CommunicationTask,  // Task function
			"Communication",   // Task name
			8192,              // Stock assigned for task in words - 4 bytes for 32 bits processors
			//4096,              // Stock assigned for task in words - 4 bytes for 32 bits processors
			//2048,              // Stock assigned for task in words - 4 bytes for 32 bits processors
			NULL,              // Task parameters
			1,                 // Task priority
			//    NULL                 // Task Handle
			&CommunicationTaskHandle,  // Task Handle
			COMMUNICATION_TASK_CORE
		);

#ifdef ENABLE_WEBSERVER
		// Serve the webpage
		server.on("/getWebSocketPort", []() {
			server.send(200, "text/plain", String(WEBSOCKETS_PORT)); // Send the websockets port as plain text
			});
		server.on("/", HTTP_GET, []() {
			server.send_P(200, "text/html", webpage); // Serve the HTML directly from PROGMEM
			});
#endif
	}  //*****************End of setup**********************************

	//*****************loop*********************************************
	void loop() {
		// #if defined(ESP32CAM) || defined(ESP32WROVERDEV)
		//		sendPhotoOverWebSocket();
		//#endif
				//Nothing here. Everything in Rtos task.
				//Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
				//Serial.printf("Total PSRAM: %d bytes\n", ESP.getPsramSize());
				//Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
				//delay(1000);
				//vTaskDelay(pdMS_TO_TICKS(1000));  //Task delay 1s
	}
	//*****************End of loop**************************************
