//Works on board WEMOS D1 R32
#define VERSION "ESP32Motors_encoders_Rtos_OTA_Telnet_NVM_241113"
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
const int leftMotorDIR = 2;            // GPIO for Left Motor DIR
const int rightMotorDIR = 27;           // GPIO for Right Motor DIR
const int leftPot = 34;                 // GPIO for Left Potentiometer
const int rightPot = 35;                // GPIO for Right Potentiometer
const int leftEncoder = 32;             // GPIO for Left Encoder
const int rightEncoder = 25;            // GPIO for Right Encoder
const int equalSpeed = 19;              // GPIO for right speed = left speed adjusted by left speed potentiometer
const int batteryPin = 36;              //ADC Port for battery voltage measurement
const float batteryDivider = 0.003587;  //Resistor divider for battery measurements
const float batteryMin = 6.5;           //Minimal battery voltage. Below motors stop
bool potOn = false; //If true target speed set by potentiometers instead of commands.
//For PID function
struct PID {
  float kp;  //Proportionality factor. To be adjusted
  float ki;  //Integration factor.  To be adjusted
  float kd;  //Derivation factor.  To be adjusted
};
Preferences preferences;
bool PIDon = true;              //If true speed PID regulation is on otherwise off
PID pidPreferences = { 1.0, 0.5, 0.1 };  // Default PID parameters
float integralLeft = 0;        // Accumulated error (integral)
float integralRight = 0;       // Accumulated error (integral)
float previousErrorLeft = 0;   // Last error (for derivative)
float previousErrorRight = 0;  // Last error (for derivative)
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int speedLeft;   //Current wheel speed in pulses
int speedRight;  //Current wheel speed in pulses
// LEDC configuration
const int pwmFreq = 5000;    // PWM frequency in Hz
const int pwmResolution = 8;  // 8-bit resolution (0-255)

// Variables for motor speed and stabilization
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long leftEncoderWay = 0;
volatile long rightEncoderWay = 0;
int leftMotorPwm = 0;
int rightMotorPwm = 0;
#define FORWARD LOW
#define BACKWARD HIGH
int leftMotorDirection = LOW;   //LOW means forward HI means backward
int rightMotorDirection = LOW;  //LOW means forward HI means backward
unsigned long lastTime;
const unsigned long countPeriod = 50;  //Counting period in ms
const int maxSpeed = 1000;              // Maximum speed in pulses per second
const unsigned long commPeriod = 1000;  //Telnet communication period in ms
bool printToTelnet = false;
float batteryVoltage = 0;  //Stores battery voltage

TaskHandle_t MotorControlTaskHandle;  //Handle for MotorControlTask
TaskHandle_t CommunicationTaskHandle;  //Handle for MotorControlTask
//Interrupts handlers for optical encoders
static void IRAM_ATTR leftEncoderISR() {
  leftEncoderCount++;
}

static void IRAM_ATTR rightEncoderISR() {
  rightEncoderCount++;
}

#if defined TELNET
void displayTelnet() {
  telnetClient.print(F("tgtLtSpeed: "));
  telnetClient.print(targetLeftSpeed);
  telnetClient.print(F("\tLtPwm: "));
  telnetClient.print(leftMotorPwm);
  telnetClient.print(F("\tLtDir: "));
  telnetClient.print(leftMotorDirection);
  telnetClient.print(F("\tSpeed Lt: "));
  telnetClient.print(speedLeft);
  telnetClient.print(F("\t| tgtRtSpeed: "));
  telnetClient.print(targetRightSpeed);
  telnetClient.print(F("\tRtPwm: "));
  telnetClient.print(rightMotorPwm);
  telnetClient.print(F("\tRtDir: "));
  telnetClient.print(rightMotorDirection);
  telnetClient.print(F("\tSpeed Rt: "));
  telnetClient.print(speedRight);
  telnetClient.print("\tkp= ");
  telnetClient.print(pidPreferences.kp);
  telnetClient.print(" ki= ");
  telnetClient.print(pidPreferences.ki);
  telnetClient.print(" kd= ");
  telnetClient.print(pidPreferences.kd);
  telnetClient.print(F(" PID: "));
  telnetClient.print(PIDon);
  telnetClient.print(F(" Pot: "));
  telnetClient.print(potOn);
  telnetClient.print(F(" Bat Volts: "));
  telnetClient.print(batteryVoltage);
  telnetClient.print(F("\r\n"));
}
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
  Serial.print(F("\t| tgtRtSpeed: "));
  Serial.print(targetRightSpeed);
  Serial.print(F("\tRtPwm: "));
  Serial.print(rightMotorPwm);
  Serial.print(F("\tRtDir: "));
  Serial.print(rightMotorDirection);
  Serial.print(F("\tSpeed Rt: "));
  Serial.print(speedRight);
  Serial.printf(F("\tkp= %f ki= %f kd= %f"), pidPreferences.kp, pidPreferences.ki, pidPreferences.kd);
  Serial.print(F("\tBat Volts: "));
  Serial.print(batteryVoltage);
  Serial.print(F("\r\n"));
}
#endif

void CommunicationTask(void* pvParameters) {
  for (;;) {
 #if defined TELNET
      // Accept a new client
      if (telnetServer.hasClient()) {
        if (!telnetClient || !telnetClient.connected()) {
          if (telnetClient) telnetClient.stop();    // Disconnect previous client
          telnetClient = telnetServer.available();  // Accept new client
          Serial.println("New Telnet client connected");
          printMenu();
        }
        else {
          telnetServer.available().stop();  // Reject additional clients
        }
      }
      orders();
      if (printToTelnet) {
        displayTelnet();
      }
#else
      displaySerial();
#endif
    vTaskDelay(500 / portTICK_PERIOD_MS);  //Task delay 10ms
  }
}

void MotorControlTask(void* pvParameters) {
  for (;;) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastTime;
    if (elapsedTime >= countPeriod) {
      batteryVoltage = analogRead(batteryPin) * batteryDivider;  //Battery voltage in volts
      if (batteryVoltage < batteryMin) stopRobot();              // Stop robot if voltage is to low.
      speedLeft = (leftEncoderCount * (1000.0 / elapsedTime));
      speedRight = (rightEncoderCount * (1000.0 / elapsedTime));
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
      lastTime = currentTime;
    }
    // vTaskDelay(1);
    vTaskDelay(10 / portTICK_PERIOD_MS);  //Task delay 10ms
  }
}  //End of MotorControlTask

//Receive and optimize command from telnet terminal


// PID-based function to stabilize motor speed
int stabilizeSpeed(int targetSpeed, int currentSpeed, float * integral, float * previousError) {
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
  leftMotorDirection = FORWARD;
  rightMotorDirection = FORWARD;
  potOn = false; //Stop robot regardless on potentiometers setting
  digitalWrite(leftMotorDIR, FORWARD);
  digitalWrite(rightMotorDIR, FORWARD);
  leftMotorPwm = 0;
  leftMotorPwm = 0;
  ledcWrite(leftMotorPWM, 0);
  ledcWrite(rightMotorPWM, 0);
  targetLeftSpeed = 0;
  targetRightSpeed = 0;
}

void printMenu() {
#if defined TELNET
  telnetClient.print(F("Robot commands\n\r[F]ORWARD <speed> <distance>\n\r[B]ACKWARD <speed> <distance>\n\r[S]TOP\n\r[D]ISPLAY\n\r"));
  telnetClient.print(F("[M]ENU - print menu\n\r[P]ID <kp> <ki> <kd> - Enter PID factors\n\rP[O]T <0/1> - Potentiometers speeds control Off / On\n\r"));
  telnetClient.print(F("P[I]D <0/1> - PID regulation Off / On\n\r"));
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
        telnetClient.printf(F("Moving forward indefinitely at %d mm/s.\n\r"), mSpeed);
        // Add logic to start moving forward at the given speed
        moveRobot(FORWARD, mSpeed);
      }
      else if (cmdData.argCount == 2) {
        int mSpeed = cmdData.arguments[0];
        int distance = cmdData.arguments[1];
        telnetClient.printf(F("Moving forward at %d mm/s for a distance of %d mm.\n\r"), mSpeed, distance);
        // Add logic to move forward at the given speed and stop after the given distance
        moveRobot(FORWARD, mSpeed, distance);
      }
      else {
        telnetClient.println(F("Invalid number of arguments for FORWARD command."));
      }
    }
    else if (cmdData.command == "BACKWARD" || cmdData.command == "B") {
      if (cmdData.argCount == 1) {
        int mSpeed = cmdData.arguments[0];
        telnetClient.printf(F("Moving backward indefinitely at %d mm/s.\n\r"), mSpeed);
        // Add logic to start moving backward at the given speed
        moveRobot(BACKWARD, mSpeed);
      }
      else if (cmdData.argCount == 2) {
        int mSpeed = cmdData.arguments[0];
        int distance = cmdData.arguments[1];
        telnetClient.printf(F("Moving backward at %d mm/s for a distance of %d mm.\n\r"), mSpeed, distance);
        // Add logic to move backward at the given speed and stop after the given distance
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
    }
    else if ((cmdData.command == "MENU" || cmdData.command == "M")) {
      printToTelnet = false;
      printMenu();
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
        telnetClient.print(F("Stored to NVM new kp = "));
        telnetClient.print(pidPreferences.kp);
        telnetClient.print(F(" ki = "));
        telnetClient.print(pidPreferences.ki);
        telnetClient.print(F(" kd = "));
        telnetClient.print(pidPreferences.kd);
        telnetClient.print(F("\n\r"));
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
  display.println(VERSION);
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
  pinMode(leftEncoder, INPUT_PULLUP);
  pinMode(rightEncoder, INPUT_PULLUP);
  attachInterrupt(leftEncoder, leftEncoderISR, CHANGE);
  attachInterrupt(rightEncoder, rightEncoderISR, CHANGE);
  lastTime = millis();

  // Confogure ADC
  pinMode(batteryPin, INPUT);

  // Configure PWM channels in one step
  ledcAttachChannel(leftMotorPWM, pwmFreq, pwmResolution, 0);   // Left Motor PWM: Channel 0
  ledcAttachChannel(rightMotorPWM, pwmFreq, pwmResolution, 1);  // Right Motor PWM: Channel 1

  // Creating FreeRTOS task to control dc motors speed
  xTaskCreate(
    MotorControlTask,  // Task function
    "Motor Control",   // Task name
    2048,              // Stock assigned for task in words - 4 bytes for 32 bits processors
    NULL,              // Task parameters
    2,                 // Task priority
    //    NULL                 // Task Handle
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
