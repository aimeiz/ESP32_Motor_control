//void displayTelnet() { // Table for non ANSI terminal
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

//bool displayPaused = false;  // table display control variable

void displayStaticTable() {
	//if (!printToTelnet) return;
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
	//if (!printToTelnet) return;
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
