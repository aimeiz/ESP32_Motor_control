
void displayStaticTable() {
	//if (!printToTelnet) return;
	TelnetStream2.print(F("\033[2J"));  // Clears screen
	TelnetStream2.print(F("\033[H"));   // Move cursor to start

	// Górna ramka
	TelnetStream2.print(F("\033[32m+------------------------------------------------------+\r\n"));
	TelnetStream2.print(F("|                    SYSTEM STATUS TABLE               |\r\n"));
	TelnetStream2.print(F("+------------------------------------------------------+\r\n"));

	// Left Motor - ŻÓŁTY
	TelnetStream2.print(F("\033[33m"));  // Yellow color
	TelnetStream2.print(F("| Left Motor:   tgtSpeed:       Pwm:        Dir:       |\r\n"));
	TelnetStream2.print(F("|               Speed:          Way:                   |\r\n"));
	TelnetStream2.print(F("\033[0m"));   // Color reset

	// Podział
	TelnetStream2.print(F("\033[32m+------------------------------------------------------+\r\n"));

	// Right Motor - CYJAN
	TelnetStream2.print(F("\033[36m"));  // Cyyan color
	TelnetStream2.print(F("| Right Motor:  tgtSpeed:       Pwm:        Dir:       |\r\n"));
	TelnetStream2.print(F("|               Speed:          Way:                   |\r\n"));
	TelnetStream2.print(F("\033[0m"));   // Color reset

	// Podział
	TelnetStream2.print(F("\033[32m+------------------------------------------------------+\r\n"));

	// PID Values - ZIELONY
	TelnetStream2.print(F("\033[32m"));  // Green color
	TelnetStream2.print(F("| PID:          kp:             ki:         kd:        |\r\n"));
	TelnetStream2.print(F("+------------------------------------------------------+\r\n"));
	TelnetStream2.print(F("\033[33m"));  // Yellow color
	TelnetStream2.print(F("| Battery:      Vbat:                                  |\r\n"));
	TelnetStream2.print(F("\033[32m"));  // Green color
	TelnetStream2.print(F("+------------------------------------------------------+\r\n"));
	TelnetStream2.print(F("\033[0m"));   // Color reset
	TelnetStream2.flush();
}

void updateDynamicValues() {
	//if (!printToTelnet) return;
	// Left Motor - values
	TelnetStream2.print(F("\033[4;28H"));  // Row 4, column 28
	TelnetStream2.printf("%3d", targetLeftSpeed);
	TelnetStream2.print(F("\033[4;39H"));  // Row 4, column 39
	TelnetStream2.printf("%3d", leftMotorDirection == FORWARD ? leftMotorPwm : 255 - leftMotorPwm);
	TelnetStream2.print(F("\033[4;51H"));  // Row 4, column 51
	TelnetStream2.print(leftMotorDirection == FORWARD ? "FWD" : "REV");

	TelnetStream2.print(F("\033[5;28H"));  // Row 5, column 28
	TelnetStream2.printf("%3d", speedLeft);
	TelnetStream2.print(F("\033[5;39H"));  // Row 5, column 39
	TelnetStream2.printf("%15d", leftWay);

	// Right Motor - values
	TelnetStream2.print(F("\033[7;28H"));  // Row 7, column 28
	TelnetStream2.printf("%3d", targetRightSpeed);
	TelnetStream2.print(F("\033[7;39H"));  // Row 7, column 39
	TelnetStream2.printf("%3d", rightMotorDirection == FORWARD ? rightMotorPwm : 255 - rightMotorPwm);
	TelnetStream2.print(F("\033[7;51H"));  // Row 7, column 51
	TelnetStream2.print(rightMotorDirection == FORWARD ? "FWD" : "REV");

	TelnetStream2.print(F("\033[8;28H"));  // Row 8, column 28
	TelnetStream2.printf("%3d", speedRight);
	TelnetStream2.print(F("\033[8;39H"));  // Row 8, column 39
	TelnetStream2.printf("%15d", rightWay);

	// PID - wartości
	TelnetStream2.print(F("\033[10;26H"));  // Row 10, column 26
	TelnetStream2.print(floatToString(pidPreferences.kp, 1, 3));
	TelnetStream2.print(F("\033[10;37H"));  // Row 10, column 37
	TelnetStream2.print(floatToString(pidPreferences.ki, 1, 3));
	TelnetStream2.print(F("\033[10;49H"));  // Row 10, column 49
	TelnetStream2.print(floatToString(pidPreferences.kd, 1, 3));
	//Battery values
	TelnetStream2.print(F("\033[12;26H"));   //Row 12, column 26
	TelnetStream2.print(batteryVoltage > batteryLow ? F("\033[32m") : F("\033[31m"));  //Green or red colour
	TelnetStream2.print(floatToString(batteryVoltage, 1, 2));
	TelnetStream2.print(F(" V \033[32m"));
	TelnetStream2.print(F("\033[0m"));   // Reset koloru
	TelnetStream2.flush();

	// Przesuń kursor poniżej tabeli
	TelnetStream2.print(F("\033[14;1H"));  // Rowz 14, column 1
	TelnetStream2.print(F("> "));          // Command cursor
	TelnetStream2.flush();
}
