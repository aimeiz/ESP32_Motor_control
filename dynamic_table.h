#define TABLE_WIDTH 60
#define TABLE_HEIGHT 16
#define TABLE_TITTLE "SYSTEM STATUS TABLE "
const char* labels[] PROGMEM = {
	"Left Motor", "tgtSpeed:","Pwm:", "Gir:",
	"Speed:", "Way:",
	"Right Motor", "tgtSpeed:", "Pwm:", "Gir:",
	"Speed:", "Way:",
	"PID", "kp:", "ki:", "kd",
	"Battery:","Vbat:", "V"
	};
int labelCount = 19;
	
void displayDynamicTable(int width, int height, const char* title, const char* labels[], int labelCount) {
    // Znaki graficzne Extended ASCII lub standardowe
    const char topLeft = '+';
    const char topRight = '+';
    const char bottomLeft = '+';
    const char bottomRight = '+';
    const char horizontal = '-';
    const char vertical = '|';

    // Oblicz maksymalną szerokość kolumn
    int columnWidth = (width - 2) / labelCount;
    if (columnWidth < 3) columnWidth = 3; // Minimalna szerokość kolumny

    // Wyczyszczenie ekranu
    telnetClient.print(F("\033[2J")); // Clear screen
    telnetClient.print(F("\033[H"));  // Move cursor to top-left

    // Górna ramka
    telnetClient.print(F("\033[32m")); // Zielony kolor
    telnetClient.print(topLeft);
    for (int i = 0; i < width - 2; i++) telnetClient.print(horizontal);
    telnetClient.print(topRight);
    telnetClient.print(F("\r\n"));

    // Tytuł tabeli
    int titleStart = (width - strlen(title)) / 2;
    telnetClient.print(vertical);
    for (int i = 0; i < titleStart - 1; i++) telnetClient.print(' ');
    telnetClient.print(title);
    for (int i = 0; i < width - titleStart - strlen(title) - 1; i++) telnetClient.print(' ');
    telnetClient.print(vertical);
    telnetClient.print(F("\r\n"));

    // Separator
    telnetClient.print(topLeft);
    for (int i = 0; i < width - 2; i++) telnetClient.print(horizontal);
    telnetClient.print(topRight);
    telnetClient.print(F("\r\n"));

    // Wiersze etykiet i wartości
    for (int row = 0; row < height; row++) {
        telnetClient.print(vertical);
        for (int col = 0; col < labelCount; col++) {
            int contentStart = (columnWidth - strlen(labels[col])) / 2;
            for (int i = 0; i < contentStart; i++) telnetClient.print(' ');
            if (row == 0 && col < labelCount) {
                telnetClient.print(labels[col]);
            } else {
                telnetClient.print(' '); // Miejsce na wartości dynamiczne
            }
            for (int i = 0; i < columnWidth - contentStart - strlen(labels[col]); i++) telnetClient.print(' ');
        }
        telnetClient.print(vertical);
        telnetClient.print(F("\r\n"));
    }

    // Dolna ramka
    telnetClient.print(bottomLeft);
    for (int i = 0; i < width - 2; i++) telnetClient.print(horizontal);
    telnetClient.print(bottomRight);
    telnetClient.print(F("\033[0m\r\n")); // Reset kolorów
    telnetClient.flush();
}

void updateDynamicValues() {
    // Aktualizacja wartości dynamicznych w tabeli

    // Left Motor - wartości
    telnetClient.print(F("\033[4;28H"));  // Wiersz 4, kolumna 28
    telnetClient.printf("%3d", targetLeftSpeed);
    telnetClient.print(F("\033[4;39H"));  // Wiersz 4, kolumna 39
    telnetClient.printf("%3d", leftMotorDirection == FORWARD ? leftMotorPwm : 255 - leftMotorPwm);
    telnetClient.print(F("\033[4;51H"));  // Wiersz 4, kolumna 51
    telnetClient.print(leftMotorDirection == FORWARD ? "FWD" : "REV");

    telnetClient.print(F("\033[5;28H"));  // Wiersz 5, kolumna 28
    telnetClient.printf("%3d", speedLeft);
    telnetClient.print(F("\033[5;39H"));  // Wiersz 5, kolumna 39
    telnetClient.printf("%15d", leftWay);

    // Right Motor - wartości
    telnetClient.print(F("\033[7;28H"));  // Wiersz 7, kolumna 28
    telnetClient.printf("%3d", targetRightSpeed);
    telnetClient.print(F("\033[7;39H"));  // Wiersz 7, kolumna 39
    telnetClient.printf("%3d", rightMotorDirection == FORWARD ? rightMotorPwm : 255 - rightMotorPwm);
    telnetClient.print(F("\033[7;51H"));  // Wiersz 7, kolumna 51
    telnetClient.print(rightMotorDirection == FORWARD ? "FWD" : "REV");

    telnetClient.print(F("\033[8;28H"));  // Wiersz 8, kolumna 28
    telnetClient.printf("%3d", speedRight);
    telnetClient.print(F("\033[8;39H"));  // Wiersz 8, kolumna 39
    telnetClient.printf("%15d", rightWay);

    // PID - wartości
    telnetClient.print(F("\033[10;26H"));  // Wiersz 10, kolumna 26
    telnetClient.print(floatToString(pidPreferences.kp, 1, 3));
    telnetClient.print(F("\033[10;37H"));  // Wiersz 10, kolumna 37
    telnetClient.print(floatToString(pidPreferences.ki, 1, 3));
    telnetClient.print(F("\033[10;49H"));  // Wiersz 10, kolumna 49
    telnetClient.print(floatToString(pidPreferences.kd, 1, 3));

    // Battery - wartości
    telnetClient.print(F("\033[12;26H"));   // Wiersz 12, kolumna 26
    telnetClient.print(batteryVoltage > batteryLow ? F("\033[32m") : F("\033[31m"));  // Zielony lub czerwony kolor
    telnetClient.print(floatToString(batteryVoltage, 1, 2));
    telnetClient.print(F(" V \033[32m"));
    telnetClient.print(F("\033[0m"));   // Reset koloru

    // Przesuń kursor poniżej tabeli
    telnetClient.print(F("\033[14;1H"));  // Wiersz 14, kolumna 1
    telnetClient.print(F("> "));          // Kursor polecenia
    telnetClient.flush();
}
