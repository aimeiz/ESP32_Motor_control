#pragma once
class Output {
private:
    static String buffer; // Static data buffer

public:
    // printf with `F()` support
    static void printf(const __FlashStringHelper* format, ...) {
        char temp[256]; // Temporary buffer
        va_list args;
        va_start(args, format);
        vsnprintf_P(temp, sizeof(temp), reinterpret_cast<const char*>(format), args); //Data formatting
        buffer += temp; // Add data to buffer
    }

    // printf without `F()` support
    static void printf(const char* format, ...) {
        char temp[256]; // Temporary buffer
        va_list args;
        va_start(args, format);
        vsnprintf(temp, sizeof(temp), format, args); // Data formatting
        va_end(args);
        buffer += temp; // Add data to buffer
    }

    // TelnetStream2.print replacement
    static void print(const String& text) {
        buffer += text;
    }

    // TelnetStream2.println replacement
    static void println(const String& text) {
        buffer += text + "\r\n";
    }

    // float support
    static void printFloat(float value, int precision = 2) {
        buffer += String(value, precision);
    }

    // Pobranie zawartoœci bufora
    static String getBuffer() {
        return buffer;
    }

    // Wyczyœ bufor
    static void clear() {
        buffer = "";
    }
};

// Definicja statycznej zmiennej poza klas¹
String Output::buffer = "";
