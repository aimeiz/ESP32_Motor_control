#pragma once
class OutputHandler {
private:
    String buffer; // Data buffer

public:
    // Obs³uga printf z `F()` dla sta³ych tekstów printf with F() for constant texts
    void printf(const __FlashStringHelper* format, ...) {
        char temp[256]; //temporary buffer
        va_list args;
        va_start(args, format);

        // Float support: conversion `%.2f` to String(value, 2)
        char* pos = strstr_P(reinterpret_cast<const char*>(format), "%.2f");
        if (pos != nullptr) {
            // float value service
            char beforeFloat[128] = { 0 };
            strncpy_P(beforeFloat, reinterpret_cast<const char*>(format), pos - reinterpret_cast<const char*>(format));
            float value = va_arg(args, double); // get float value
            String floatValue = String(value, 2); // float format to 2 decimal places
            snprintf(temp, sizeof(temp), beforeFloat);
            buffer += temp;
            buffer += floatValue;
            const char* afterFloat = pos + 4; // position after `%.2f`
            snprintf(temp, sizeof(temp), afterFloat, args);
            buffer += temp;
        }
        else {
            // Standard formatting if no `%.2f`
            vsnprintf_P(temp, sizeof(temp), reinterpret_cast<const char*>(format), args);
            buffer += temp;
        }

        va_end(args);
    }
//printf support without `F()`
    void printf(const char* format, ...) {
        char temp[256]; // temporary buffer
        va_list args;
        va_start(args, format);
        vsnprintf(temp, sizeof(temp), format, args); // data formatting
    }
    // TelnetStream2.print replacement
    void print(const String& text) {
        buffer += text;
    }

    // TelnetStream2.println replacement
    void println(const String& text) {
        buffer += text + "\n";
    }

    // float support
    void printFloat(float value, int precision = 2) {
        buffer += String(value, precision);
    }

    // get buffer content
    String getBuffer() const {
        return buffer;
    }

    // flush buffer
    void clear() {
        buffer = "";
    }
};
