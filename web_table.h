#pragma once

#include"WebPage.html"
void websocketsUpdateDynamicValues() {
    // Create a JSON string to represent the data
    String json = "{";

    // Left Motor values
    json += "\"targetLeftSpeed\":" + String(targetLeftSpeed) + ",";
    json += "\"leftMotorPwm\":" + String(leftMotorDirection == FORWARD ? leftMotorPwm : 255 - leftMotorPwm) + ",";
    json += "\"leftMotorDirection\":\"" + String(leftMotorDirection == FORWARD ? "FWD" : "REV") + "\",";
    json += "\"speedLeft\":" + String(speedLeft) + ",";
    json += "\"leftWay\":" + String(leftWay) + ",";

    // Right Motor values
    json += "\"targetRightSpeed\":" + String(targetRightSpeed) + ",";
    json += "\"rightMotorPwm\":" + String(rightMotorDirection == FORWARD ? rightMotorPwm : 255 - rightMotorPwm) + ",";
    json += "\"rightMotorDirection\":\"" + String(rightMotorDirection == FORWARD ? "FWD" : "REV") + "\",";
    json += "\"speedRight\":" + String(speedRight) + ",";
    json += "\"rightWay\":" + String(rightWay) + ",";

    // PID values
    json += "\"PID\":\"" + String(PIDon == 1 ? "ON" : "OFF") + "\",";
    json += "\"kp\":" + String(pidPreferences.kp, 3) + ",";
    json += "\"ki\":" + String(pidPreferences.ki, 3) + ",";
    json += "\"kd\":" + String(pidPreferences.kd, 3) + ",";
    json += "\"errLeft\":" + String(errorLeft) + ",";
    json += "\"intLeft\":" + String(integralLeft) + ",";
    json += "\"derLeft\":" + String(derivativeLeft) + ",";
    json += "\"errRight\":" + String(errorRight) + ",";
    json += "\"intRight\":" + String(integralRight) + ",";
    json += "\"derRight\":" + String(derivativeRight) + ",";
    json += "\"otaHostname\":\"" + String(OTA_HOSTNAME) + "\",";

    // Battery values
    json += "\"batteryVoltage\":" + String(batteryVoltage, 2) + ",";
    json += "\"batteryStatus\":\"" + String(batteryVoltage > batteryLow ? "Normal" : "Low") + "\"";
    //json += "\"batteryLow\":" + String(batteryLow);
    json += "}";

    // Broadcast the JSON string to all connected WebSocket clients
    webSocket.broadcastTXT(json);
}
