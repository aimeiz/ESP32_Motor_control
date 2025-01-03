#pragma once

#include"WebPage.html"
extern String ordersResponse;
void websocketsUpdateDynamicValues() {
    // Create a JSON string to represent the data
    String json = "{";
    //responses
    if (!ordersResponse.isEmpty()) {
        // Ucieczka znaków specjalnych w ordersResponse
        ordersResponse.replace("\"", "\\\"");
        ordersResponse.replace("\n", "\\n");
        ordersResponse.replace("\r", "\\r");

        // Dodanie odpowiedzi do JSON-a
        json += "\"ordersResponse\":\"" + ordersResponse + "\",";
    }
    else {
        // Dodanie pustej wartoœci, jeœli brak odpowiedzi
        json += "\"ordersResponse\":\"\",";
    }
    ordersResponse = "";
        // Left Motor values
    json += "\"targetSpeed\":" + String(targetSpeed) + ",";
    json += "\"direction\":" + String(direction) + ",";
    json += "\"turn\":" + String(turn) + ",";
    json += "\"stopped\":" + String(stopped) + ",";
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

    // Battery values
    json += "\"batteryVoltage\":" + String(batteryVoltage, 2) + ",";
    json += "\"batteryStatus\":\"" + String(batteryVoltage > batteryLow ? "Normal" : "Low") + "\",";
    //json += "\"batteryLow\":" + String(batteryLow);
    json += "\"otaHostname\":\"" + String(OTA_HOSTNAME) + "\"";
    json += "}";

    // Broadcast the JSON string to all connected WebSocket clients
    //Serial.println(json);
    webSocket.broadcastTXT(json);
}
