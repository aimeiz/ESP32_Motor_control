#ifndef DEBUG
#define DEBUG Serial
#endif
//#ifdef ESP32
#include <WiFi.h>
#include <ESPmDNS.h>
//#else
//#include <ESP8266WiFi.h>
//#include <ESP8266mDNS.h>
//#endif

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#if defined TELNET
WiFiServer telnetServer(23);  // Port 23 is commonly used for Telnet
WiFiClient telnetClient;
#endif

#if defined(ESP32_RTOS) && defined(ESP32)
TaskHandle_t OTATaskHandle;  //Handle for OTA

void ota_handle(void* parameter) {
  for (;;) {
    if ((WiFi.status() == WL_CONNECTED)) {
      ArduinoOTA.handle();
      vTaskDelay(3500);
      //		delay(3500);
    } else
      ESP.restart();
  }
}
#endif

void setupOTA(const char* nameprefix, const char* ssid, const char* password, const char* OTApassword = "") {
  // Configure the hostname
  Serial.println(F("Invocation of setupOTA"));
  uint16_t maxlen = strlen(nameprefix) + 7;
  char* fullhostname = new char[maxlen];
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(fullhostname, maxlen, "%s-%02x%02x%02x", nameprefix, mac[3], mac[4], mac[5]);
  ArduinoOTA.setHostname(fullhostname);
  delete[] fullhostname;
  // Configure and start the WiFi station
  WiFi.mode(WIFI_STA);
  Serial.println(F("Set mode to WIFI_STA"));
  WiFi.begin(ssid, password);

  // Wait for connection
  Serial.print(F("Waiting for WiFi connection to "));
  Serial.println(ssid);
#if defined OLED
  delay(2000);
  display.clearDisplay();
  display.setCursor(0, 8);
  display.print(F("Trying "));
  display.println(ssid);
  display.display();
#endif
  //  while (WiFi.status() != WL_CONNECTED) {
  for (int i = 0; (i < 20) && (WiFi.status() != WL_CONNECTED); i++) {
    //  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(F("."));
    delay(1000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("\nWiFi failed to connect!!!"));
#if defined OLED
    display.println(F("Failed"));
    display.display();
#endif
    delay(2000);
    ESP.restart();
  }
  Serial.print(F("\nWifi connected "));
  Serial.print(WiFi.localIP());
#if defined OLED
  display.clearDisplay();
  display.setCursor(0, TEXTFIRSTROW);
  display.print(F("Connected to "));
  display.println(ssid);
  display.print("IP: ");
  display.println(WiFi.localIP());
  display.display();
#endif
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232); // Use 8266 port if you are working in Sloeber IDE, it is fixed there and not adjustable


  // No authentication by default
  //  ArduinoOTA.setPassword("admin");
  //ArduinoOTA.setPassword("vaillant"); //Password for Owen Reset
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  if (OTApassword != "")
    ArduinoOTA.setPassword(OTApassword);  //Password for Owen Reset
  ArduinoOTA.onStart([]() {
    //NOTE: make .detach() here for all functions called by Ticker.h library - not to interrupt transfer process in any way.

    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else  // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    DEBUG.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    DEBUG.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) DEBUG.println("\nAuth Failed");
    else if (error == OTA_BEGIN_ERROR) DEBUG.println("\nBegin Failed");
    else if (error == OTA_CONNECT_ERROR) DEBUG.println("\nConnect Failed");
    else if (error == OTA_RECEIVE_ERROR) DEBUG.println("\nReceive Failed");
    else if (error == OTA_END_ERROR) DEBUG.println("\nEnd Failed");
  });

  ArduinoOTA.begin();
#if defined TELNET
  telnetServer.begin();              // Start the Telnet server
  telnetServer.setNoDelay(true);     // Reduce latency for real-time communication
#endif
  DEBUG.println("OTA Initialized");
  DEBUG.print("IP address: ");
  DEBUG.println(WiFi.localIP());

//#if defined(ESP32_RTOS) && defined(ESP32)
#if defined(ESP32_RTOS) && defined(ESP32)
  xTaskCreatePinnedToCore(
    ota_handle,         /* Task function. */
    "OTA_HANDLE",       /* String with name of task. */
    4096,              /* Stock assigned for task in words - 4 bytes for 32 bits processors */
    NULL,               /* Parameter passed as input of the task */
    1,                  /* Priority of the task. */
    &OTATaskHandle,     /* Task handle. */
    0                   /* ESP32 core for task separate from core 1 used for arduino tasks */
  );
#endif
}
