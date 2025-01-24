#pragma once
//Support functions for ESP32CAM camera.
#include <esp_camera.h>
#include <FS.h>
#include <SPIFFS.h>
void setupCamera() {
	camera_config_t config;
#ifdef ESP32CAM
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_pwdn = 32;
	config.pin_reset = -1;
	config.pin_xclk = 0;
	config.pin_sscb_sda = 26;
	config.pin_sscb_scl = 27;
	config.pin_d7 = 35;
	config.pin_d6 = 34;
	config.pin_d5 = 39;
	config.pin_d4 = 36;
	config.pin_d3 = 21;
	config.pin_d2 = 19;
	config.pin_d1 = 18;
	config.pin_d0 = 5;
	config.pin_vsync = 25;
	config.pin_href = 23;
	config.pin_pclk = 22;
	config.xclk_freq_hz = 20000000;
	config.pixel_format = PIXFORMAT_JPEG;
	config.frame_size = FRAMESIZE_QVGA;
	config.jpeg_quality = 12;
	config.fb_count = 1;
	config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

#elif defined ESP32WROVERDEV
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_pwdn = -1;        // PWDN niewykorzystywany
	config.pin_reset = -1;       // RESET niewykorzystywany
	config.pin_xclk = 21;        // XCLK
	config.pin_sscb_sda = 26;    // SDA (SIOD)
	config.pin_sscb_scl = 27;    // SCL (SIOC)
	config.pin_d7 = 35;          // Dane Y9
	config.pin_d6 = 34;          // Dane Y8
	config.pin_d5 = 39;          // Dane Y7
	config.pin_d4 = 36;          // Dane Y6
	config.pin_d3 = 19;          // Dane Y5
	config.pin_d2 = 18;          // Dane Y4
	config.pin_d1 = 5;           // Dane Y3
	config.pin_d0 = 4;           // Dane Y2
	config.pin_vsync = 25;       // VSYNC
	config.pin_href = 23;        // HREF
	config.pin_pclk = 22;        // PCLK
	config.xclk_freq_hz = 20000000;  // Czêstotliwoœæ XCLK
	config.pixel_format = PIXFORMAT_JPEG;  // Format pikseli
	config.frame_size = FRAMESIZE_QVGA;    // Rozmiar ramki
	config.jpeg_quality = 12;              // Jakoœæ JPEG
	config.fb_count = 1;                   // Liczba buforów ramek
	config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;  // Lokalizacja bufora ramek
#endif
	if (psramFound()) { Serial.println(F("PSRAM found")); }
		Serial.println("PSRAM found");
	if (esp_camera_init(&config) != ESP_OK) {
		Serial.println("Camera init failed!");
		return;
	}
	sensor_t* s = esp_camera_sensor_get();
	if (s != NULL) {
		s->set_vflip(s, 1); // Rotate 180 degrees
		s->set_hmirror(s, 1); // Optional: Mirror image
		s->set_brightness(s, 1); // Brightness levels: -2 to 2
	}
Serial.println("Camera initialized");
}

void sendPhotoOverWebSocket(int count = -1) {
	if (count < 0) return;
	camera_fb_t* fb = esp_camera_fb_get();

	if (!fb) {
		Serial.println("Camera capture failed");
		return;
	}

	webSocket.broadcastBIN(fb->buf, fb->len);
	esp_camera_fb_return(fb);
	//Serial.println("Photo sent");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
}

void discardPhoto() {
	camera_fb_t* fb = esp_camera_fb_get();
	if (fb) {
		esp_camera_fb_return(fb);
	}
}

void resetCamera() {
	esp_camera_deinit();
	setupCamera();
}


