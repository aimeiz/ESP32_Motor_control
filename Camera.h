#pragma once
//Support functions for ESP32CAM camera.
#include <esp_camera.h>
#include <FS.h>
#include <SPIFFS.h>
void setupCamera() {
	camera_config_t config;
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


//void handleWebSocketMessage(uint8_t* payload, size_t length) {
//    String message = String((char*)payload);
//
//    if (message == "GET_PHOTO") {
//        sendPhotoOverWebSocket();
//    }
//    else {
//        // Handle other commands or data requests
//    }
//}

//void cameraSetup() {
//    webSocket.begin();
//    webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
//        if (type == WStype_TEXT) {
//            handleWebSocketMessage(payload, length);
//        }
//        });
//}
