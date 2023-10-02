#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include "esp_camera.h"

// WiFi settings
char ssid[] = "your_SSID";
char password[] = "your_PASSWORD";

// Pan-Tilt Servo pins
#define PAN_SERVO_PIN 12
#define TILT_SERVO_PIN 13

// Color to track (adjust these values as per your requirement)
int targetHue = 120;  // Hue value of the target color
int hueTolerance = 10;  // Tolerance for hue comparison

// Servo objects
Servo panServo;
Servo tiltServo;

// Camera object
camera_fb_t *fb = NULL;

// Web server object
AsyncWebServer server(80);

void setup() {
  // Serial port for debugging
  Serial.begin(115200);
  delay(1000);

  // Initialize servo objects
  panServo.attach(PAN_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi Connected!");

  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Init Camera
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Start streaming server
  startStreamingServer();

  // Start servo control task
  xTaskCreatePinnedToCore(servoControlTask, "servoControlTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // Nothing to do in the main loop
}

void startStreamingServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!fb) {
      request->send(200, "text/plain", "Camera capture failed!");
      return;
    }

    AsyncWebServerResponse *response = request->beginResponse_P(200, "image/jpeg", fb->buf, fb->len);
    request->send(response);
    fb = NULL;
  });

  server.begin();
  Serial.println("Streaming server started!");
}

void servoControlTask(void *pvParameters) {
  while (1) {
    if (!fb) {
      // Capture a new frame
      fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed!");
        continue;
      }
    }

    // Find the particle by color
    int targetX = -1;
    int targetY = -1;
    for (int i = 0; i < fb->len; i += 2) {
      uint16_t pixel = (fb->buf[i] << 8) | fb->buf[i + 1];
      int r = ((pixel >> 11) & 0x1F) << 3;
      int g = ((pixel >> 5) & 0x3F) << 2;
      int b = (pixel & 0x1F) << 3;
      float h, s, v;
      RGBtoHSV(r, g, b, h, s, v);

      if (abs(h - targetHue) <= hueTolerance) {
        int x = (i / 2) % fb->width;
        int y = (i / 2) / fb->width;

        if (targetX == -1 || targetY == -1) {
          targetX = x;
          targetY = y;
        } else {
          int currentDist = abs(targetX - (fb->width / 2)) + abs(targetY - (fb->height / 2));
          int newDist = abs(x - (fb->width / 2)) + abs(y - (fb->height / 2));
          if (newDist < currentDist) {
            targetX = x;
            targetY = y;
          }
        }
      }
    }

    // Move the servos to keep the particle in the center
    if (targetX != -1 && targetY != -1) {
      int panAngle = map(targetX, 0, fb->width, 0, 180);
      int tiltAngle = map(targetY, 0, fb->height, 0, 180);
      panServo.write(panAngle);
      tiltServo.write(tiltAngle);
    }

    // Delay to control the frame rate
    delay(100);
  }
}

void RGBtoHSV(int r, int g, int b, float &h, float &s, float &v) {
  float R = r / 255.0;
  float G = g / 255.0;
  float B = b / 255.0;
  float cmax = max(max(R, G), B);
  float cmin = min(min(R, G), B);
  float delta = cmax - cmin;

  if (delta == 0) {
    h = 0;
  } else if (cmax == R) {
    h = 60 * fmod(((G - B) / delta), 6);
  } else if (cmax == G) {
    h = 60 * (((B - R) / delta) + 2);
  } else if (cmax == B) {
    h = 60 * (((R - G) / delta) + 4);
  }

  if (cmax == 0) {
    s = 0;
  } else {
    s = delta / cmax;
  }

  v = cmax;
}
