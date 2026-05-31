/*
  ESP32-CAM Video Streamer & Controller for Smart Assistive Wheelchair
  Compatible with AI-Thinker ESP32-CAM board.
  
  This code:
  1. Connects to the local WiFi network.
  2. Initializes the OV2640 camera.
  3. Launches an MJPEG video stream on port 80 (/stream).
  4. Provides HTTP APIs for camera control (/control) and status (/status) to toggle the flashlight and camera tuning parameters.
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =========================
// WiFi Settings
// =========================
const char* ssid = "Hostel_WiFi";
const char* password = "wifi@HostRUSL";

// =========================
// AI Thinker ESP32-CAM Pins
// =========================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_LED_PIN      4  // Onboard bright white flash LED
#define STATUS_LED_PIN    33  // Onboard red status LED (Active Low)

// =========================
// Stream Server Configuration
// =========================
httpd_handle_t stream_httpd = NULL;

static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
static const char* STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return ESP_FAIL;
    }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    char part_buf[64];
    size_t hlen = snprintf(part_buf, 64, STREAM_PART, fb->len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) {
      break;
    }
  }

  return res;
}

// =========================
// Camera Control Endpoint (/control?var=X&val=Y)
// =========================
static esp_err_t cmd_handler(httpd_req_t *req) {
  char buf[128];
  char var[32] = {0};
  char val[32] = {0};

  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    if (httpd_query_key_value(buf, "var", var, sizeof(var)) == ESP_OK &&
        httpd_query_key_value(buf, "val", val, sizeof(val)) == ESP_OK) {
      
      int value = atoi(val);
      sensor_t * s = esp_camera_sensor_get();
      int res = 0;

      if (strcmp(var, "flash") == 0) {
        digitalWrite(FLASH_LED_PIN, value ? HIGH : LOW);
        Serial.printf("Flash set to %d\n", value);
      } 
      else if (strcmp(var, "framesize") == 0) {
        if (s) res = s->set_framesize(s, (framesize_t)value);
        Serial.printf("Framesize set to %d\n", value);
      } 
      else if (strcmp(var, "quality") == 0) {
        if (s) res = s->set_quality(s, value);
        Serial.printf("Quality set to %d\n", value);
      } 
      else if (strcmp(var, "contrast") == 0) {
        if (s) res = s->set_contrast(s, value);
        Serial.printf("Contrast set to %d\n", value);
      } 
      else if (strcmp(var, "brightness") == 0) {
        if (s) res = s->set_brightness(s, value);
        Serial.printf("Brightness set to %d\n", value);
      } 
      else if (strcmp(var, "vflip") == 0) {
        if (s) res = s->set_vflip(s, value);
        Serial.printf("Vflip set to %d\n", value);
      } 
      else if (strcmp(var, "hmirror") == 0) {
        if (s) res = s->set_hmirror(s, value);
        Serial.printf("Hmirror set to %d\n", value);
      } 
      else {
        res = -1;
      }

      if (res != 0) {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send_500(req);
      }
    }
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// =========================
// Status Query Endpoint (/status)
// =========================
static esp_err_t status_handler(httpd_req_t *req) {
  char json_response[192];
  sensor_t * s = esp_camera_sensor_get();
  int flash_val = digitalRead(FLASH_LED_PIN);
  
  if (s) {
    sprintf(json_response, 
      "{\"flash\":%d,\"framesize\":%d,\"quality\":%d,\"contrast\":%d,\"brightness\":%d,\"vflip\":%d,\"hmirror\":%d}",
      flash_val, s->status.framesize, s->status.quality, s->status.contrast, s->status.brightness, s->status.vflip, s->status.hmirror
    );
  } else {
    sprintf(json_response, "{\"error\":\"Sensor not found\"}");
  }
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

// =========================
// Start Server
// =========================
void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/control",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t status_uri = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = status_handler,
    .user_ctx  = NULL
  };

  Serial.println("Starting camera stream server on port 80...");
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &cmd_uri);
    httpd_register_uri_handler(stream_httpd, &status_uri);
    Serial.println("Server started successfully!");
  } else {
    Serial.println("Failed to start server.");
  }
}

// =========================
// Arduino Setup
// =========================
void setup() {
  // Disable brownout detector to prevent reset loops from voltage drops during WiFi startup
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1000);
  Serial.println("\nESP32-CAM Initializing...");

  // Setup Flash and Status LEDs
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW); // Flash off by default
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH); // Status off (Active Low)

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Camera initialization
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (true) {
      digitalWrite(STATUS_LED_PIN, LOW); // Rapid status blink on fail
      delay(100);
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(100);
    }
  }

  // Connect to WiFi
  digitalWrite(STATUS_LED_PIN, LOW); // LED ON during WiFi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  digitalWrite(STATUS_LED_PIN, HIGH); // LED OFF when connected
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32-CAM Local IP Address: http://");
  Serial.println(WiFi.localIP());
  Serial.print("Video Stream URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");

  // Start the video streaming server
  startServer();
}

void loop() {
  delay(10000);
}
