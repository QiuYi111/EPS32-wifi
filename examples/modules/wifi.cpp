#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

#include <ESPmDNS.h>
#include "secrets.h"

#ifndef TCP_PORT
#define TCP_PORT 12345
#endif

#ifndef RGB_DATA_PIN
#define RGB_DATA_PIN 48
#endif

#ifndef RGB_POWER_PIN
#define RGB_POWER_PIN (-1)
#endif

constexpr uint8_t RGB_COUNT = 1;
constexpr uint32_t CLIENT_IDLE_TIMEOUT_MS = 30000;

WiFiServer server(TCP_PORT);
Adafruit_NeoPixel rgb_led(RGB_COUNT, RGB_DATA_PIN, NEO_GRB + NEO_KHZ800);

struct RgbColor {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

static RgbColor current_color;

static void apply_color(const RgbColor &c) {
  rgb_led.setPixelColor(0, rgb_led.Color(c.r, c.g, c.b));
  rgb_led.show();
}

static void connect_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // 低时延
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.print(".");
    if (millis() - t0 > 15000) {
      Serial.println("\n[WiFi] Timeout, reboot");
      ESP.restart();
    }
  }
  Serial.printf("\n[WiFi] OK  IP=%s  RSSI=%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  if (MDNS.begin("esp32s3-rgb")) {
    MDNS.addService("rgb", "tcp", TCP_PORT);  // _rgb._tcp.local
    Serial.println("[mDNS] esp32s3-rgb.local published");
  } else {
    Serial.println("[mDNS] start failed");
  }
}

static void handle_set(WiFiClient &client, int r, int g, int b) {
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
  current_color.r = static_cast<uint8_t>(r);
  current_color.g = static_cast<uint8_t>(g);
  current_color.b = static_cast<uint8_t>(b);
  apply_color(current_color);
  client.printf("OK %u %u %u\n", current_color.r, current_color.g, current_color.b);
  Serial.printf("[LED] SET R=%u G=%u B=%u\n", current_color.r, current_color.g, current_color.b);
}

static void handle_off(WiFiClient &client) {
  current_color.r = 0;
  current_color.g = 0;
  current_color.b = 0;
  apply_color(current_color);
  client.println("OK OFF");
  Serial.println("[LED] OFF");
}

static void handle_get(WiFiClient &client) {
  client.printf("COLOR %u %u %u\n", current_color.r, current_color.g, current_color.b);
}

static void dispatch_command(WiFiClient &client, const char *line) {
  if (line[0] == '\0') {
    return;
  }
  if (strncmp(line, "SET", 3) == 0) {
    int r = 0, g = 0, b = 0;
    int matched = sscanf(line + 3, "%d %d %d", &r, &g, &b);
    if (matched == 3) {
      handle_set(client, r, g, b);
    } else {
      client.println("ERR usage: SET <r> <g> <b>");
    }
    return;
  }
  if (strcmp(line, "OFF") == 0) {
    handle_off(client);
    return;
  }
  if (strcmp(line, "GET") == 0) {
    handle_get(client);
    return;
  }
  if (strcmp(line, "PING") == 0) {
    client.println("PONG");
    return;
  }
  client.println("ERR unknown");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  if (RGB_POWER_PIN >= 0) {
    pinMode(RGB_POWER_PIN, OUTPUT);
    digitalWrite(RGB_POWER_PIN, HIGH);
  }
  rgb_led.begin();
  rgb_led.setBrightness(64);
  apply_color(current_color);
  connect_wifi();
  server.begin();
  server.setNoDelay(true);
  Serial.printf("[TCP] RGB control listening :%d\n", TCP_PORT);
}

void loop() {
  WiFiClient client = server.available();
  if (!client) {
    delay(4);
    return;
  }

  client.setNoDelay(true);
  IPAddress rip = client.remoteIP();
  Serial.printf("[TCP] Client %s connected\n", rip.toString().c_str());

  static constexpr size_t BUF_LEN = 64;
  char line[BUF_LEN];
  size_t len = 0;
  uint32_t last_activity = millis();

  while (client.connected()) {
    while (client.available()) {
      int byte_in = client.read();
      if (byte_in < 0) {
        break;
      }
      char c = static_cast<char>(byte_in);
      last_activity = millis();

      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        line[len] = '\0';
        dispatch_command(client, line);
        len = 0;
        continue;
      }
      if (len < BUF_LEN - 1) {
        line[len++] = c;
      } else {
        client.println("ERR line too long");
        len = 0;
      }
    }

    if (!client.connected()) {
      break;
    }

    if (millis() - last_activity > CLIENT_IDLE_TIMEOUT_MS) {
      client.println("ERR timeout");
      Serial.println("[TCP] idle timeout");
      break;
    }

    delay(2);
  }

  client.stop();
  Serial.printf("[TCP] Client %s closed\n", rip.toString().c_str());
}
