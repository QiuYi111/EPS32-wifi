#include <Arduino.h>
#include <WiFi.h>

#include <ESPmDNS.h>
#include "secrets.h"

#ifndef TCP_PORT
#define TCP_PORT 12345
#endif

WiFiServer server(TCP_PORT);

static void connect_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);           // 低时延
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

  if (MDNS.begin("esp32s3-echo")) {
    MDNS.addService("echo", "tcp", TCP_PORT); // _echo._tcp.local
    Serial.println("[mDNS] esp32s3-echo.local published");
  } else {
    Serial.println("[mDNS] start failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  connect_wifi();
  server.begin();
  server.setNoDelay(true);
  Serial.printf("[TCP] Echo listening :%d\n", TCP_PORT);
}

void loop() {
  static uint32_t lastStat = millis();
  WiFiClient client = server.available();
  if (client) {
    client.setNoDelay(true);
    IPAddress rip = client.remoteIP();
    Serial.printf("[TCP] Client %s connected\n", rip.toString().c_str());

    uint8_t buf[1460];
    size_t total_rx = 0, total_tx = 0;
    uint32_t t0 = millis();

    while (client.connected()) {
      int avail = client.available();
      if (avail > 0) {
        int n = client.read(buf, min(avail, (int)sizeof(buf)));
        if (n > 0) {
          total_rx += n;
          int m = client.write(buf, n);
          total_tx += m;
        }
      } else {
        // 简易 keep-alive：避免空转占用 CPU
        delay(1);
      }
      // 10 秒无数据则断开
      if (millis() - lastStat > 10000 && (millis() - t0) > 10000 && total_rx == 0) {
        break;
      }
      if (millis() - lastStat > 2000) {
        lastStat = millis();
        Serial.printf("[TCP] RX=%u TX=%u bytes\n", (unsigned)total_rx, (unsigned)total_tx);
      }
      // 客户端主动断开
      if (!client.connected()) break;
    }

    client.stop();
    Serial.printf("[TCP] Client %s closed. RX=%u TX=%u\n",
                  rip.toString().c_str(), (unsigned)total_rx, (unsigned)total_tx);
  } else {
    delay(2);
  }
}
