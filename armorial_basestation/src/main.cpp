#include "packet.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_task_wdt.h>

#define BAUD_RATE 115200
#define WDT_TIMEOUT 3 // seconds
#define ENABLE_WDT true

char buffer[256] = {'\0'};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const char startDelimiter = '<';
const char endDelimiter = '>';
bool canSendFeedbacks = false;

ssl_pack package;
struct_feedback feedback;
esp_now_peer_info_t peerInfo;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&feedback, incomingData, sizeof(feedback));

  if (canSendFeedbacks) {
    Serial.write(&startDelimiter, sizeof(char));
    Serial.write((byte *)&feedback, sizeof(feedback));
    Serial.write(&endDelimiter, sizeof(char));
    Serial.flush();
  }
}

bool ProcessAndSendControl(char *data, long size) {
  String cvt;
  for (int i = 0; i < size; i++) {
    if (data[i] == startDelimiter || data[i] == endDelimiter) {
      cvt += data[i];
    } else {
      cvt += '0';
    }
  }

  bool parsedPacket = false;
  String strBuff;
  for (int i = 0; i < cvt.length(); i++) {
    if (cvt[i] == startDelimiter) {
      strBuff.clear();
    } else if (cvt[i] != endDelimiter) {
      strBuff += cvt[i];
    } else if (cvt[i] == endDelimiter) {
      if (strBuff.length() == sizeof(ssl_pack)) {
        char buff[sizeof(ssl_pack)];
        strBuff.toCharArray(buff, sizeof(ssl_pack));
        esp_now_send(broadcastAddress, (uint8_t *)&buff, sizeof(buff));
        parsedPacket = true;
      }
      strBuff.clear();
    }
  }

  return parsedPacket;
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setRxBufferSize(1024);
  while (!Serial) {
    delay(10);
  }

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Start watchdog
  if (ENABLE_WDT) {
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return;
  }
}

void loop() {
  // If serial is available and its available size of bigger than our expected
  // control packet size
  long sizeToRead = Serial.available();
  if (sizeToRead >= sizeof(ssl_pack)) {
    // Read the available bytes
    long serialSize = Serial.readBytes(buffer, sizeToRead);

    // Reset watch dog if could parse any packet properly
    if (ProcessAndSendControl(buffer, sizeToRead)) {
      if (ENABLE_WDT)
        esp_task_wdt_reset();
    }

    // Mark as can send feedbacks
    canSendFeedbacks = true;
  }
}