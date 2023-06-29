#include "communication.h"
#include "espnow/espnow.h"
#include <esp_task_wdt.h>

#include <string>

#define BAUD_RATE 115200
#define RX_BUFFER_SIZE 1024

#define WDT_TIMEOUT 1 // seconds
#define ENABLE_WDT true

// Buffer control
std::string strBuff;

void setup() {
  // Start serial and wait for it to be ready
  Serial.begin(BAUD_RATE);
  Serial.setRxBufferSize(RX_BUFFER_SIZE);
  while (!Serial) {
    delay(10);
  }

  // Start watchdog
  if (ENABLE_WDT) {
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
  }

  // Start ESPNow
  InitEspNow();
}

void loop() {  
  while(Serial.available()) {
    int len = Serial.available();
    char buff[len];
    Serial.readBytes(buff, len);
    for(int i = 0; i < len; i++) {
      strBuff += buff[i];
    }
    
    if(ProcessPattern(strBuff)) {
      if(ENABLE_WDT) 
        esp_task_wdt_reset();
    }

    SetCanSendFeedbacks();
  }

  for(int i = 0; i < MAX_NUM_ROBOTS; i++) {
    if(PeerExists(i)) {
      if(feedbackBuffer[i].size()) {
        Serial.write(feedbackBuffer[i].c_str(), feedbackBuffer[i].size());
        feedbackBuffer[i].clear();
      }
    }
  }
}