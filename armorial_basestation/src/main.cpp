#include "communication.h"
#include "espnow/espnow.h"
#include <esp_task_wdt.h>

#define BAUD_RATE 115200
#define RX_BUFFER_SIZE 1024

#define WDT_TIMEOUT 1 // seconds
#define ENABLE_WDT true

// Buffer control
char buffer[RX_BUFFER_SIZE] = {'\0'};

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

  // Setup peers (TODO: check how to take peers from serial (?))
  InsertPeer(0, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});

  // Start ESPNow
  InitEspNow();
}

void loop() {
  // If serial is available and its available size of bigger than our expected
  // control packet size
  long sizeToRead = Serial.available();
  if (sizeToRead >= (sizeof(ControlPacket) + 2 * startDelimiter.length())) {
    // Read the available bytes
    long serialSize = Serial.readBytes(buffer, sizeToRead);

    // Reset watch dog if could parse any packet properly
    if (ProcessAndSendControl(buffer, sizeToRead)) {
      if (ENABLE_WDT)
        esp_task_wdt_reset();
    }

    // Mark as can send feedbacks
    SetCanSendFeedbacks();
  }
}