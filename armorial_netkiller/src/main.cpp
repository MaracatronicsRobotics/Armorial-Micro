#include "communication.h"
#include "espnow/espnow.h"
#include <esp_task_wdt.h>

#define BAUD_RATE 115200
#define RX_BUFFER_SIZE 1024

#define WDT_TIMEOUT 1 // seconds
#define ENABLE_WDT false

// Struct with same size of control packet, but with garbage data
typedef struct {
  char garbageControl;
  float garbageVw1, garbageVw2, garbageVw3, garbageVw4;
  uint16_t garbageCrc;
} GarbagePacket;

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
  uint32_t CONFIG_WIFI_CHANNEL = esp_random() % 1;
  InitEspNow(CONFIG_WIFI_CHANNEL);
}

void loop() {
  SendGarbage();
  ReconfigureEspNow();
}