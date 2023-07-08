#include <buzzer.h>
#include <communication.h>
#include <esp_task_wdt.h>
#include <leds.h>

#include <string>

#define BAUD_RATE 115200
#define RX_BUFFER_SIZE 1024

#define WDT_TIMEOUT 2      // seconds
#define FEEDBACK_TIMEOUT 2 // seconds
#define ENABLE_WDT true

// Buffer control
std::string strBuff;

void setup() {
  // Start serial and wait for it to be ready
  Serial.begin(BAUD_RATE);
  Serial.setRxBufferSize(RX_BUFFER_SIZE);
  // Serial.setTimeout(1000000000L);
  while (!Serial) {
    delay(10);
  }

  // Start watchdog
  if (ENABLE_WDT) {
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
  }

  // Start ESPNow
  Communication::setupEspNow();

  // Start leds
  Leds::setupLeds();

  // Start buzzer
  Buzzer::setupBuzzer();
}

void loop() {
  // Process serial
  if (Serial.available()) {
    int len = Serial.available();
    char buff[len];
    Serial.readBytes(buff, len);
    for (int i = 0; i < len; i++) {
      strBuff += buff[i];
    }

    if (Communication::processSerial(strBuff)) {
      if (ENABLE_WDT)
        esp_task_wdt_reset();
    }
  }

  // Process leds, feedback and feedback check
  uint64_t currentTime = esp_timer_get_time();
  for (int robotId = 0; robotId < MAX_NUM_ROBOTS; robotId++) {
    if (Peers::peerExists(robotId)) {
      Leds::turnOnRobot(robotId);
      std::string feedbackBuffer;
      if (Communication::getFeedbackFromRobot(robotId, feedbackBuffer)) {
        Serial.write(feedbackBuffer.c_str(), feedbackBuffer.size());
      } else {
        uint64_t passedTimeFromLastFeedback =
            currentTime - Communication::getLastFeedbackTimestamp(robotId);
        if (passedTimeFromLastFeedback >= FEEDBACK_TIMEOUT * 1E6) {
          if (Peers::deletePeer(robotId)) {
            Buzzer::connectSound();
          }
        }
      }
    } else {
      Leds::turnOffRobot(robotId);
    }
  }
  Communication::resetUnconnectedPeersFeedbackTimestamp(currentTime);
}