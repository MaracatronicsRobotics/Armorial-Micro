#include <WiFi.h>
#include <esp_now.h>

/*MOTOR 1
  gpio 15
  gpio 16

  MOTOR 2
  gpio 17
  gpio 18
*/

unsigned long prevTime;
float angularSpeedWL = 0, angularSpeedWR = 0, pulsesPerRevolution = 7,
      gearRatio = 50, encoderResolution = 10000;
int pinc1 = 15, pinc2 = 16;
int pinc12 = 17, pinc22 = 18;
volatile int encoderCountWL = 0;
volatile int encoderCountWR = 0;

const float L = 0.075; // robot length
const float d = 0.053; // wheel diameter

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to receive data
// Must match the sender structure
// Feedback packet
typedef struct {
  char control;
  uint8_t batteryPercentage;
  bool infraRedStatus;
  float vw1, vw2, vw3, vw4;
  float vw1_encoder, vw2_encoder, vw3_encoder, vw4_encoder;
  uint64_t timestamp;
  uint16_t crc;
} FeedbackPacket;

// Control packet
typedef struct {
  char control;
  float vw1, vw2, vw3, vw4;
  char solenoidPower;
  uint16_t crc;
} ControlPacket;

#define in1 12
#define in2 13
#define in3 2
#define in4 26
#define enablePin 27

// Create a struct_vss_package called package
ControlPacket controlPacket;
FeedbackPacket feedbackPacket;

esp_now_peer_info_t peerInfo;

void handleEncoderWL() {
  int pinState1 = digitalRead(pinc1);
  int pinState2 = digitalRead(pinc2);

  if (pinState1 == pinState2)
    encoderCountWL++;
  else
    encoderCountWL--;
}

void handleEncoderWR() {
  int pinState1 = digitalRead(pinc12);
  int pinState2 = digitalRead(pinc22);

  if (pinState1 == pinState2)
    encoderCountWR++;
  else
    encoderCountWR--;
}

void drive(float vl, float vr) {
  vl = (vl / 5.0) * 255;
  vr = (vr / 5.0) * 255;

  // left wheel control
  if (vl > 0) {
    ledcWrite(1, (int)vl);
    ledcWrite(2, 0);
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, (int)-vl);
  }

  // right wheel control
  if (vr > 0) {
    ledcWrite(3, (int)vr);
    ledcWrite(4, 0);
  } else {
    ledcWrite(3, 0);
    ledcWrite(4, (int)-vr);
  }
}

// callback when feedback is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Feedback Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" :
  // "Delivery Fail");
}

// callback function that will be executed when data is received
/// TODO: validate CRC
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&controlPacket, incomingData, sizeof(ControlPacket));

  digitalWrite(LED_BUILTIN, HIGH);

  //  Serial.print(len);
  //  Serial.println(" BYTES RECEIVED");
  //
  Serial.println(controlPacket.control, BIN);
  Serial.println(controlPacket.vw1);
  Serial.println(controlPacket.vw2);
  Serial.println(controlPacket.vw3);
  Serial.println(controlPacket.vw4);
  Serial.println((int)controlPacket.solenoidPower);
  Serial.println(controlPacket.crc, BIN);
}

void setup() {
  // enable H bridge
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

  prevTime = micros();

  // define pwm pins
  ledcAttachPin(in1, 1);
  ledcAttachPin(in2, 2);
  ledcAttachPin(in3, 3);
  ledcAttachPin(in4, 4);

  for (int i = 1; i <= 4; i++)
    ledcSetup(i, 1000, 8); // 1000 Hz with 8bit resolution

  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  pinMode(pinc1, INPUT_PULLUP);
  pinMode(pinc2, INPUT_PULLUP);
  pinMode(pinc12, INPUT_PULLUP);
  pinMode(pinc22, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pinc1), handleEncoderWL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinc12), handleEncoderWR, CHANGE);

  // Init ESP-NOW
  ESP_ERROR_CHECK(esp_now_init());

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  ESP_ERROR_CHECK(esp_now_register_send_cb(OnDataSent));
  ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
}

void loop() {
  unsigned long currentTime = micros();
  unsigned long elapsedTime = currentTime - prevTime;

  if (elapsedTime >= encoderResolution) {
    angularSpeedWL =
        (2 * PI * encoderCountWL) /
        (pulsesPerRevolution * gearRatio * (elapsedTime / 1000000.0f));
    angularSpeedWR =
        (2 * PI * encoderCountWR) /
        (pulsesPerRevolution * gearRatio * (elapsedTime / 1000000.0f));
    encoderCountWL = 0;
    encoderCountWR = 0;
    prevTime = currentTime;

    /// TODO: compute CRC, get InfraRed and battery percentage
    feedbackPacket.control = controlPacket.control;
    feedbackPacket.batteryPercentage = 0;
    feedbackPacket.infraRedStatus = false;
    feedbackPacket.vw1_encoder = angularSpeedWR;
    feedbackPacket.vw1 = controlPacket.vw1;
    feedbackPacket.vw2_encoder = angularSpeedWL;
    feedbackPacket.vw2 = controlPacket.vw2;
    feedbackPacket.timestamp = esp_timer_get_time();
    digitalWrite(LED_BUILTIN, LOW);

    esp_err_t result = esp_now_send(
        broadcastAddress, (uint8_t *)&feedbackPacket, sizeof(FeedbackPacket));

    if (result == ESP_OK) {
      // Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
    Serial.flush();
  }

  drive(controlPacket.vw1, controlPacket.vw2);
}