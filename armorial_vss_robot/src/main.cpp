#include <esp_now.h>
#include <WiFi.h>

/*MOTOR 1
  gpio 15
  gpio 16

  MOTOR 2
  gpio 17
  gpio 18
*/

unsigned long prevTime;
float angularSpeedWL = 0, angularSpeedWR = 0, pulsesPerRevolution = 7, gearRatio = 50, encoderResolution = 15000;
int pinc1 = 15, pinc2 = 16;
int pinc12 = 17, pinc22 = 18;
volatile int encoderCountWL = 0;
volatile int encoderCountWR = 0;

const float L = 0.075;  //robot length
const float d = 0.053;  //wheel diameter


// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to receive data
// Must match the sender structure
typedef struct ssl_pack {
  char control;
  float wv1, wv2, wv3, wv4;
  char solenoid;
  uint16_t crc;
} ssl_pack;

typedef struct struct_feedback {
  char control;
  float wvl, wvl_encoder;
  float wvr, wvr_encoder;
  uint16_t crc;
} struct_feedback;

/*
  FEEDBACK STRUCTURE 
  battery   (1 byte)
  control   (1 byte) including id
  w1      (4 byte)
  w2      (4 byte)
  w3      (4 byte)
  w4      (4 byte)
  w1 encoder(4 byte)
  w2 encoder(4 byte)
  w3 encoder(4 byte)
  w4 encoder(4 byte)
  crc     (2 byte)
*/

#define in1 12
#define in2 13
#define in3 2
#define in4 26
#define enablePin 27

// Create a struct_vss_package called package
ssl_pack package;
struct_feedback feedback;

esp_now_peer_info_t peerInfo;


void handleEncoderWL() {
  int pinState1 = digitalRead(pinc1);
  int pinState2 = digitalRead(pinc2);

  if (pinState1 == pinState2) encoderCountWL++;
  else encoderCountWL--;
}

void handleEncoderWR() {
  int pinState1 = digitalRead(pinc12);
  int pinState2 = digitalRead(pinc22);

  if (pinState1 == pinState2) encoderCountWR++;
  else encoderCountWR--;
}

void drive(float vl, float vr) {
  vl = (vl / 5.0) * 255;
  vr = (vr / 5.0) * 255;
  
  //left wheel control
  if (vl > 0) {
     ledcWrite(1, (int) vl);
     ledcWrite(2, 0);
  }
  else {
     ledcWrite(1, 0);
     ledcWrite(2, (int) -vl);
  }

  // right wheel control
  if (vr > 0) {
     ledcWrite(3,(int) vr);
     ledcWrite(4, 0);
  }
  else {
     ledcWrite(3, 0);
     ledcWrite(4, (int) -vr);
  }
}

// callback when feedback is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Feedback Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&package, incomingData, sizeof(ssl_pack));
  feedback.control = package.control;
  feedback.crc = package.crc;


    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &feedback, sizeof(feedback));
  
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    Serial.flush();

//  Serial.print(len);
//  Serial.println(" BYTES RECEIVED");
//
//  Serial.println(package.control, BIN);
//  Serial.println(package.wv1);
//  Serial.println(package.wv2);
//  Serial.println(package.wv3);
//  Serial.println(package.wv4);
//  Serial.println((int) package.solenoid);
//  Serial.println(package.crc, BIN);

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

  for (int i = 1; i <= 4; i++) ledcSetup(i, 1000, 8); // 1000 Hz with 8bit resolution
  
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  pinMode(pinc1, INPUT_PULLUP);
  pinMode(pinc2, INPUT_PULLUP);
  pinMode(pinc12, INPUT_PULLUP);
  pinMode(pinc22, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinc1), handleEncoderWL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinc12), handleEncoderWR, CHANGE);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

//  //Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  //feedback stuff ######## CHANGE LATER ########
  
  

  /*##########################################################*/

  unsigned long currentTime = micros();
  unsigned long elapsedTime = currentTime - prevTime;

  if(elapsedTime >= encoderResolution) {
    angularSpeedWL = (2 * PI * encoderCountWL) / (pulsesPerRevolution * gearRatio * (elapsedTime / 1000000.0f));
    angularSpeedWR = (2 * PI * encoderCountWR) / (pulsesPerRevolution * gearRatio * (elapsedTime / 1000000.0f));
    encoderCountWL = 0;
    encoderCountWR = 0;
    prevTime = currentTime;
    
    feedback.wvl_encoder = angularSpeedWL;
    feedback.wvl = package.wv1;
    feedback.wvr_encoder = angularSpeedWR;
    feedback.wvr = package.wv2;

    /*
    Serial.print("wvl_encoder = ");
    Serial.println(feedback.wvl_encoder);
    Serial.print("wvl = ");
    Serial.println(feedback.wvl);
    Serial.print("wvr_encoder = ");
    Serial.println(feedback.wvr_encoder);
    Serial.print("wvr = ");
    Serial.println(feedback.wvr);
    */

    feedback.wvr_encoder = max(0.01f, feedback.wvr_encoder);
    feedback.wvr = max(0.01f, feedback.wvr);
    feedback.wvl = max(0.01f, feedback.wvl);
    feedback.wvl_encoder = max(0.01f, feedback.wvl_encoder);

    //Serial.println(sizeof(feedback));

  }
  //Serial.println("==================");

  if(isnan(package.wv1)) package.wv1 = 0.0f;
  if(isnan(package.wv2)) package.wv2 = 0.0f;
  drive(package.wv1, package.wv2);
}