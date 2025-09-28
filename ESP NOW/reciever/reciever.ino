#include <WiFi.h>
#include <esp_now.h>

//#include "BluetoothSerial.h"
//BluetoothSerial SerialBT;
//#define BT_DEVICE_NAME "ESP32-BT"
//#define BT_PIN         "1234"

#include "mbedtls/md.h"

static const uint8_t hmac_key[32] = {
  '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',
  '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'
};


// Stepper Motor Driver Pins
#define DIR_PIN 23
#define STEP_PIN 22

#define DIR_PIN_2 16
#define STEP_PIN_2 17

#define RIGHT '1'
#define LEFT '2'
#define UP '3'
#define DOWN '4'
#define PUMP_ON '5'
#define BRUSH_ON '6'
#define PUMP_OFF '7'
#define BRUSH_OFF '8'

// Pins for step angle
#define M0 2
#define M1 0
#define M2 4

 // Joystick thresholds
 #define DOWN_THRESHOLD  10
 #define UP_THRESHOLD 4090
 #define LEFT_THRESHOLD  10
 #define RIGHT_THRESHOLD 4090

#define PUMP 19
#define FAN 18
#define BRUSHES 15
int cmd;

int stepDelay = 1500;                // delay between pulses
const int stepsPerRevolution = 60;   // Number of steps per joystick move

uint8_t SenderAddress[] = {0x14, 0x2B, 0x2F, 0xC2, 0x1F, 0x7C}; 

 // ESP-NOW data structure
typedef struct struct_message {
   int vx;
   int vy;
   char pump_control;
   char brush_control;
 } struct_message;

struct_message receivedData;


typedef struct {
  struct_message data;
  uint8_t hmac[32];
} packet_t;

packet_t incomingPacket;


void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *buf, int len) {
  if (len != sizeof(packet_t)) {
    Serial.println("Unexpected packet size");
    return;
  }

  memcpy(&incomingPacket, buf, sizeof(incomingPacket));

  // 1) Recompute HMAC
  uint8_t calc_hmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const mbedtls_md_info_t *info_md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_setup(&ctx, info_md, 1);
  mbedtls_md_hmac_starts(&ctx, hmac_key, sizeof(hmac_key));
  mbedtls_md_hmac_update(&ctx,
                        (uint8_t *)&incomingPacket.data,
                        sizeof(incomingPacket.data));
  mbedtls_md_hmac_finish(&ctx, calc_hmac);
  mbedtls_md_free(&ctx);

  // 2) Verify integrity
  if (memcmp(calc_hmac, incomingPacket.hmac, sizeof(calc_hmac)) != 0) {
    Serial.println("HMAC mismatch! Packet dropped.");
    return;
  }

  // 3) Integrity OK—use the payload
  receivedData = incomingPacket.data;
  Serial.printf("Verified VX:%4d VY:%4d PUMP=%d BRUSH=%d\n",
                receivedData.vx,
                receivedData.vy,
                receivedData.pump_control,
                receivedData.brush_control);
}

void setFullStepMode() {
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
}

void stepMotor(int steps, int dir) {
  digitalWrite(DIR_PIN, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}

void stepMotor2(int steps, int dir) {
  digitalWrite(DIR_PIN_2, dir);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN_2, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN_2, LOW);
    delayMicroseconds(stepDelay);
  }
}


void setup() {
  
  //SerialBT.begin(BT_DEVICE_NAME, true);
  //bool pinOK = SerialBT.setPin(BT_PIN);
  //if (!pinOK) {
  // Serial.println("Failed to set Bluetooth PIN!");
  //}
 
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(BRUSHES, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  setFullStepMode();

   // ESP-NOW init
   WiFi.mode(WIFI_STA);
   if (esp_now_init() != ESP_OK) {
     Serial.println("ESP-NOW init failed");
     return;
   }

   // 2) Set the same 16-byte PMK
  uint8_t pmk[16] = "12345678";
  if (esp_now_set_pmk(pmk) != ESP_OK) {
    Serial.println("Error setting PMK");
    return;
  }

  // 3) Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // 4) Register master as a peer to allow decryption
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, SenderAddress, 6);
  peerInfo.channel = 0;        // match channel in sender
  peerInfo.encrypt = true;     // expect AES-128 encrypted packets
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}


void loop() {
  
     digitalWrite(FAN,HIGH);
  if(receivedData.vy < DOWN_THRESHOLD) {
     stepMotor(stepsPerRevolution, LOW);
   }
   else if(receivedData.vy > UP_THRESHOLD) {
     stepMotor(stepsPerRevolution, HIGH); 
   }
  
   if(receivedData.vx < LEFT_THRESHOLD) {
     stepMotor2(stepsPerRevolution, HIGH);
   }
   else if (receivedData.vx > RIGHT_THRESHOLD) {
     stepMotor2(stepsPerRevolution, LOW); 
   }else{}

   if(receivedData.pump_control==1){
      digitalWrite(PUMP,HIGH);
   }else{
       digitalWrite(PUMP,LOW);
       }
  
    if(receivedData.brush_control==0){
      digitalWrite(BRUSHES,LOW);
    }else{
      digitalWrite(BRUSHES,HIGH);
    }

    }

   //Bluetooth code
  /*
    if(SerialBT.available()){
        digitalWrite(FAN,HIGH);
        cmd = SerialBT.read();
     } 
     if(cmd == RIGHT){
         stepMotor2(stepsPerRevolution, HIGH);
     }
     else if(cmd == LEFT){
         stepMotor2(stepsPerRevolution, LOW);
     }
     else if(cmd==UP){
         stepMotor(stepsPerRevolution, HIGH);
     }
     else if(cmd==DOWN){
         stepMotor(stepsPerRevolution, LOW); 
     }
     else if(cmd==PUMP_ON){
         digitalWrite(PUMP,HIGH);   
     }
     else if(cmd==PUMP_OFF){
         digitalWrite(PUMP,LOW);   
     }
     
     else if(cmd==BRUSH_ON){
         digitalWrite(BRUSHES,HIGH);      
     }
     
     else if(cmd==BRUSH_OFF){
         digitalWrite(BRUSHES,LOW);      
     }
     }
     
*/    

