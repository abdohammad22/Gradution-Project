#include <WiFi.h>
#include <esp_now.h>
#include <LiquidCrystal.h>
#include "mbedtls/md.h"

/*************************/

const int rs = 14, en = 18, d4 = 22, d5 = 23, d6 = 21, d7 = 19;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/*************************/

static const uint8_t hmac_key[32] = {
  '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',
  '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'
};

/*************************/
//Joystick pins

#define VX_PIN 34  
#define VY_PIN 35  

/*************************/
//Switches Pins

#define CONTROL 13
#define BRUSHES 25

/*************************/
//Struct For Data sent

typedef struct struct_message {
  int vx;
  int vy;
  char pump_control;
  char brush_control;
} struct_message;

/*************************/

struct_message joystickData;

/*************************/

// Packet = payload + HMAC digest

typedef struct {
  struct_message data;
  uint8_t hmac[32];
} packet_t;

/*************************/

packet_t packet;

/*************************/

//Mac Address

uint8_t receiverAddress[] = {0xD0, 0xEF, 0x76, 0x57, 0x8B, 0xF0}; 

/*************************/
//Function to send data

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

/*************************/

void setup() {
  pinMode(CONTROL, INPUT);  
  pinMode(BRUSHES, INPUT);
  lcd.begin(16, 2);
  lcd.print("hello");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Welcome to");
  lcd.setCursor(0, 1);
  lcd.print("Joystick mode ");
  delay(1000);
  lcd.clear();

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  uint8_t pmk[16] = "12345678";
  if (esp_now_set_pmk(pmk) != ESP_OK) {
    Serial.println("Error setting PMK");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;         
  peerInfo.encrypt = true;      
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
} 

void loop() {

 //Take read from Joystick and switches

  packet.data.vx            = analogRead(VX_PIN);
  packet.data.vy            = analogRead(VY_PIN);
  packet.data.pump_control  = digitalRead(CONTROL);
  packet.data.brush_control = digitalRead(BRUSHES);

/*************************/

  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_setup(&ctx, info, 1);
  mbedtls_md_hmac_starts(&ctx, hmac_key, sizeof(hmac_key));
  mbedtls_md_hmac_update(&ctx,(uint8_t *)&packet.data,sizeof(packet.data));
  mbedtls_md_hmac_finish(&ctx, packet.hmac);
  mbedtls_md_free(&ctx);

/*************************/

  lcd.setCursor(0, 0);
  lcd.print("VX: ");
  lcd.print(joystickData.vx);
  lcd.setCursor(0, 1);
  lcd.print("VY: ");
  lcd.print(joystickData.vy);
 
  //Send data
  esp_err_t res = esp_now_send(receiverAddress,(uint8_t *)&packet,sizeof(packet));
  if (res != ESP_OK) {
    Serial.printf("Send error: %d\n", res);
  }

  delay(50);
}