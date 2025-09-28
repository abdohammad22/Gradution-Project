#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define DIR_PIN 22
#define STEP_PIN 23

#define DIR_PIN_2 2
#define STEP_PIN_2 0

#define M0 4
#define M1 16
#define M2 17
#define RIGHT '1'
#define LEFT '2'
#define UP '3'
#define DOWN '4'

char cmd;

int stepDelay = 1500;
const int stepsPerPress = 60;

void setMicrostepping()
{
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
}

void stepMotor(int steps, int dir)
{
  digitalWrite(DIR_PIN, dir); // Set direction

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}

void stepMotor2(int steps, int dir)
{
  digitalWrite(DIR_PIN_2, dir); // Set direction

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN_2, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN_2, LOW);
    delayMicroseconds(stepDelay);
  }
}

void setup() {
  
  SerialBT.begin("ESP32");
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  setMicrostepping();  // Set 1/16 step mode

}

void loop() {
  // put your main code here, to run repeatedly:
  if(SerialBT.available()){
      cmd = SerialBT.read();
      } 
      if(cmd == RIGHT){
          stepMotor2(stepsPerPress, HIGH);
       }
       else if(cmd == LEFT){
          stepMotor2(stepsPerPress, LOW);
      }
      else if(cmd==UP){
          stepMotor(stepsPerPress, HIGH);
       }
      else if(cmd==DOWN){
          stepMotor(stepsPerPress, LOW); 
       }
}

