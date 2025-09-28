#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Stepper Motor Driver Pins
#define DIR_PIN 2
#define STEP_PIN 0
/////////////////////////
/* Pins for step angle */
#define M0 4
#define M1 16
#define M2 17
/////////////////////////
/* For Bluetooth module */
#define RIGHT '1'
#define LEFT '2'
/////////////////////////
/* For joystick */
#define VRX_PIN  32 
#define VRY_PIN  33 
#define LEFT_THRESHOLD  1000
#define RIGHT_THRESHOLD 4000
////////////////////////
int valueX = 0 ; 
int valueY = 0 ; 
char cmd;
int j=0;
int stepDelay = 1500;                /* delay between pulses (larger delay slower motor )*/
const int stepsPerRevolution =60 ; /* Number of steps per one move on joystick */

void setFullStepMode()
{
  digitalWrite(M0, LOW );
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

void setup() {
  SerialBT.begin("ESP32-BT");
  
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  setFullStepMode();  // Set to full-step mode
}

void loop() {

  valueX = analogRead(VRX_PIN);
  valueY = analogRead(VRY_PIN);

  if (valueX < LEFT_THRESHOLD)
    stepMotor(stepsPerRevolution, HIGH);
  else if (valueX > RIGHT_THRESHOLD)
    stepMotor(stepsPerRevolution, LOW); 
}
