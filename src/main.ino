#include "Arduino.h"
#include <TimerOne.h>

#define STATUS_LED 13 //Digital pin

#define TORPEDO_1 11
#define TORPEDO_2 8
#define TORPEDO_CURRENT_SELECT  9  //Digital pin
#define TORPEDO_CURRENT_ENABLE  10 //Digital pin
#define TORPEDO_CURRENT_SENSE   0  //Analog Pin

#define MOTOR_DIR 7 //Digital pin
#define MOTOR_STEP  6 //Digital pin
#define MOTOR_ENABLE  2 //Digital pin
#define MOTOR_MS1 3 //Digital pin
#define MOTOR_MS2 4 //Digital pin
#define MOTOR_MS3 5 //Digital pin

void setup() {
  //Setup pins
  pinMode(STATUS_LED, OUTPUT);
  pinMode(TORPEDO_1, OUTPUT);
  pinMode(TORPEDO_2, OUTPUT);

  digitalWrite(TORPEDO_1, HIGH);
  digitalWrite(TORPEDO_2, LOW);

  //Setup Serial
  Serial.begin(9600);

  //Setup inteerupt
  Timer1.initialize(500000);
  Timer1.attachInterrupt(blinkLED);
  
}

void loop() {

  if(Serial.available()) {
    String commandString = Serial.readString();

    
  }
  
}

void blinkLED() {
  digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}

