#include "Arduino.h"
#include <TimerOne.h>

#define STATUS_LED 13 //Digital pin

#define TORPEDO_1 11
#define TORPEDO_2 8
#define TORPEDO_CURRENT_SELECT  9  //Digital pin
#define TORPEDO_CURRENT_ENABLE  10 //Digital pin
#define TORPEDO_CURRENT_SENSE   0  //Analog Pin
#define NUMBER_OF_TORPEDOS      2

#define MOTOR_DIR 7 //Digital pin
#define MOTOR_STEP  6 //Digital pin
#define MOTOR_ENABLE  2 //Digital pin
#define MOTOR_MS1 3 //Digital pin
#define MOTOR_MS2 4 //Digital pin
#define MOTOR_MS3 5 //Digital pin

uint8_t motorSpeed = 0;
uint32_t numRotations = 0;

void stopMotor() {
  motorSpeed = 0;
  digitalWrite(MOTOR_STEP, LOW);
  digitalWrite(MOTOR_ENABLE, HIGH);
}

void setMotorContinuous(uint8_t _speed) {
  motorSpeed = _speed;
}

void setMotorRotations(uint8_t rotations, uint8_t _speed) {
  numRotations = rotations;
  motorSpeed = _speed;
}


void setMicroStep(uint8_t resolution) {
      
  uint8_t microstep_LUT[5][3] = { {0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 1}, {1, 1, 1} };
  uint8_t index = 0;
  
  if(resolution == 1) {
    index = 0;
  } else if (resolution == 2) {
    index = 1;
  } else if (resolution == 4) {
    index = 2;
  } else if (resolution == 8) {
    index = 3;
  } else if (resolution == 16) {
    index = 4;
  } else {
    //no match
  }
  
  digitalWrite(MOTOR_MS3, microstep_LUT[index][0]);
  digitalWrite(MOTOR_MS2, microstep_LUT[index][1]);
  digitalWrite(MOTOR_MS2, microstep_LUT[index][2]);
}

int8_t fireTorpedo(uint8_t number) {

  if( number < 1 || number > NUMBER_OF_TORPEDOS) {
    return -1; // Error. Invalid number
  }

  number -= 1; // To adjust for array index
  uint8_t torpedoPinMap[NUMBER_OF_TORPEDOS] = {TORPEDO_1, TORPEDO_2};
  
  digitalWrite(torpedoPinMap[number], HIGH);
  delay(500);
  digitalWrite(torpedoPinMap[number], LOW);
}

int8_t poll_UART() {
  
    if(Serial.available()) {
    String commandString = Serial.readStringUntil('\n');
    
    if(commandString == "RID" || commandString == "*IDN?") {
      Serial.println("Weight_Dropper_Torpedo_1.0");
      
    } else if(commandString.substring(0, 2) == "RM" && commandString.length() == 5) {

      digitalWrite(MOTOR_ENABLE, LOW);
      setMotorRotations((commandString.substring(2, 5)).toInt(), 1);
      Serial.println("ACK");
      
    } else if(commandString == "STP") {
      stopMotor();
      
      Serial.println("ACK");
    } else if(commandString.substring(0, 2) == "MS") {
      
      uint8_t argument = (commandString.substring(2, 4)).toInt();
      setMicroStep(argument);
      
    } else if (commandString.substring(0, 2) == "PE") {
      
      char device = commandString.charAt(2);
      uint8_t argument = ((char)commandString.charAt(3)) - '0';

      if(device == 'M') {
        digitalWrite(MOTOR_ENABLE, !argument);
        
      }

      Serial.println("ACK");
      
    } else if (commandString.substring(0, 2) == "FT" && commandString.length() == 3) {
      uint8_t argument = (commandString.substring(2,3)).toInt();

      Serial.println("ACK");
      fireTorpedo(argument);
    } else {
      Serial.println("ERR");
    }

  } else {
    return 0;
  }

  return 1;
}

void setup() {
  //Setup LED
  pinMode(STATUS_LED, OUTPUT);

  //Setup interupt
  Timer1.initialize(500000);
  Timer1.attachInterrupt(blinkLED);

  //Init Stepper Motor
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_STEP, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_MS1, OUTPUT);
  pinMode(MOTOR_MS2, OUTPUT);
  pinMode(MOTOR_MS3, OUTPUT);
  digitalWrite(MOTOR_ENABLE, HIGH);
  setMicroStep(8);

  //Init Torpedo
  pinMode(TORPEDO_1, OUTPUT);
  pinMode(TORPEDO_2, OUTPUT);

  //Setup Serial
  Serial.begin(9600);
}

void loop() {
 
   poll_UART();

   if(motorSpeed > 0 && numRotations > 0) {
    digitalWrite(MOTOR_STEP, !digitalRead(MOTOR_STEP));
    numRotations--;
    delay(motorSpeed);
   }
  
}

void blinkLED() {
  digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}

