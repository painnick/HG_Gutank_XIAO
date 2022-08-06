#include <Arduino.h>

#include "SoftwareSerial.h"

#if !( defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) \ 
      || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) \ 
      || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) \ 
      || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(__SAMD51__) || defined(__SAMD51J20A__) \ 
      || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__)  \ 
      || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \ 
      || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \ 
      || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__) ) 
  #error This code is designed to run on SAMD21/SAMD51 platform! Please check your Tools->Board setting. 
#endif 
#define ISR_SERVO_DEBUG 1

#include "SAMD_ISR_Servo.hpp"     //https://github.com/khoih-prog/SAMD_ISR_Servo
#include "SAMD_ISR_Servo_Impl.h"
#include "SAMD_FastTimerInterrupt.h"

 // TIMER_TC3 for SAMD51, TIMER_TC3/TIMER_TCC for SAMD21 
 #define USING_SAMD_TIMER            TIMER_TC5
  
 // Published values for SG90 servos; adjust if needed 
 #define MIN_MICROS        800 
 #define MAX_MICROS        2450 

#define PIN_TRACK_A1 0
#define PIN_TRACK_A2 1
#define PIN_TRACK_B1 2
#define PIN_TRACK_B2 3

#define PIN_BODY 8
#define PIN_LEFT_ARM 9
#define PIN_RIGHT_ARM 10

#define PIN_LEFT_GUN 4
#define PIN_RIGHT_GUN 5

#define BLUETOOTH_TX 6
#define BLUETOOTH_RX 7

#define SG90_DELAY 15 // ms
#define MINI_SG_DELAY 15 // ms

#define ANGLE_CENTER 90

#define BODY_ANGLE_MIN 30
#define BODY_ANGLE_MAX 150

#define ARM_ANGLE_MIN 45
#define ARM_ANGLE_MAX 135


#define _USE_SERIAL_ 0

typedef enum {
  UNKNOWN_SIDE = -1,
  LEFT,
  RIGHT,
  BOTH
} GUNTANK_SIDE;

typedef enum {
  UNKNOWN_DIRECTION = -1,
  FORWARD,
  BACKWARD,
  CENTER
} GUNTANK_DIRECTION;

int bodyServoIndex;
int armsServoIndex[2];
int gunPins[2] = {PIN_LEFT_GUN, PIN_RIGHT_GUN};
int trackPins[4] = {PIN_TRACK_A1, PIN_TRACK_A2, PIN_TRACK_B1, PIN_TRACK_B2};

SoftwareSerial SerialBT(BLUETOOTH_RX, BLUETOOTH_TX);

void setup() {
  #ifdef _USE_SERIAL_
  Serial.begin(115200);
  #endif

  SerialBT.begin(9600);

  pinMode(PIN_BODY, OUTPUT);
  pinMode(PIN_LEFT_ARM, OUTPUT);
  pinMode(PIN_RIGHT_ARM, OUTPUT);

  digitalWrite(PIN_BODY, LOW);
  digitalWrite(PIN_LEFT_ARM, LOW);
  digitalWrite(PIN_RIGHT_ARM, LOW);

  #ifdef _USE_SERIAL_
  Serial.print(F("\nStarting SAMD_MultipleRandomServos on ")); Serial.println(BOARD_NAME); 
  Serial.println(SAMD_ISR_SERVO_VERSION); 
  #endif

  // SAMD51 always uses TIMER_TC3 
  SAMD_ISR_Servos.useTimer(TIMER_TC3); 

  bodyServoIndex = SAMD_ISR_Servos.setupServo(PIN_BODY, MIN_MICROS, MAX_MICROS);
  #ifdef _USE_SERIAL_
  if (bodyServoIndex == -1) {
    Serial.println("[ERR] Body-servo init. FAILED!");
  }
  #endif
  armsServoIndex[GUNTANK_SIDE::LEFT] = SAMD_ISR_Servos.setupServo(PIN_LEFT_ARM, MIN_MICROS, MAX_MICROS);
  #ifdef _USE_SERIAL_
  if (armsServoIndex[GUNTANK_SIDE::LEFT] == -1) {
    Serial.println("[ERR] Left-arm-servo init. FAILED!");
  }
  #endif
  armsServoIndex[GUNTANK_SIDE::RIGHT] = SAMD_ISR_Servos.setupServo(PIN_RIGHT_ARM, MIN_MICROS, MAX_MICROS);
  #ifdef _USE_SERIAL_
  if (armsServoIndex[GUNTANK_SIDE::RIGHT] == -1) {
    Serial.println("[ERR] Right-arm-servo init. FAILED!");
  }
  #endif

  SAMD_ISR_Servos.setReadyToRun();
  
  pinMode(PIN_TRACK_A1, OUTPUT);
  pinMode(PIN_TRACK_A2, OUTPUT);
  pinMode(PIN_TRACK_B1, OUTPUT);
  pinMode(PIN_TRACK_B2, OUTPUT);

  pinMode(PIN_LEFT_GUN, OUTPUT);
  pinMode(PIN_RIGHT_GUN, OUTPUT);

  delay(1000);

  #ifdef _USE_SERIAL_
  Serial.println("R.E.A.D.Y!!!");
  #endif
}

GUNTANK_SIDE convertToSide(char c) {
  if (c == 'l')
    return GUNTANK_SIDE::LEFT;
  else if (c == 'r')
    return GUNTANK_SIDE::RIGHT;
  else if (c == 'b')
    return GUNTANK_SIDE::BOTH;
  else
    return GUNTANK_SIDE::UNKNOWN_SIDE;
}

GUNTANK_DIRECTION convertToDirection(char c) {
  if (c == 'f')
    return GUNTANK_DIRECTION::FORWARD;
  else if (c == 'b')
    return GUNTANK_DIRECTION::BACKWARD;
  else if (c == 'c')
    return GUNTANK_DIRECTION::CENTER;
  else
    return GUNTANK_DIRECTION::UNKNOWN_DIRECTION;
}

int bodyAngle = ANGLE_CENTER;
void turnBody(GUNTANK_DIRECTION direction, int angle) {
  #ifdef _USE_SERIAL_
  Serial.print("turnBody dir:");
  Serial.print(direction);
  Serial.print(" angle:");
  Serial.println(angle);
  #endif

  if (direction == GUNTANK_DIRECTION::UNKNOWN_DIRECTION)
    return;
  int newAngle = ANGLE_CENTER;
  if (direction != GUNTANK_DIRECTION::CENTER) {
    newAngle = bodyAngle + (direction == GUNTANK_DIRECTION::FORWARD ? angle : -angle);
    newAngle = min(BODY_ANGLE_MAX, newAngle);
    newAngle = max(BODY_ANGLE_MIN, newAngle);
  }
  bodyAngle = newAngle;
  #ifdef _USE_SERIAL_
  Serial.printf("Body Angle %d\n", bodyAngle);
  #endif
  SAMD_ISR_Servos.setPosition(PIN_BODY, bodyAngle);
  delay(SG90_DELAY);
}

int armAngles[2] = {ANGLE_CENTER, ANGLE_CENTER};
void turnArm(GUNTANK_SIDE side, GUNTANK_DIRECTION direction, int angle) {
  #ifdef _USE_SERIAL_
  Serial.print("turnArm side:");
  Serial.print(side);
  Serial.print(" dir:");
  Serial.print(direction);
  Serial.print(" angle:");
  Serial.println(angle);
  #endif

  if (side == GUNTANK_SIDE::UNKNOWN_SIDE)
    return;
  if (direction == GUNTANK_DIRECTION::UNKNOWN_DIRECTION)
    return;
  int newAngle = ANGLE_CENTER;
  if (direction != GUNTANK_DIRECTION::CENTER) {
    newAngle = armAngles[side] + (direction == GUNTANK_DIRECTION::FORWARD ? angle : -angle);
    newAngle = min(ARM_ANGLE_MAX, newAngle);
    newAngle = max(ARM_ANGLE_MIN, newAngle);
  }
  armAngles[side] = newAngle;
  SAMD_ISR_Servos.setPosition(armsServoIndex[side], armAngles[side]);
  delay(MINI_SG_DELAY);
}

void fire(GUNTANK_SIDE side) {
  #ifdef _USE_SERIAL_
  Serial.print("fire side:");
  Serial.println(side);
  #endif

  if (side == GUNTANK_SIDE::UNKNOWN_SIDE)
    return;
  bool isOn = true;
  for(int i = 0; i < 6; i ++) {
    if (side == GUNTANK_SIDE::BOTH) {
      digitalWrite(gunPins[GUNTANK_SIDE::LEFT], isOn ? HIGH : LOW);
      digitalWrite(gunPins[GUNTANK_SIDE::RIGHT], isOn ? HIGH : LOW);
      delay(100);
    } else {
      digitalWrite(gunPins[side], isOn ? HIGH : LOW);
      delay(100);
    }
    isOn = !isOn;
  }
}

void track(GUNTANK_SIDE side, GUNTANK_DIRECTION direction) {
  #ifdef _USE_SERIAL_
  Serial.print("track side:");
  Serial.print(side);
  Serial.print(" dir:");
  Serial.println(direction);
  #endif

  if (side == GUNTANK_SIDE::UNKNOWN_SIDE)
    return;
  if (direction == GUNTANK_DIRECTION::UNKNOWN_DIRECTION)
    return;
  switch (side) {
    case GUNTANK_SIDE::LEFT:
      switch (direction) {
      case GUNTANK_DIRECTION::FORWARD:
        digitalWrite(trackPins[0], HIGH);
        digitalWrite(trackPins[1], LOW);
        break;
      case GUNTANK_DIRECTION::BACKWARD:
        digitalWrite(trackPins[0], LOW);
        digitalWrite(trackPins[1], HIGH);
        break;
      case GUNTANK_DIRECTION::CENTER:
        digitalWrite(trackPins[0], LOW);
        digitalWrite(trackPins[1], LOW);
        break;
      }
    break;
    case GUNTANK_SIDE::RIGHT:
      switch (direction) {
      case GUNTANK_DIRECTION::FORWARD:
        digitalWrite(trackPins[2], HIGH);
        digitalWrite(trackPins[3], LOW);
        break;
      case GUNTANK_DIRECTION::BACKWARD:
        digitalWrite(trackPins[2], LOW);
        digitalWrite(trackPins[3], HIGH);
        break;
      case GUNTANK_DIRECTION::CENTER:
        digitalWrite(trackPins[2], LOW);
        digitalWrite(trackPins[3], LOW);
        break;
      }
    break;
    case GUNTANK_SIDE::BOTH:
      switch (direction) {
      case GUNTANK_DIRECTION::FORWARD:
        digitalWrite(trackPins[0], HIGH);
        digitalWrite(trackPins[1], LOW);
        digitalWrite(trackPins[2], HIGH);
        digitalWrite(trackPins[3], LOW);
        break;
      case GUNTANK_DIRECTION::BACKWARD:
        digitalWrite(trackPins[0], LOW);
        digitalWrite(trackPins[1], HIGH);
        digitalWrite(trackPins[2], LOW);
        digitalWrite(trackPins[3], HIGH);
        break;
      case GUNTANK_DIRECTION::CENTER:
        digitalWrite(trackPins[0], LOW);
        digitalWrite(trackPins[1], LOW);
        digitalWrite(trackPins[2], LOW);
        digitalWrite(trackPins[3], LOW);
        break;
      }
    break;
  } 
}

char command[16];

void loop() {
  // #ifdef _USE_SERIAL_
  // Serial.println("...");
  // #endif

  int index = 0;
  while (SerialBT.available()) {
    int c = SerialBT.read();
    command[index] = c;
    if (c == '/')
      break;
    index++;
  }
  if (index == 0) {
    delay(200);
    return;
  }
  command[index] = 0;
  #ifdef _USE_SERIAL_
  Serial.print("Command : [");
  Serial.print(command);
  Serial.println("]");
  #endif

  char side;
  char direction;
  int angle;
  switch (command[0])
  {
  case 'b': // Body. B (F/B/C) 15
    sscanf(command, "b%c%d", &direction, &angle);
    turnBody(convertToDirection(direction), angle);
    break;
  case 'a': // Arm. A (L/R/B) (F/B/C) 15
    sscanf(command, "a%c%c%d", &side, &direction, &angle);
    turnArm(convertToSide(side), convertToDirection(direction), angle);
    break;
  case 'f': // Fire. F (L/R/B)
    sscanf(command, "f%c", &side);
    fire(convertToSide(side));
    break;
  case 't': // Track. T (L/R/B) (F/B/C)
    sscanf(command, "t%c%c", &side, &direction);
    track(convertToSide(side), convertToDirection(direction));
    break;
  }
}
