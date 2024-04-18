/********************************************************************
  ECEN 240/301 Lab Code
  Light-Following Robot

  The approach of this code is to use an architecture that employs
  three different processes:
    Perception
    Planning
    Action

  By separating these processes, this allows one to focus on the
  individual elements needed to do these tasks that are general
  to most robotics.

  Version History
  1.1.3       11 January 2023   Creation by Dr. Mazzeo and TAs from 2022 version
 ********************************************************************/

#include "Arduino.h"
#include <CapacitiveSensor.h>
#include <Servo.h>
#include <NewPing.h>


#define PHOTODIODE_1  A1 // left diode, turn left
#define PHOTODIODE_2  A2 // front diode, move down
#define PHOTODIODE_3  A3  // right diode, turn right
#define PHOTODIODE_4  A4 // back diode, move up

#define H_BRIDGE_ENA 3
#define LED_3   4
#define H_BRIDGE_ENB 5


// Capacitive sensor pins - Lab 4
#define CAP_SENSOR_SEND     11
#define CAP_SENSOR_RECEIVE  10


// Ultrasonic sensor pin - Lab 6
#define TRIGGER_PIN 8
#define ECHO_PIN 7

// Configuration parameter definitions
#define BUTTON_THRESHOLD 2.5

// Voltage at which a photodiode voltage is considered to be present - Lab 5
#define PHOTODIODE_LIGHT_THRESHOLD 2.7

// Number of samples that the capacitor sensor will use in a measurement - Lab 4
#define CAP_SENSOR_SAMPLES 40
#define CAP_SENSOR_TAU_THRESHOLD 100

// The speeds for the wheels from the capactance - Lab 4
#define SPEED_STOP 0
#define SPEED_LOW (int) (255 * 0.45)
#define SPEED_MED (int) (255 * 0.75)
#define SPEED_HIGH (int) (255 * 1)

// Servo pin - Lab 6
#define SERVO_PIN 9

// Parameters for servo control as well as instantiation - Lab 6
#define SERVO_START_ANGLE 90
#define SERVO_UP_LIMIT 160
#define SERVO_DOWN_LIMIT 20
static Servo myServo;


// Parameters for ultrasonic sensor and instantiation - Lab 6
#define MAX_DISTANCE 200 // centimeters
static NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Parameter to define when the ultrasonic sensor detects a collision - Lab 6
#define COLLISION_DISTANCE 15


/***********************************************************/

// Definitions that allow one to set states
#define DETECTION_NO    0
#define DETECTION_YES   1

#define COLLISION_ON   0
#define COLLISION_OFF  1

#define DRIVE_STOP      0
#define DRIVE_LEFT      1
#define DRIVE_RIGHT     2
#define DRIVE_STRAIGHT  3

#define SERVO_MOVE_STOP 0
#define SERVO_MOVE_UP   1
#define SERVO_MOVE_DOWN 2

// Global variables for PERCEPTION
int SensedCollision = DETECTION_NO;
int SensedLightRight = DETECTION_NO;
int SensedLightLeft = DETECTION_NO;
int SensedLightUp = DETECTION_NO;
int SensedLightDown = DETECTION_NO;

int SensedCapacitiveTouch = DETECTION_NO;

// Global variables for ACTION
int ActionCollision = COLLISION_OFF;
int ActionRobotDrive = DRIVE_STOP;
int ActionRobotSpeed = SPEED_STOP;
int ActionServoMove = SERVO_MOVE_STOP;

void setup() {
  Serial.begin(9600);
 
  pinMode(H_BRIDGE_ENA, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(H_BRIDGE_ENB, OUTPUT);
 
  pinMode(PHOTODIODE_1, INPUT);
  pinMode(PHOTODIODE_2, INPUT);
  pinMode(PHOTODIODE_3, INPUT);
  pinMode(PHOTODIODE_4, INPUT);

  pinMode(CAP_SENSOR_RECEIVE, INPUT);
  pinMode(CAP_SENSOR_SEND, OUTPUT);
  
  //Set up servo - Lab 6
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_START_ANGLE);

  pinMode(TRIGGER_PIN, OUTPUT); // pulse sent out through TRIGGER_PIN    
  pinMode(ECHO_PIN, INPUT); // return signal read through ECHO_PIN


}

/********************************************************************
  Main LOOP function - this gets executed in an infinite loop until
  power off or reset. - Notice: PERCEPTION, PLANNING, ACTION
 ********************************************************************/

void loop() {
  RobotPerception();
  RobotPlanning();
  RobotAction();
}

void RobotPerception() {
  // Photodiode Sensing
  if (isLight(PHOTODIODE_1)){
    SensedLightLeft = DETECTION_YES;
  } else {
    SensedLightLeft = DETECTION_NO;
  }
  if (isLight(PHOTODIODE_3)) {
    SensedLightRight = DETECTION_YES;
  } else {
    SensedLightRight = DETECTION_NO;
  }

  // Add sensing for light up and down
  if (isButtonPushed(PHOTODIODE_4)){
    SensedLightUp = DETECTION_YES;
  } else {
    SensedLightUp = DETECTION_NO;
  }
  if (isButtonPushed(PHOTODIODE_2)) {
    SensedLightDown = DETECTION_YES;
  } else {
    SensedLightDown = DETECTION_NO;
  }

  // Collision Sensing
  if (isCollision()) {
    SensedCollision = DETECTION_YES;
  } else {
    SensedCollision = DETECTION_NO;
  }

  if (isCapacitiveSensorTouched()) {
    SensedCapacitiveTouch = DETECTION_YES;
  }
  else {
    SensedCapacitiveTouch = DETECTION_NO;
  }
}

float getPinVoltage(int pin) {
  return 5 * (float)analogRead(pin) / 1024;
}

bool isButtonPushed(int button_pin) {
  return getPinVoltage(button_pin) >= BUTTON_THRESHOLD;
}

bool isCollision() {
  int sonar_distance = sonar.ping_cm(); // If the distance is too big, it returns 0.
  if(sonar_distance != 0){ 
    return (sonar_distance < COLLISION_DISTANCE);
  } else {
	return false;
  }

}

bool isCapacitiveSensorTouched() {
  static CapacitiveSensor sensor 
    = CapacitiveSensor(CAP_SENSOR_SEND, CAP_SENSOR_RECEIVE);
  long tau 
    =  sensor.capacitiveSensor(CAP_SENSOR_SAMPLES); 
  return tau > CAP_SENSOR_TAU_THRESHOLD;
}

bool isLight(int pin) {
  float light = getPinVoltage(pin);
  return (light > PHOTODIODE_LIGHT_THRESHOLD);
}

void RobotPlanning(void) {
  fsmCollisionDetection();
  fsmSteerRobot();
  fsmMoveServoUpAndDown();
  fsmCapacitiveSensorSpeedControl();
}

void fsmCollisionDetection() {
  static int collisionDetectionState = 0;
 
  switch (collisionDetectionState) {
    case 0:
      ActionCollision = COLLISION_ON;
      if (SensedCollision == DETECTION_NO) {
        collisionDetectionState = 1;
      }
      break;
    case 1:
      ActionCollision = COLLISION_OFF;
      fsmSteerRobot();
      if (SensedCollision == DETECTION_YES) {
        collisionDetectionState = 0;
      }
      break;
  }
}

void fsmSteerRobot() {
  static int steerRobotState = 0;

  // Add a check to ensure that driving actions are not allowed if a collision has been detected.
  if (ActionCollision == COLLISION_ON) {
    // If a collision is detected, ensure the robot does not attempt to drive left or right.
    ActionRobotDrive = DRIVE_STOP;
    return; // Exit the function early as no further steering action should be taken.
  }

  switch (steerRobotState) {
    case 0: // No light detected
      ActionRobotDrive = DRIVE_STOP;
      if (SensedLightLeft == DETECTION_YES) {
        steerRobotState = 1; // Light is on the left, prepare to steer left
      } else if (SensedLightRight == DETECTION_YES) {
        steerRobotState = 2; // Light is on the right, prepare to steer right
      }
      break;
    case 1: // Light is to the left of the robot
      ActionRobotDrive = DRIVE_LEFT;
      if (SensedLightRight == DETECTION_YES) {
        steerRobotState = 3; // Light detected on both sides, consider stopping or going straight
      } else if (SensedLightLeft == DETECTION_NO) {
        steerRobotState = 0; // No light detected, stop
      }
      break;
    case 2: // Light is to the right of the robot
      ActionRobotDrive = DRIVE_RIGHT;
      if (SensedLightLeft == DETECTION_YES) {
        steerRobotState = 3; // Light detected on both sides, consider stopping or going straight
      } else if (SensedLightRight == DETECTION_NO) {
        steerRobotState = 0; // No light detected, stop
      }
      break;
    case 3: // Light detected on both sides
      ActionRobotDrive = DRIVE_STRAIGHT;
      if (SensedLightLeft == DETECTION_NO && SensedLightRight == DETECTION_NO) {
        steerRobotState = 0; // No light detected, stop
      } else if (SensedLightRight == DETECTION_NO) {
        steerRobotState = 1; // Only light on the left, turn left
      } else if (SensedLightLeft == DETECTION_NO) {
        steerRobotState = 2; // Only light on the right, turn right
      }
      break;
  }
}


void fsmMoveServoUpAndDown() {
  static int moveServoState = 0;
 
  switch (moveServoState) {
    case 0:
      ActionServoMove = SERVO_MOVE_STOP;
      if (SensedLightUp == DETECTION_YES && SensedLightDown == DETECTION_NO) {
        moveServoState = 1;
      } else if (SensedLightUp == DETECTION_NO && SensedLightDown == DETECTION_YES) {
        moveServoState = 2;
      }
      break;
    case 1:
      ActionServoMove = SERVO_MOVE_UP;
      if ((SensedLightUp == DETECTION_YES && SensedLightDown == DETECTION_YES) || (SensedLightUp == DETECTION_NO && SensedLightDown == DETECTION_NO)) {
        moveServoState = 0;
      }
      break;
    case 2:
      ActionServoMove = SERVO_MOVE_DOWN;
      if ((SensedLightUp == DETECTION_YES && SensedLightDown == DETECTION_YES) || (SensedLightUp == DETECTION_NO && SensedLightDown == DETECTION_NO)) {
        moveServoState = 0;
      }
      break;
  }
}

////////////////////////////////////////////////////////////////////
// State machine for detecting when the capacitive sensor is
// touched, and changing the robot's speed.
////////////////////////////////////////////////////////////////////
void fsmCapacitiveSensorSpeedControl() {
  static int capacitiveSensorState = 0;
  switch (capacitiveSensorState) {
    case 0: // Wait for button press
      if (SensedCapacitiveTouch == DETECTION_YES) {
        capacitiveSensorState = 1;
      }
      break;
    case 1: // Wait for release
      if (SensedCapacitiveTouch == DETECTION_NO) {
        capacitiveSensorState = 2;
      }
      break;
    case 2: // Toggle speed! (fsmChangeSpeed)
      fsmChangeSpeed();
      capacitiveSensorState = 0;
      break;
  }
}

////////////////////////////////////////////////////////////////////
// State machine for cycling through the robot's speeds.
////////////////////////////////////////////////////////////////////
void fsmChangeSpeed() {
  static int changeSpeedState = 0;
  
  /*Implement in lab 4*/
  switch(changeSpeedState) {
    case 0: // Speed stop
      ActionRobotSpeed = SPEED_STOP;
      changeSpeedState = 1;
      break;
    case 1: // Speed low
      ActionRobotSpeed = SPEED_LOW;
      changeSpeedState = 2;  
      break;
    case 2: //Speed medium
      ActionRobotSpeed = SPEED_MED;
      changeSpeedState = 3; 
      break;
    case 3: // Speed high;
      ActionRobotSpeed = SPEED_HIGH;
      changeSpeedState = 0; 
      break;
  }
}

void RobotAction() {
  switch(ActionCollision) {
    case COLLISION_OFF:
      doTurnLedOff(LED_3);
      break;
    case COLLISION_ON:
      doTurnLedOn(LED_3);
      break;
  }
 
  switch(ActionRobotDrive) {
    case DRIVE_STOP:
      analogWrite(H_BRIDGE_ENB, 0);
      analogWrite(H_BRIDGE_ENA, 0);
      break;
    case DRIVE_STRAIGHT:
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);
      break;
    case DRIVE_LEFT:
      analogWrite(H_BRIDGE_ENB, 0);
      analogWrite(H_BRIDGE_ENA, ActionRobotSpeed);
      break;
    case DRIVE_RIGHT:
      analogWrite(H_BRIDGE_ENB, ActionRobotSpeed);
      analogWrite(H_BRIDGE_ENA, 0);
      break;
  }
 
  MoveServo();
}

void MoveServo() {
  static int servoAngle = SERVO_START_ANGLE;
  switch(ActionServoMove) {
    case SERVO_MOVE_STOP:
      break;
    case SERVO_MOVE_UP:
      if (servoAngle < SERVO_UP_LIMIT) {
        servoAngle++;
      }
      break;
    case SERVO_MOVE_DOWN:
      if (servoAngle > SERVO_DOWN_LIMIT) {
        servoAngle--;
      }
      break;
  }
  myServo.write(servoAngle);
}

void doTurnLedOn(int led_pin) {
  digitalWrite(led_pin, HIGH);
}

void doTurnLedOff(int led_pin) {
  digitalWrite(led_pin, LOW);
}
