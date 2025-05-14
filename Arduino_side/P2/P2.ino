#include <Dynamixel2Arduino.h>
#include <SoftwareSerial.h>
#include <DynamixelShield.h>


SoftwareSerial soft_serial(7, 8);     // DYNAMIXELShield UART RX/TX
SoftwareSerial soft_serial1(23, 24);  // DYNAMIXELShield UART RX/TX

#define DXL_SERIAL Serial1
#define DXL_SERIAL1 Serial2
#define DEBUG_SERIAL soft_serial
#define DEBUG_SERIAL1 soft_serial1

float value1, value2, value3;  // Global variables for storing inputs
bool stringComplete = false;
float theta[6];
int32_t speed[6];
String inputString = "";
//const int DXL_DIR_PIN = 2;    // Direction control pin for RS-485
const int DXL_DIR_PIN1 = 22;  // Direction control pin for RS-485


bool measureStart = false;
bool haverun = true;
bool getonce = false;
bool getonceT = false;
int targetTime = 3000;
float executingMovement = false;
double startTime = 0;
bool motionActive = true;
float lastMillis = millis();

float start[5];


const uint8_t DXL_ID1 = 1;  // Set your Dynamixel servo ID
const uint8_t DXL_ID2 = 2;  // Set your Dynamixel servo ID
const uint8_t DXL_ID4 = 4;  // Set your Dynamixel servo ID
const uint8_t DXL_ID3 = 3;  // Set your Dynamixel servo ID
const uint8_t DXL_ID5 = 5;  // Set your Dynamixel servo ID
const uint8_t DXL_ID6 = 6;  // Set your Dynamixel servo ID

uint8_t DXL_IDs[6] = { 1, 2, 4, 3, 5, 6 };


const uint8_t DXL_ID1b = 1;  // Set your Dynamixel servo ID

int32_t speedA = 1;


const float DXL_PROTOCOL_VERSION = 2.0;  // Use 2.0 for newer servos

// Initialize the Dynamixel2Arduino library
DynamixelShield dxl(DXL_SERIAL, 2);
DynamixelShield dxl1(DXL_SERIAL1, DXL_DIR_PIN1);


// Use namespace for control table items
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  inputString.reserve(300);  // Reserve memory for efficiency

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(57600);
  while (!DEBUG_SERIAL)
    ;

  Serial.begin(57600);
  dxl.begin(57600);
  dxl1.begin(9600);
  //dxl1.begin(9600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl1.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  //dxl1.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID4);
  dxl.ping(DXL_ID3);
  dxl.ping(DXL_ID5);
  dxl.ping(DXL_ID6);
  dxl1.ping(DXL_ID1b);

  //dxl1.ping(DXL_ID1b);

  DEBUG_SERIAL.println("first board initialized and pinged successfully!");
  DEBUG_SERIAL1.println("Second board initialized and pinged successfully!");
  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.torqueOn(DXL_ID1);

  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.torqueOn(DXL_ID2);

  dxl.torqueOff(DXL_ID4);
  dxl.setOperatingMode(DXL_ID4, OP_POSITION);
  dxl.torqueOn(DXL_ID4);

  dxl.torqueOff(DXL_ID3);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.torqueOn(DXL_ID3);

  dxl.torqueOff(DXL_ID5);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.torqueOn(DXL_ID3);

  dxl.torqueOff(DXL_ID6);
  dxl.setOperatingMode(DXL_ID3, OP_POSITION);
  dxl.torqueOn(DXL_ID3);


  //dxl1.torqueOff(DXL_ID1b);
  //dxl1.setOperatingMode(DXL_ID1b, OP_POSITION);
  //dxl1.torqueOn(DXL_ID1b);
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, 0);

  dxl1.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1b, 50);

  //readUserInputs();
  dxl1.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1b, 100);
}

// Helper function to read a float from the Serial Monitor after printing a prompt.
// This function waits until the user enters a full line (ends with '\n' or '\r').

void test3DOF() {
  dxl.setGoalPosition(DXL_ID1, value1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, value2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID4, value3, UNIT_DEGREE);
}


void parseMessage(String msg) {
  for (int i = 0; i < 6; i++) {
    String motorTag = "M" + String(i + 1);
    String endTag = String(i + 1) + "M";
    int startIndex = msg.indexOf(motorTag);
    int endIndex = msg.indexOf(endTag);

    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
      String valueStr = msg.substring(startIndex + motorTag.length(), endIndex);
      float temp = valueStr.toFloat();
      theta[i] = convertAngle(temp, i + 1);
    } else {
      theta[i] = 0;  // Error fallback
    }
  }

  for (int i = 0; i < 6; i++) {
    String speedTag = "S" + String(i + 1);
    String endTag = String(i + 1) + "S";
    int startIndex = msg.indexOf(speedTag);
    int endIndex = msg.indexOf(endTag);

    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
      String valueStr = msg.substring(startIndex + speedTag.length(), endIndex);
      float temp = valueStr.toFloat();
      //speed[i] = convertSpeed(temp);
    } else {
      //speed[i] = 0;  // Error fallback
    }
  }
}

int32_t convertSpeed(float speed) {
  float convertedspeed = speed / (6 * 0.229);

  if (convertedspeed < 100) {
    return (int32_t)convertedspeed;
  } else {
    while (1) {
    }
  }
}

float convertAngle(float angle, int id) {
  float servoAngle = angle + 180;
  if (id == 2) {
    servoAngle += 45;
  }
  return servoAngle;
}

/*void loop() {
  if (stringComplete) {
    parseMessage(inputString);
    inputString = "";
    stringComplete = false;

    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, speed[0]+speedA,0);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, speed[1]+speedA,0);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, speed[3]+speedA,0);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, speed[2]+speedA,0);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, speed[4]+speedA,0);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, speed[5]+speedA,0);

    dxl.setGoalPosition(DXL_ID1, theta[0], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID2, theta[1], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID3, theta[3], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID4, theta[2] - 90, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID5, theta[4], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID6, theta[5], UNIT_DEGREE);

    // Just for testing: print out the extracted theta values
    Serial.print(speed[0],4);
    Serial.print(speed[1],4);
    Serial.print(speed[3],4);
    Serial.print(speed[2],4);
    Serial.print(speed[4],4);
    Serial.print(speed[5],4);
  }
}*/

/*void parseMessage(String msg) {
  for (int i = 0; i < 6; i++) {
    String motorTag = "M" + String(i + 1);
    String endTag = String(i + 1) + "M";
    int startIndex = msg.indexOf(motorTag);
    int endIndex = msg.indexOf(endTag);

    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
      String valueStr = msg.substring(startIndex + motorTag.length(), endIndex);
      float temp = valueStr.toFloat();
      theta[i] = convertAngle(temp, i + 1);
    } else {
      theta[i] = 0;  // Error fallback
    }
  }
}*/


void loop() {

  static float currentSpeed = 0;
  while (!getonce) {
    start[0] = dxl.getPresentPosition(DXL_ID1);
    start[1] = dxl.getPresentPosition(DXL_ID2);
    start[2] = dxl.getPresentPosition(DXL_ID4);
    start[3] = dxl.getPresentPosition(DXL_ID3);
    start[4] = dxl.getPresentPosition(DXL_ID5);
    start[5] = dxl.getPresentPosition(DXL_ID6);

    getonce = true;
  }

  if (stringComplete) {
    haverun = false;
    parseMessage(inputString);
    delay(10);
    inputString = "";
    motionActive = false;
    stringComplete = false;
  }
  if (!motionActive) {
    while (getonceT) {
      startTime = millis();
      getonceT = true;
    }
  }
  for (int i = 0; i < 6; i++) {
    speed[i] = convertSpeed(calcSpeed(start[i], theta[i], 300, startTime));
  }

  if (speed[0] > 0) {

    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, speed[0] + speedA);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, speed[1] + speedA);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, speed[3] + speedA);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, speed[2] + speedA);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, speed[4] + speedA);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, speed[5] + speedA);

    dxl.setGoalPosition(DXL_ID1, theta[0], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID2, theta[1], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID3, theta[3], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID4, theta[2] - 90, UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID5, theta[4], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID6, theta[5], UNIT_DEGREE);
  }
}


float calcSpeed(float start, float thetaEnd, float targetTimeT, float startTime) {

  float currentTime = (millis() - startTime) / 100;

  float targetTime = targetTimeT / 100;


  float a_0 = start;
  float a_1 = 0;
  float a_2 = (3 / pow(targetTime, 2)) * (thetaEnd - start);
  float a_3 = (-2 / pow(targetTime, 3)) * (thetaEnd - start);

  float speed = a_1 + 2 * a_2 * currentTime + 3 * a_3 * pow(currentTime, 2);

  return speed;
}



void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}