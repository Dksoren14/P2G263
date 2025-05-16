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


uint16_t data[12];
float correct_data[12];
float last_correct_data[12];

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
//DynamixelShield dxl1(DXL_SERIAL1, DXL_DIR_PIN1);


// Use namespace for control table items
using namespace ControlTableItem;

void setup() {
 
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(57600);
  while (!DEBUG_SERIAL)
    ;

  Serial.begin(115200);
  dxl.begin(57600);

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID4);
  dxl.ping(DXL_ID3);
  dxl.ping(DXL_ID5);
  dxl.ping(DXL_ID6);
  

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
  dxl.setOperatingMode(DXL_ID5, OP_POSITION);
  dxl.torqueOn(DXL_ID5);

  dxl.torqueOff(DXL_ID6);
  dxl.setOperatingMode(DXL_ID6, OP_POSITION);
  dxl.torqueOn(DXL_ID6);

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 50);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 50);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, 50);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 50);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, 50);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, 50);
  dxl.setGoalPosition(DXL_ID1, 180, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, 180 + 45, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, 180, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID4, 180 - 90, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID5, 180, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID6, 180, UNIT_DEGREE);
}


void loop() {

  if (stringComplete) {
    stringComplete = false;
    int idx = 0;
    int start = 0;

    for (int i = 0; i <= inputString.length(); i++) {
      if (inputString[i] == ',' || inputString[i] == '\n' || i == inputString.length()) {
        String part = inputString.substring(start, i);
        part.trim();                          // remove spaces
        if (part.length() > 0 && idx < 12) {  // Only count if it's not empty
          data[idx] = part.toInt();
          idx++;
        }
        start = i + 1;
      }
    }
    if (idx == 12) {
      String send_this = "";
      for (int i = 0; i < 12; i++) {
        correct_data[i] = float(data[i]) / 10.0;
        send_this += String(correct_data[i], 1);
        if (i < 11) send_this += ",";
      }
      Serial.println(send_this);
    } else {
      Serial.println("ERROR: Expected 12 values, got " + String(idx));
    }
    inputString = "";
  }
  if (correct_data[0] != last_correct_data[0]) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, convertSpeed(correct_data[0]));
    delay(20);
    dxl.setGoalPosition(DXL_ID1, correct_data[0], UNIT_DEGREE);
    last_correct_data[0] = correct_data[0];
  }
  if (correct_data[1] != last_correct_data[1]) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, convertSpeed(correct_data[1]));
    delay(20);
    dxl.setGoalPosition(DXL_ID2, correct_data[1] + 45, UNIT_DEGREE);
    last_correct_data[1] = correct_data[1];
  }
  if (correct_data[2] != last_correct_data[2]) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, convertSpeed(correct_data[2]));
    delay(20);
    dxl.setGoalPosition(DXL_ID3, correct_data[2] - 90, UNIT_DEGREE);
    last_correct_data[2] = correct_data[2];
  }
  if (correct_data[3] != last_correct_data[3]) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, convertSpeed(correct_data[3]));
    delay(20);
    dxl.setGoalPosition(DXL_ID4, correct_data[3], UNIT_DEGREE);
    last_correct_data[3] = correct_data[3];
  }
  if (correct_data[4] != last_correct_data[4]) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, convertSpeed(correct_data[4]));
    delay(20);
    dxl.setGoalPosition(DXL_ID5, correct_data[4], UNIT_DEGREE);
    last_correct_data[4] = correct_data[4];
  }
  if (correct_data[5] != last_correct_data[5]) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, convertSpeed(correct_data[5]));
    delay(20);
    dxl.setGoalPosition(DXL_ID6, correct_data[5], UNIT_DEGREE);
    last_correct_data[5] = correct_data[5];
  }
  
}


int32_t convertSpeed(float speed) {
  float convertedspeed = speed / (6 * 0.229);

  if (convertedspeed < 200) {
    return (int32_t)convertedspeed;
  } else {
    while (1) {
      Serial.println("EROROROOR");
    }
  }
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