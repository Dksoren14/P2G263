#include <Dynamixel2Arduino.h>
#include <DynamixelShield.h>
#include <Servo.h>

#define DXL_SERIAL Serial1
#define DXL_SERIAL1 Serial2

bool stringComplete = false;
float theta[6];
int32_t speed[6];
String inputString = "";
//const int DXL_DIR_PIN = 2;    // Direction control pin for RS-485
const int DXL_DIR_PIN1 = 22;  // Direction control pin for RS-485

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

float inversion = 0;

const float DXL_PROTOCOL_VERSION = 2.0;  // Use 2.0 for newer servos

// Initialize the Dynamixel2Arduino library
DynamixelShield dxl(DXL_SERIAL, 2);
//DynamixelShield dxl1(DXL_SERIAL1, DXL_DIR_PIN1);

Servo intServo;
Servo Servo1;


// Use namespace for control table items
using namespace ControlTableItem;

void setup() {

  // Use UART port of DYNAMIXEL Shield to debug.
  Serial.begin(115200);
  dxl.begin(57600);

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  intServo.attach(9);
  Servo1.attach(10);

  dxl.ping(DXL_ID1);
  dxl.ping(DXL_ID2);
  dxl.ping(DXL_ID4);
  dxl.ping(DXL_ID3);
  dxl.ping(DXL_ID5);
  dxl.ping(DXL_ID6);
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
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID1, 50);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID2, 50);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID3, 50);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID4, 50);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID5, 50);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID6, 50);

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 100);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 65);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, 65);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 250);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID5, 120);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID6, 120);
  dxl.setGoalPosition(DXL_ID1, 180, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, 180 + 45, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID3, 180 - 16, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID4, 180 - 90, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID5, 180 + 90, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID6, 180 + 90, UNIT_DEGREE);
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

  if (correct_data[9] == 1) {
    dxl.setGoalPosition(DXL_ID1, dxl.getCurPosition(DXL_ID1));
    dxl.setGoalPosition(DXL_ID2, dxl.getCurPosition(DXL_ID2));
    dxl.setGoalPosition(DXL_ID3, dxl.getCurPosition(DXL_ID3));
    dxl.setGoalPosition(DXL_ID4, dxl.getCurPosition(DXL_ID4));
    dxl.setGoalPosition(DXL_ID5, dxl.getCurPosition(DXL_ID5));
    dxl.setGoalPosition(DXL_ID6, dxl.getCurPosition(DXL_ID6));

    while(true){
      
    }
  }

  if (correct_data[8] == 1) {
    inversion = 180;
  } else if (correct_data[8] == 0) {
    inversion = 0;
  }

  if (correct_data[0] != last_correct_data[0]) {
    dxl.setGoalPosition(DXL_ID1, correct_data[0], UNIT_DEGREE);
    last_correct_data[0] = correct_data[0];
  }
  if (correct_data[1] != last_correct_data[1]) {
    dxl.setGoalPosition(DXL_ID2, correct_data[1] + 45, UNIT_DEGREE);
    last_correct_data[1] = correct_data[1];
  }
  if (correct_data[2] != last_correct_data[2]) {
    dxl.setGoalPosition(DXL_ID4, correct_data[2] - 90, UNIT_DEGREE);
    last_correct_data[2] = correct_data[2];
  }
  if (correct_data[3] != last_correct_data[3] || correct_data[8] != last_correct_data[8]) {
    dxl.setGoalPosition(DXL_ID3, correct_data[3] - 16 + inversion, UNIT_DEGREE);
    last_correct_data[3] = correct_data[3];
    last_correct_data[8] = correct_data[8];
  }
  if (correct_data[4] != last_correct_data[4]) {
    correct_data[4] = abs(correct_data[4] - 360);
    dxl.setGoalPosition(DXL_ID5, correct_data[4], UNIT_DEGREE);
    last_correct_data[4] = correct_data[4];
  }
  if (correct_data[5] != last_correct_data[5]) {
    correct_data[5] = abs(correct_data[5] - 360);
    dxl.setGoalPosition(DXL_ID6, correct_data[5] + 90, UNIT_DEGREE);
    last_correct_data[5] = correct_data[5];
  }
  if (correct_data[6] == 1) {
    intServo.write(150);
  } else if (correct_data[6] == 0) {
    intServo.write(60);
  }
  if (correct_data[7] == 1) {

    Servo1.write(100);
  } else if (correct_data[7] == 0) {
    Servo1.write(150);
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