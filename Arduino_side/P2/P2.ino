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
String inputString = "";
//const int DXL_DIR_PIN = 2;    // Direction control pin for RS-485
const int DXL_DIR_PIN1 = 22;  // Direction control pin for RS-485


const uint8_t DXL_ID1 = 1;  // Set your Dynamixel servo ID
const uint8_t DXL_ID2 = 2;  // Set your Dynamixel servo ID
const uint8_t DXL_ID4 = 4;  // Set your Dynamixel servo ID
const uint8_t DXL_ID3 = 3;  // Set your Dynamixel servo ID

const uint8_t DXL_ID1b = 1;  // Set your Dynamixel servo ID




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
  //DEBUG_SERIAL1.begin(57600);
  //while (!DEBUG_SERIAL1)
  //  ;
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.

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

  dxl1.torqueOff(DXL_ID1b);
  dxl1.setOperatingMode(DXL_ID1b, OP_POSITION);
  dxl1.torqueOn(DXL_ID1b);

  //dxl1.torqueOff(DXL_ID1b);
  //dxl1.setOperatingMode(DXL_ID1b, OP_POSITION);
  //dxl1.torqueOn(DXL_ID1b);
  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 50);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 50);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID4, 50);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID3, 50);
  dxl1.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1b, 50);

  //readUserInputs();
  dxl1.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1b, 100);
}
void readUserInputs() {
  value1 = readFloat("Enter value 1:");
  Serial.print("Value 1 stored: ");
  Serial.println(value1);

  value2 = readFloat("Enter value 2:");
  Serial.print("Value 2 stored: ");
  Serial.println(value2);

  value3 = readFloat("Enter value 3:");
  Serial.print("Value 3 stored: ");
  Serial.println(value3);

  Serial.println("All values have been recorded.");
}

// Helper function to read a float from the Serial Monitor after printing a prompt.
// This function waits until the user enters a full line (ends with '\n' or '\r').
float readFloat(const char* prompt) {
  Serial.println(prompt);
  String inputString = "";
  char c;

  // Wait until valid input is received
  while (true) {
    // Check if data is available on Serial
    if (Serial.available()) {
      c = Serial.read();

      // Check for end-of-line markers
      if (c == '\n' || c == '\r') {
        if (inputString.length() > 0) {  // Ensure the line isn't empty
          break;
        }
      } else {
        inputString += c;
      }
    }
  }

  return inputString.toFloat();
}

void test3DOF() {
  dxl.setGoalPosition(DXL_ID1, value1, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID2, value2, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_ID4, value3, UNIT_DEGREE);
}


void parseMessage(String msg) {
  for (int i = 0; i < 6; i++) {
    String motorTag = "M" + String(i + 1) + ":";
    String endTag = "M" + String(i + 1) + "end";

    int startIndex = msg.indexOf(motorTag);
    int endIndex = msg.indexOf(endTag);

    if (startIndex != -1 && endIndex != -1) {
      String valueStr = msg.substring(startIndex + motorTag.length(), endIndex);
      float temp = valueStr.toFloat();
      theta[i] = convertAngle(temp, i + 1);
    } else {
      theta[i] = 0;  // Default or error handling
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

void loop() {
  //dxl.setGoalPosition(DXL_ID3, 180, UNIT_DEGREE);

  /*dxl1.setGoalPosition(DXL_ID1b, 2000);
  dxl.setGoalPosition(DXL_ID1, 2000);
  delay(2000);
  dxl.setGoalPosition(1, 500);
  delay(2000);*/
  if (stringComplete) {
    parseMessage(inputString);
    inputString = "";
    stringComplete = false;
    dxl.setGoalPosition(DXL_ID1, theta[0], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID2, theta[1], UNIT_DEGREE);
    dxl.setGoalPosition(DXL_ID4, theta[2], UNIT_DEGREE);
    // Just for testing: print out the extracted theta values
    Serial.print("Theta ");
    Serial.print(0);
    Serial.print(": ");
    Serial.println(theta[0]);
  }

  //test3DOF();
  //delay(10000);
  /*
  // Read and print the servo's current position
  dxl.setGoalPosition(DXL_ID1, 2046);
  dxl.setGoalPosition(DXL_ID2, 2046);
  dxl.setGoalPosition(DXL_ID4, 3000);
  dxl.setGoalPosition(DXL_ID3, 3000);
  //dxl1.setGoalPosition(DXL_ID1b, 3000);

  int pos = dxl.getPresentPosition(DXL_ID1);
  DEBUG_SERIAL.print("Servo Position: ");
  DEBUG_SERIAL.println(pos);
  

  delay(5000);
  dxl.setGoalPosition(DXL_ID1, 1000);
  dxl.setGoalPosition(DXL_ID2, 1546);
  dxl.setGoalPosition(DXL_ID4, 1546);
  dxl.setGoalPosition(DXL_ID3, 2000);
 // dxl1.setGoalPosition(DXL_ID1b, 1000);
   pos = dxl.getPresentPosition(DXL_ID1);
  DEBUG_SERIAL.print("Servo Position: ");
  DEBUG_SERIAL.println(pos);
  delay(5000);
  dxl.setGoalPosition(DXL_ID1, 2546);
  dxl.setGoalPosition(DXL_ID2, 2546);
  dxl.setGoalPosition(DXL_ID4, 1023);
  dxl.setGoalPosition(DXL_ID3, 1000);
 // dxl1.setGoalPosition(DXL_ID1b, 2000);
  pos = dxl.getPresentPosition(DXL_ID1);

  DEBUG_SERIAL.print("Servo Position: ");
  DEBUG_SERIAL.println(pos);
  delay(5000);*/
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