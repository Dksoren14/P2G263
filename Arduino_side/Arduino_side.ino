int motor1Value = 0;  // Motor 1
int motor2Value = 0;  // Motor 2
int motor3Value = 0;  // Motor 3
int motor4Value = 0;  // Motor 4
int motor5Value = 0;  // Motor 5
int motor6Value = 0;  // Motor 6

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);  // called this way, it uses the default address 0x40

#define SERVOMIN 150  // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 620

void setup() {
  Serial.begin(9600);  // Start Serial communication;
  board1.begin();
  board1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {

  if (Serial.available() > 0) {
    /*String receivedData = Serial.readStringUntil('\n'); // Read full line
    receivedData.trim(); // Remove extra spaces/newlines
*/
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();
    Serial.println("Hello");
    processSerialData(receivedData);
    //if (receivedData.length() > 0) {
    motor1Value = receivedData.toInt();
    Serial.println(motor1Value);



    //}
  }
}

// Function to process received data
void processSerialData(String data) {
  int m1Index = data.indexOf("M1:");  // Find "M1:"
  int m2Index = data.indexOf("M2:");  // Find "M2:"
  int m3Index = data.indexOf("M3:");  // Find "M3:"
  int m4Index = data.indexOf("M4:");  // Find "M4:"
  int m5Index = data.indexOf("M5:");  // Find "M5:"
  int m6Index = data.indexOf("M6:");  // Find "M6:"

  if (m1Index != -1 && m2Index != -1 && m3Index != -1 && m4Index != -1 && m5Index != -1 && m6Index != -1) {
    int motor1End = data.indexOf("M1end|");  // End of M1 value
    int motor2End = data.indexOf("M2end|");
    int motor3End = data.indexOf("M3end|");
    int motor4End = data.indexOf("M4end|");
    int motor5End = data.indexOf("M5end|");
    int motor6End = data.length();

    // Extract motor 1 value
    motor1Value = data.substring(m1Index + 3, motor1End).toInt();
    motor2Value = data.substring(m2Index + 3, motor2End).toInt();
    motor3Value = data.substring(m3Index + 3, motor3End).toInt();
    motor4Value = data.substring(m4Index + 3, motor4End).toInt();
    motor5Value = data.substring(m5Index + 3, motor5End).toInt();
    // Extract motor 2 value
    motor6Value = data.substring(m6Index + 3, motor6End).toInt();

    // Send confirmation back to Processing
    Serial.print("Motor 1: ");
    Serial.print(motor1Value);
    Serial.print(" | Motor 2: ");
    Serial.print(motor2Value);
    Serial.print("Motor 3: ");
    Serial.print(motor3Value);
    Serial.print(" | Motor 4: ");
    Serial.print(motor4Value);
    Serial.print("Motor 5: ");
    Serial.print(motor5Value);
    Serial.print(" | Motor 6: ");
    Serial.print(motor6Value);
    board1.setPWM(0, 0, map(motor1Value, 0, 180, SERVOMIN, SERVOMAX));
    board1.setPWM(1, 0, motor2Value);
    board1.setPWM(2, 0, motor3Value);
    board1.setPWM(3, 0, motor4Value);
    board1.setPWM(4, 0, motor5Value);
  }
}



int angleToPulse(int ang)  //gets angle in degree and returns the pulse width
{
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max
  //Serial.print("Angle: ");Serial.print(ang);
  //Serial.print(" pulse: ");Serial.println(pulse);
  return pulse;
}
