bool stringComplete = false;
String inputString = "";
float theta[6];
float speed[6];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (stringComplete) {
    parseMessage(inputString);
    inputString = "";
    stringComplete = false;

    for (int i = 0; i < 6; i++) {
      //Serial.print("V");
      //Serial.print(i);
      Serial.println(theta[i]);
      //Serial.print("S");
      //Serial.print(i);
      Serial.println(speed[i]);
    }
  }
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
      speed[i] = convertSpeed(temp);
    } else {
      speed[i] = 0;  // Error fallback
    }
  }
 /* for (int i = 0; i < 12; i++) {
    String motorTag = "M" + String(i + 1);
    String endTag = String(i + 1) + "M";
    int startIndex = msg.indexOf(motorTag);
    int endIndex = msg.indexOf(endTag);

    if (startIndex != -1 && endIndex != -1 && endIndex > startIndex) {
      String valueStr = msg.substring(startIndex + motorTag.length(), endIndex);
      float temp = valueStr.toFloat();

      if (i < 6) {
        theta[i] = convertAngle(temp, i + 1);
      } else {
        speed[i - 6] = temp;
        //convertSpeed(temp);
      }
    } else {
      if (i < 6) {
        theta[i] = 5;
      } else {
        speed[i - 6] = 5;
      }
    }
  }*/
}
float convertSpeed(float speed) {
  float convertedspeed = speed / (6 * 0.229);

  if (convertedspeed < 400) {
    return convertedspeed;
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
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}