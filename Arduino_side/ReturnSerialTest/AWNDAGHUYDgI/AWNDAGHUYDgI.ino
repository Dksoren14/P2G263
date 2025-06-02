bool stringComplete = false;
String inputString = "";

uint16_t data[12];
float correct_data[12];

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (stringComplete) {
    stringComplete = false;

    // Split the string by commas
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

    inputString = "";  // Clear for next message
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
