bool stringComplete = false;
String inputString = "";

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
}
void loop() {
  // put your main code here, to run repeatedly:
  if (stringComplete){
  Serial.println(inputString);
  stringComplete = false;
  inputString = "";
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