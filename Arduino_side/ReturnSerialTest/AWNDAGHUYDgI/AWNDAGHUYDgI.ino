bool stringComplete = false;
String inputString = "";
uint16_t data[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float correct_data[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t idx = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}
void loop() {
  // put your main code here, to run repeatedly:
  //if (stringComplete) {
    if (inputString.length() > 0) {
      for (int n = 0; n < inputString.length(); n++) {
        byte new_data = inputString[n];
        bool comma = new_data == 44;       // Fra ASCII tabel
        bool space = new_data == 32;       // Fra ASCII tabel
        bool NL = new_data == '\n';       // Fra ASCII tabel

        uint16_t number = new_data - '0';  // jeg fjerner 48 ('0') fra min byte. Hvis det er et ASCII tal, burde dette tal repræsentere det.

        if (comma) {  //denne byte repræsentere et komma
          idx++;
        } else if (number < 10 && number >= 0) {  // Number from 0-9
          data[idx] = data[idx] * 10 + number;
        } else if(NL){
          stringComplete = true; 
          break; 
        } else if (~space) {  //hvis der sendes en anden type byte er der nok sket en fejl.
          // Note: jeg acceptere og fjerner mellemrum, da folk måske vil skrive dette imellem tegn.
          Serial.println("Recieved a " + String(inputString[n]) + ". Please send 6 numbers in the format x,x,x,x,x,x\n");
        }
      }
      inputString = "";

      if(stringComplete){
        Serial.println("FOUND LINE");

        if (idx == 11) {
          String send_this = "";
          for (int i = 0; i < 12; i++) {
            correct_data[i] = float(data[i]) / 10 - 180;
            send_this = send_this + String(correct_data[i]);
            if(i<11) send_this = send_this+ ",";
            //Serial.print(",");
          }
          Serial.println(send_this);
        } else {
          Serial.println("ERROR: expect 12 data points");
        }
      
        //Serial.println(inputString);
        stringComplete = false;
        for (int i = 0; i < 12; i++) data[i] = 0;
        idx = 0; 
      }

    }
  //}
}
void serialEvent() {
  while (Serial.available()) {
    delay(10);
    if (Serial.available()>0) {
      char inChar = (char)Serial.read();
      inputString += inChar;
    }
    /*if (inChar == '\n') {
      stringComplete = true;
    } else {
    */
    //}
  }
}