import controlP5.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.geometry.euclidean.threed.*;
import processing.serial.*;


Serial serial;  //used to communicate with the arduino. dunno how. sourse: Søren.
Textarea receivedArea; //sourse: Søren to explain. Think it is a CP5 thing.
Println arduinoConsole;//Søren
ScrollableList portlist;
ScrollableList baudlist;
float lastSentValue1, lastSentValue2, lastSentValue3, lastSentValue4, lastSentValue5, lastSentValue6; //Track what values was last sent to the arduino.
boolean connectButtonStatus = false; //Status of the connect button
String selectedport; //Søren
int selectedbaudrate; //Søren
Button cntbutton;

boolean keyVariableA, keyVariableB, keyVariable1, keyVariable2, keyVariable3, keyVariable4, keyVariable5, keyVariable6, keyVariable7, keyVariable8; //Track key A and B
boolean keyVariableC = true;
float saveThetaValues[][] = new float[4][7];

ControlP5 cp5;
Utils utils = new Utils();
Arm Arm1;

Slider slider1, slider2, slider3, slider4, slider5, slider6;
float theta1, theta2, theta3, theta4, theta5, theta6; //Theta values
float[] theta = {0, 0, 0, 0, 0, 0};

double[][] MDH = { //Alpha, a, d, theta offset
  {0, 0, 122.65, 0},
  {90, 39.43, 0, 90},
  {0, 115.49, 0, 0},
  {90, 0, 115.49*2, 0},
  {-90, 0, 0, 0},
  {90, 0, 0, 0}};

double[][] pos1 = { //Not used
  {0, 0, 0, 0},
  {0, 0, 100, 0},
  {90, 110, 0, 90},
  {0, 120, 0, 0}};


void setup() {
  size(1440, 810);
  cp5 = new ControlP5(this);
  Arm1 = new Arm(MDH);



  //connectionUI(1000, 400);

  slidersFunction();
}

void draw() {
  background(125, 125, 250);

  Arm1.moveArm(theta);
  utils.drawResult(theta, 30, 120);
  //utils.drawResult(Arm1.resultMatrix03, 450, 150);
  utils.drawResult(Arm1.resultMatrix, 450, 150);
  utils.drawResult("Arm1 result matrix (T06)", 450, 100);
  double[] temp1 = Arm1.IK(Arm1.resultMatrix.getData());
  utils.drawResult(temp1, 1100, 120);
  Arm1.moveArm(temp1);
  utils.drawResult("Arm1 result matrix (T06)", 450, 100);
  utils.drawResult(Arm1.resultMatrix, 450, 450);
  //utils.drawResult(Arm1.Matrix03FromIK, 450, 450);

  //sendData();

  checkButtons();
}







void playSliderValues() {
  theta[0] = theta1;
  theta[1] = theta2;
  theta[2] = theta3;
  theta[3] = theta4;
  theta[4] = theta5;
  theta[5] = theta6;
}


void saveThetaValues(int a) {
  saveThetaValues[a][0] = theta1;
  saveThetaValues[a][1] = theta2;
  saveThetaValues[a][2] = theta3;
  saveThetaValues[a][3] = theta4;
  saveThetaValues[a][4] = theta5;
  saveThetaValues[a][5] = theta6;
}

void playSavedThetaValues(int a) {
  theta[0] = saveThetaValues[a][0];
  theta[1] = saveThetaValues[a][1];
  theta[2] = saveThetaValues[a][2];
  theta[3] = saveThetaValues[a][3];
  theta[4] = saveThetaValues[a][4];
  theta[5] = saveThetaValues[a][5];
}




void slidersFunction() {

  slider1 = cp5.addSlider("theta1")
    .setPosition(150, 100)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
  slider2 = cp5.addSlider("theta2")
    .setPosition(150, 150)
    .setSize(200, 20)
    .setRange(-90, 65)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
  slider3 = cp5.addSlider("theta3")
    .setPosition(150, 200)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
  slider4 = cp5.addSlider("theta4")
    .setPosition(150, 250)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
  slider5 = cp5.addSlider("theta5")
    .setPosition(150, 300)
    .setSize(200, 20)
    .setRange(-90, 65)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
  slider6 = cp5.addSlider("theta6")
    .setPosition(150, 350)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
}




void keyPressed() {         //keyPressed is a built-in function that is called once every time a key is pressed.
  if (keyCode==65) {        //To check what key is pressed, simple "if".
    keyVariableA = true;    //This variable is (at the time of writing this) being used for drawing something. It is therefore made like a flip-flop, to draw it every frame and not just once.
  } else {                  //The variable will only be turned "false" if any other button that ASCII 65 (ASCII 65 = A) is pressed.
    keyVariableA = false;
  }
  if (keyCode==66) {
    keyVariableB = true;
  } else {
    keyVariableB = false;
  }
  if (keyCode==67) {
    if (keyVariableC) {
      keyVariableC = false;
    } else {
      keyVariableC = true;
    }
  }
  if (keyCode==49) {
    keyVariable1 = true;
  } else {
    keyVariable1 = false;
  }
  if (keyCode==50) {
    keyVariable2 = true;
  } else {
    keyVariable2 = false;
  }
  if (keyCode==51) {
    keyVariable3 = true;
  } else {
    keyVariable3 = false;
  }
  if (keyCode==52) {
    keyVariable4 = true;
  } else {
    keyVariable4 = false;
  }
  if (keyCode==53) {
    keyVariable5 = true;
  } else {
    keyVariable5 = false;
  }
  if (keyCode==54) {
    keyVariable6 = true;
  } else {
    keyVariable6 = false;
  }
  if (keyCode==55) {
    keyVariable7 = true;
  } else {
    keyVariable7 = false;
  }
  if (keyCode==56) {
    keyVariable8 = true;
  } else {
    keyVariable8 = false;
  }
}

void checkButtons() {
  if (keyVariableA == true) {
    saveThetaValues(1);
  }
  if (keyVariableB == true) {
    //playSavedThetaValues();
  }

  if (keyVariable1 == true) {
    saveThetaValues(0);
  }
  if (keyVariable2 == true) {
    saveThetaValues(1);
  }
  if (keyVariable3 == true) {
    saveThetaValues(2);
  }
  if (keyVariable4 == true) {
    saveThetaValues(3);
  }

  if (keyVariableC == true) {
    playSliderValues();
  } else {

    if (keyVariable5 == true) {
      playSavedThetaValues(0);
    }
    if (keyVariable6 == true) {
      playSavedThetaValues(1);
    }
    if (keyVariable7 == true) {
      playSavedThetaValues(2);
    }
    if (keyVariable8 == true) {
      playSavedThetaValues(3);
    }
  }
}






//public void sendData() { //Sends some data to arduino. Søren write more comments.
//  try {
//    if (theta[1] != lastSentValue1 || theta[2] != lastSentValue2 || theta[3] != lastSentValue3 || theta[4] != lastSentValue4 || theta[5] != lastSentValue5 || theta[6] != lastSentValue6) {
//      String message = "M1:" + theta[1] + "M1end| M2:" + theta[2] + "M2end| M3:" + theta[3] + "M3end| M4:" + theta[4] + "M4end| M5:" + theta[5] + "M5end| M6:" + theta[6] + "\n";
//      serial.write(message);
//      lastSentValue1 = theta[1];
//      lastSentValue2 = theta[2];
//      lastSentValue3 = theta[3];
//      lastSentValue4 = theta[4];
//      lastSentValue5 = theta[5];
//      lastSentValue6 = theta[6];
//    }
//    String data = serial.readStringUntil('\n');
//    if (data != null) {
//      data = data.trim();
//      receivedArea.setText("Arduino: " + data);
//      println("Arduino: " + data);
//    }
//  }
//  catch (Exception e) {
//    println("Error opening serial port: " + e.getMessage());
//  }
//}

//void baudratelist(int index) {
//  String baudstring;
//  baudstring = baudlist.getItem(index).get("name").toString();
//  selectedbaudrate = Integer.parseInt(baudstring);
//  println("Selected", selectedbaudrate);
//}
//void comportlist(int index) {
//  selectedport = portlist.getItem(index).get("name").toString();
//  println("Selected", selectedport);
//}
//void button() {
//  if (!connectButtonStatus) {
//    serial = new Serial(this, selectedport, selectedbaudrate);
//    cntbutton.setLabel("Disconnect");
//    connectButtonStatus = true;
//    println("Connected", selectedport, "at", selectedbaudrate);
//  } else {
//    serial.stop();
//    cntbutton.setLabel("Connect");
//    connectButtonStatus = false;
//    println("Disconnected from", selectedport);
//  }
//}
//public void connectionUI(int x, int y) { //Function that creates the connection UI

//  cntbutton = cp5.addButton("button")
//    .setLabel("Connect")
//    .setSize(70, 30)
//    .setPosition(x, y);

//  portlist = cp5.addScrollableList("comportlist")
//    .setLabel("select port")
//    .setBarHeight(30)
//    .setPosition(x+100, y)
//    .setItemHeight(25);

//  baudlist = cp5.addScrollableList("baudratelist")
//    .setLabel("select baudrate")
//    .setBarHeight(30)
//    .setPosition(x+220, y)
//    .setItemHeight(24);

//  baudlist.addItem("9600", 9600);
//  baudlist.addItem("19200", 19200);
//  baudlist.addItem("38400", 38400);
//  baudlist.addItem("57600", 57600);

//  receivedArea = cp5.addTextarea("receivedData")
//    .setSize(360, 140)
//    .setPosition(x, y+250)
//    .setColorBackground(80);
//  arduinoConsole = cp5.addConsole(receivedArea);

//  String[] availableports = Serial.list(); //   <-------------------- Søren explain plz
//  for (int i = 0; i < availableports.length; i++) {
//    portlist.addItem(availableports[i], availableports[i]);
//  }
//}
