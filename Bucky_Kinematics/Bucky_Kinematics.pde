import controlP5.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.geometry.euclidean.threed.*;
import processing.serial.*;


Serial serial;  //used to communicate with the arduino. dunno how. sourse: Søren.
Textarea receivedArea; //sourse: Søren to explain. Think it is a CP5 thing.
Println arduinoConsole;//Søren
ScrollableList portlist;
ScrollableList baudlist;
float theta4, theta5, theta6; //Theta values.
float lastSentValue1, lastSentValue2, lastSentValue3, lastSentValue4, lastSentValue5, lastSentValue6; //Track what values was last sent to the arduino.
boolean connectButtonStatus = false; //Status of the connect button
String selectedport; //Søren
int selectedbaudrate; //Søren
Button cntbutton;


ControlP5 cp5;
Utils utils = new Utils();
Arm Arm1 = new Arm();
Joint jointArray[] = new Joint[4];
Slider slider1, slider2, slider3;
float theta1, theta2, theta3;
float[] theta = {0, 0, 0, 0, 0, 0};

double[][] MDH = {
  {0, 0, 100, 0},
  {0, 0, 0, 0},
  {90, 110, 0, 90},
  {0, 120, 0, 0},
  {90, 0, 130, 0},
  {-90, 0, 0, 0},
  {90, 0, 0, 0}};


void setup() {
  size(1440, 810);
  cp5 = new ControlP5(this);

  for (int i = 0; i<4; i++) {
    jointArray[i] = new Joint(MDH[i]);
    jointArray[i].trans(0);
  }
  
  connectionUI(1000, 400);
  
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
}

void draw() {
  background(125, 125, 250);
  utils.drawResult(theta, 500, 150);
  //utils.drawResult(MDH, 650, 150);
  Arm1.finalMatrix();
  utils.drawResult(Arm1.resultMatrix, 650, 150);

  jointArray[1].trans(Math.toRadians(theta1));
  jointArray[2].trans(Math.toRadians(theta2));
  jointArray[3].trans(Math.toRadians(theta3));
  sendData();
}


















public void sendData() { //Sends some data to arduino. Søren write more comments.
  try {
    if (theta1 != lastSentValue1 || theta2 != lastSentValue2 || theta3 != lastSentValue3 || theta4 != lastSentValue4 || theta5 != lastSentValue5 || theta6 != lastSentValue6) {
      String message = "M1:" + theta1 + "M1end| M2:" + theta2 + "M2end| M3:" + theta3 + "M3end| M4:" + theta4 + "M4end| M5:" + theta5 + "M5end| M6:" + theta6 + "\n";
      serial.write(message);
      lastSentValue1 = theta1;
      lastSentValue2 = theta2;
      lastSentValue3 = theta3;
      lastSentValue4 = theta4;
      lastSentValue5 = theta5;
      lastSentValue6 = theta6;
    }
    String data = serial.readStringUntil('\n');
    if (data != null) {
      data = data.trim();
      receivedArea.setText("Arduino: " + data);
      println("Arduino: " + data);
    }
  }
  catch (Exception e) {
    println("Error opening serial port: " + e.getMessage());
  }
}

void baudratelist(int index) {
  String baudstring;
  baudstring = baudlist.getItem(index).get("name").toString();
  selectedbaudrate = Integer.parseInt(baudstring);
  println("Selected", selectedbaudrate);
}
void comportlist(int index) {
  selectedport = portlist.getItem(index).get("name").toString();
  println("Selected", selectedport);
}
void button() {
  if (!connectButtonStatus) {
    serial = new Serial(this, selectedport, selectedbaudrate);
    cntbutton.setLabel("Disconnect");
    connectButtonStatus = true;
    println("Connected", selectedport, "at", selectedbaudrate);
  } else {
    serial.stop();
    cntbutton.setLabel("Connect");
    connectButtonStatus = false;
    println("Disconnected from", selectedport);
  }
}
public void connectionUI(int x, int y) { //Function that creates the connection UI

  cntbutton = cp5.addButton("button")
    .setLabel("Connect")
    .setSize(70, 30)
    .setPosition(x, y);

  portlist = cp5.addScrollableList("comportlist")
    .setLabel("select port")
    .setBarHeight(30)
    .setPosition(x+100, y)
    .setItemHeight(25);

  baudlist = cp5.addScrollableList("baudratelist")
    .setLabel("select baudrate")
    .setBarHeight(30)
    .setPosition(x+220, y)
    .setItemHeight(24);

  baudlist.addItem("9600", 9600);
  baudlist.addItem("19200", 19200);
  baudlist.addItem("38400", 38400);
  baudlist.addItem("57600", 57600);

  receivedArea = cp5.addTextarea("receivedData")
    .setSize(360, 140)
    .setPosition(x, y+250)
    .setColorBackground(80);
  arduinoConsole = cp5.addConsole(receivedArea);

  String[] availableports = Serial.list(); //   <-------------------- Søren explain plz
  for (int i = 0; i < availableports.length; i++) {
    portlist.addItem(availableports[i], availableports[i]);
  }
}
