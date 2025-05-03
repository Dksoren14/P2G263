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

int posX, posY; //Declaring a kind of center position of the coordinate system. Initialized in setup.
float panX = 0, panY = 0; //Declaring and making the panning offsets. Startvalue is 0.
float rotX = -1, rotY = -0.75; //Rotation angles (in radians). Start value != 0, so the view starts at a nice angle.
float lastMouseX, lastMouseY; //Used to track where the mouse was last. Used when the mouse is dragged to pan and rotate.
boolean rightMousePressed = false; //Used to track which mouse button is the one being pressed.
boolean leftMousePressed = false;  //Used to track which mouse button is being pressed.
int menuHeight = 50;      //The height the "menu" goes down to. The "menu" is not a object at the moment.
int menuWidth;      //The menu is just some boxes where the color is different and the "mouseDragged()" function doesn't do anything. Initialized in setup.
int zoom = 800;          //Start zoom / start distande from the view.


boolean keyVariableA, keyVariableB, keyVariable1, keyVariable2, keyVariable3, keyVariable4, keyVariable5, keyVariable6, keyVariable7, keyVariable8; //Track key A and B
boolean keyVariableC = true;
float saveThetaValues[][] = new float[4][6];


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


RealMatrix Matrix1232 = new Array2DRowRealMatrix(new double[][] {{15, 20, 30, 40}, {1, 2, 3.5, 4.1}, {10, 29, 30, 40}, {1, 2, 3.2, 4}});
RealMatrix Matrix1233 = new Array2DRowRealMatrix(new double[][] {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}});


void setup() {
  size(1625, 900, P3D);           //Make the canvas/window. Size 1625x by 900y. P3D means it is a 3D "canvas"
  posX = width/2-325;             //x and y positions of the new orego of the coordinate system in terms of the window.
  posY = height-100;                  //Used in a "translate" function in "draw".
  menuWidth = width-375;
  cp5 = new ControlP5(this);
  Arm1 = new Arm(MDH);
  //connectionUI(1000, 400);
  slidersFunction(width-325, 450, 50);
}

void draw() {
  background(125, 125, 250);

  background(200);                                                  //Make background color 200. It goes between black 0-255 white. It is also possible to use (R,G,B) as input.
  directionalLight(126, 126, 126, 0, 0, -1);                        //Make some random ass light. Needed so we can get a perception of the depth of the PShapes.
  ambientLight(102, 102, 102);                                      //Some light shit.
  fill(200, 200, 255);                                              //Fill kinda sets a global variable that shapes use as color. In this case the next rectangle will be colored (R,G,B) (200, 200, 255).
  rect(0, 0, width, menuHeight);                                    //Make rectangle at position 0x 0y with a width of "width" and height of menuHeight.
  rect(menuWidth, menuHeight, width-menuHeight, height-menuHeight); //Look up https://processing.org/reference/ for more information about these kind of things.


  //Under here is where the transformations from the rotate pan zoom functionality happens.
  pushMatrix();                                //Look up the reference sheet "processing.org/reference". It is like making a quicksave before making changes.
  fill(0);
  utils.drawResult("Slider angles", 30, 70);
  utils.drawResult(theta, 30, 120); //Draw slider values


  utils.drawResult("Arm1 result matrix with slider angles (T06)", 175, 100);

  translate(posX + panX, posY + panY, -zoom);  //Translate will move the coordinate system in XYZ. See reference sheet.
  rotateX(-rotX);                              //Self explanatory.
  rotateZ(rotY);
  rectMode(CENTER);
  noStroke();                    //https://processing.org/reference/
  fill(255);                    //Fill() sets a global variable that shapes use as color.
  rect(0, 0, 1000, 1000);      //Ground/talbe/build-area/white-plate/motherfuga
  rectMode(CORNER);
  scale(1, -1, 1);

  pushMatrix();
  translate(-100, 0);
  Arm1.moveAndDraw(theta);

  popMatrix();
  popMatrix();

  utils.drawResult(Arm1.resultMatrix, 175, 150);
  double[] temp1 = Arm1.IK(Arm1.resultMatrix.getData()); //Calculate the IK angle based on the result matrix which is made from slider angles.
  utils.drawResult("IK angles", 800, 70);
  utils.drawResult(temp1, 800, 120);
  utils.drawResult("Arm1 result matrix with IK angles (T06)", 1000, 100);

  pushMatrix();                                //Look up the reference sheet "processing.org/reference". It is like making a quicksave before making changes.
  translate(posX + panX, posY + panY, -zoom);  //Translate will move the coordinate system in XYZ. See reference sheet.
  rotateX(-rotX);                              //Self explanatory.
  rotateZ(rotY);
  rectMode(CENTER);
  noStroke();                    //https://processing.org/reference/
  fill(255);                    //Fill() sets a global variable that shapes use as color.
  rect(0, 0, 1000, 1000);      //Ground/talbe/build-area/white-plate/motherfuga
  rectMode(CORNER);
  scale(1, -1, 1);



  pushMatrix();
  translate(100, 0);
  Arm1.moveAndDraw(temp1);

  popMatrix();
  popMatrix();
  utils.drawResult(Arm1.resultMatrix, 1000, 150);

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



public void slidersFunction(int x, int y, int ya) { //Function that creates theta sliders.
  int start = 0;
  slider1 = cp5.addSlider("theta1")
    .setPosition(x, y)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider2 = cp5.addSlider("theta2")
    .setPosition(x, y + ya)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider3 = cp5.addSlider("theta3")
    .setPosition(x, y + ya * 2)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider4 = cp5.addSlider("theta4")
    .setPosition(x, y + ya * 3)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider5 = cp5.addSlider("theta5")
    .setPosition(x, y + ya * 4)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider6 = cp5.addSlider("theta6")
    .setPosition(x, y + ya * 5)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
}
void slidersFunction1() {
  int start = 0;
  slider1 = cp5.addSlider("theta1")
    .setPosition(150, 100)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider2 = cp5.addSlider("theta2")
    .setPosition(150, 150)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider3 = cp5.addSlider("theta3")
    .setPosition(150, 200)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider4 = cp5.addSlider("theta4")
    .setPosition(150, 250)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider5 = cp5.addSlider("theta5")
    .setPosition(150, 300)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider6 = cp5.addSlider("theta6")
    .setPosition(150, 350)
    .setSize(200, 20)
    .setRange(-90, 90)
    .setValue(start)
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
    //saveThetaValues(1);
  }
  if (keyVariableB == true) {
    //playSavedThetaValues();
  }

  if (keyVariable1 == true) {
    saveThetaValues(0);
    keyVariable1 = false;
  }
  if (keyVariable2 == true) {
    saveThetaValues(1);
    keyVariable2 = false;
  }
  if (keyVariable3 == true) {
    saveThetaValues(2);
    keyVariable3 = false;
  }
  if (keyVariable4 == true) {
    saveThetaValues(3);
    keyVariable4 = false;
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


void mousePressed() { //mousePressed is a built-in function that is called once every time a mouse button is pressed.
  lastMouseX = mouseX; //Record the position of the mouse at the time it is pressed.
  lastMouseY = mouseY;

  if (mouseButton == RIGHT) { //Determine what mouse button is pressed.
    rightMousePressed = true;
  }
  if (mouseButton == LEFT) {
    leftMousePressed = true;
  }
}


void mouseDragged() {                       //mouseDragged is a built-in function that is called every time the mouse moves WHILE a mouse button is pressed.
  float changeX = mouseX - lastMouseX;      //Declaring and finding the change in mouse position since last time the lastMouse was updated.
  float changeY = mouseY - lastMouseY;      //The change is in pixels, so if the mouse moves 10 pixels Y axis, changeY will be 10.

  if (rightMousePressed && mouseY > menuHeight && mouseX < menuWidth) { //Only if the right mouse is clicked, and the mouse is inside the given area.
    //Pan values are used in a "translate()" function at the top of the "draw()" function.
    panX += changeX;                        //They accumulate the change, to be used as a 'offset' in the translate function.
    panY += changeY;                        //The result will be that whatever is drawn afterwards is moved with this offset.
  }                                         //We can see it as the camera moving.
  if (leftMousePressed && mouseY > menuHeight && mouseX < menuWidth) { //Only if the left mouse is clicked, and the mouse is inside the given area.
    //Tilt the same as pan
    rotX += changeY * 0.01;                 //Multiplied with 0.01 to act as sensitivity.
    rotY += changeX * 0.01;                 //It it was not multiplied with 0.01 it would rotate the changeX and changeY in radians, and not a lot of radians is a lot of rotation...
  }

  lastMouseX = mouseX;                      //update lastMouse
  lastMouseY = mouseY;
}

void mouseReleased() {                      //Runs once when mouse is released.
  rightMousePressed = false;
  leftMousePressed = false;
}

void mouseWheel(MouseEvent event) {         //mouseWheel is the same as mousePressed, but for mouse wheel. MouseEvent is to alow us to use shit like ".getCounts" from the mouse.
  if (mouseY > menuHeight && mouseX < menuWidth) {
    float e = event.getCount();               //"event.getCount()" detects scroll direction. Returns "1" or "-1".
    zoom += e * 40;                           //Zoom is also just used in a "translate()" in "draw()". Just on the Z axis (towards and away from view), instead of XY.
    zoom = constrain(zoom, -350, 3000);       //Set zoom limits
    panX += (mouseX-(width/2))*0.1*e;         //This is just a little extra to make the camera zoom kinda towards the mouse position.
    panY += (mouseY-(height/2))*0.1*e;        //It is not accurate, it is just better that nothing.
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
