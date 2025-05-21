import controlP5.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.geometry.euclidean.threed.*;
import processing.serial.*;

Serial serial, serial1;  //used to communicate with the arduino.
Textarea receivedArea, receivedArea1;
Println arduinoConsole, arduinoConsole1;
ScrollableList portlist, portlist1;
ScrollableList baudlist, baudlist1;
float[] lastSentValue = new float[15]; //Track what values was last sent to the arduino.
boolean connectButtonStatus = false; //Status of the connect button
boolean connectButtonStatus1 = false;
String selectedport, selectedport1;
int selectedbaudrate, selectedbaudrate1;

Button connectionButton, connectionButton1, toggleConnectionUIButton, infoButton, saveProgramPointButton, leftArrowButton, rightArrowButton, addPointButton, saveProgramButton, editProgramLocationButton, playProgramButton, toggleSaveLoadUIButton, loadProgramButton;
Button moveMethodToggleButton, conformXYZRPYButton, sendToRobotButton, gripperButton, vialBoxButton, inversionButton;
Textfield programSelectionTextField, timeTextField, pxTextField, pyTextField, pzTextField, rTextField, pTextField, yTextField;
double[][][] globalTemp2;
boolean toggleUIBool = false; //Status of the "toggleUI" button.
boolean toggleSaveLoadUIBool = true;
boolean toggleInputMethodBool = true;
boolean infoButtonVariable = false;
boolean gripperBoolean = false;
boolean vialBoxBoolean = false;
boolean inversionBoolean = false;
int gripperVariable = 0;
int vialBoxVariable = 0;
int inversionVariable = 0;
boolean keyVariableA, keyVariableB, keyVariable1, keyVariable2, keyVariable3, keyVariable4, keyVariable5, keyVariable6, keyVariable7, keyVariable8; //Track key A and B
boolean loopVariable0, loopVariable1, loopVariable2, loopVariable3;
boolean keyVariableC = true;
boolean keyVariableD = false;
boolean keyVariableE = false;
boolean keyVariableF = false;

int movementNumber = 0;
int addingPoint = 0;
String currentProgram = "untitled program";

int posX, posY; //Declaring a kind of center position of the coordinate system. Initialized in setup.
float panX = 0, panY = 0; //Declaring and making the panning offsets. Startvalue is 0.
float rotX = -1, rotY = -0.75; //Rotation angles (in radians). Start value != 0, so the view starts at a nice angle.
float lastMouseX, lastMouseY; //Used to track where the mouse was last. Used when the mouse is dragged to pan and rotate.
boolean rightMousePressed = false; //Used to track which mouse button is the one being pressed.
boolean leftMousePressed = false;  //Used to track which mouse button is being pressed.
int menuHeight = 50;      //The height the "menu" goes down to. The "menu" is not a object at the moment.
int menuWidth;      //The menu is just some boxes where the color is different and the "mouseDragged()" function doesn't do anything. Initialized in setup.
int zoom = 400;          //Start zoom / start distande from the view.


float saveThetaValues[][] = new float[4][6];


ControlP5 cp5;
Utils utils = new Utils();
Arm Arm1, Arm2;

Slider slider1, slider2, slider3, slider4, slider5, slider6;
float theta1, theta2, theta3, theta4, theta5, theta6; //Theta values
float[] theta = {0, 0, 0, 0, 0, 0};
//float[] startTheta;
double[] time = {0, 0, 0, 0};
boolean runFastLoop = false;
Thread fastThread;

double[][] MDH = { //Alpha, a, d, theta offset
  {0, 0, 122.65, 0},
  {90, 39.43, 0, 90},
  {0, 115.49, 0, 0},
  {90, 0, 153, 0},
  {-90, 0, 0, 0},
  {90, 0, 0, 0}};

double[][][] movementProgram = {{
    {3000, 0, 0, 0}, //Time, nothing, nothing, nothing
    {-0.8336, 0.5135, 0.2038, 61.43}, //r, r, r, Px
    {0.326, -0.755, -0.5689, -111.75}, //r, r, r, Py
    {0.4459, -0.4078, 0.7968, 322.42}, //r, r, r, Pz
    {0, 0, 0, 1}//0, 0, 0, 1
  }, {
    {3000, 0, 0, 0},
    {-0.2181, 0, 0.9759, 192.43},
    {0, -1, 0, 0},
    {0.9759, 0, 0.2181, 238.14},
    {0, 0, 0, 1}
}};

double[][][] movementProgram1232;

int switchProgramVariable = 0;

RealMatrix Matrix1232 = new Array2DRowRealMatrix(new double[][] {{15, 20, 30, 40}, {1, 2, 3.5, 4.1}, {10, 29, 30, 40}, {1, 2, 3.2, 4}});
RealMatrix Matrix1233 = new Array2DRowRealMatrix(new double[][] {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}});


PShape[] textures = new PShape[7];  //The .obj files of the component models will be loaded in to this "PShape" data type.


void setup() {
  size(1625, 900, P3D);           //Make the canvas/window. Size 1625x by 900y. P3D means it is a 3D "canvas"
  posX = (width-325)/2;             //x and y positions of the new orego of the coordinate system in terms of the window.
  posY = height-250;                  //Used in a "translate" function in "draw".
  menuWidth = width-375;

  textures[0] = loadShape("obj_files/Base.obj");
  textures[1] = loadShape("obj_files/Link1.obj");
  textures[2] = loadShape("obj_files/Link2.obj");
  textures[3] = loadShape("obj_files/Link3.obj");
  textures[4] = loadShape("obj_files/Link4.obj");
  textures[5] = loadShape("obj_files/Link5.obj");


  cp5 = new ControlP5(this);
  Arm1 = new Arm(MDH, textures);
  Arm2 = new Arm(MDH, textures);


  makeSlidersFunction(width-325, 450, 50);
  connectionUI(10, 10);
  saveLoadUI(width-325, 70);
  infoButton = cp5.addButton("infoButton") //Make button "toggleUI".
    .setLabel("Info")
    .setSize(100, 30)
    .setPosition(130, 10);
  frameRate(60);
}

void draw() {

  background(200);                                                  //Make background color 200. It goes between black 0-255 white. It is also possible to use (R,G,B) as input.
  directionalLight(126, 126, 126, 0, 0, -1);                        //Make some random ass light. Needed so we can get a perception of the depth of the PShapes.
  ambientLight(102, 102, 102);                                      //Some light shit.

  fill(200, 200, 255);                                              //Fill kinda sets a global variable that shapes use as color. In this case the next rectangle will be colored (R,G,B) (200, 200, 255).
  rect(0, 0, width, menuHeight);                                    //Make rectangle at position 0x 0y with a width of "width" and height of menuHeight.
  rect(menuWidth, menuHeight, width-menuHeight, height-menuHeight); //Look up https://processing.org/reference/ for more information about these kind of things.
  checkKeyPressed();
  loop2();

  if (infoButtonVariable) {
    utils.drawResult("Slider angles", 30, 70);
    utils.drawResult(theta, 30, 120); //Draw slider values
    utils.drawResult("Arm1 result matrix with slider angles (T06)", 175, 100);
    utils.drawResult(Arm1.resultMatrix, 175, 150);
    utils.drawResult("IK angles", 800, 70);
    utils.drawResult(Arm1.anglesFromIK(Arm1.resultMatrix.getData()), 800, 120);
    utils.drawResult("Arm1 result matrix with IK angles (T06)", 1000, 100);
    utils.drawResult(Arm2.resultMatrix, 1000, 150);
    utils.drawResult(Arm2.pos, 900, 450);
    utils.drawResult(Arm2.speed, 1050, 450);
    utils.drawResult(Arm2.acc, 1200, 450);
    //utils.drawResult(rotationMatrixFromAngles12332, 175, 400);
    utils.drawResult((double)gripperVariable, 175, 400);
    utils.drawResult((double)vialBoxVariable, 175, 450);
    utils.drawResult((double)inversionVariable, 175, 500);
  }





  //Under this is where the transformations from the rotate pan zoom functionality happens.
  pushMatrix();
  translate(posX + panX, posY + panY, -zoom);
  rotateX(-rotX);
  rotateZ(rotY);

  clip(0, menuHeight, menuWidth, height);

  rectMode(CENTER);
  noStroke();
  fill(255);
  rect(0, 0, 1000, 1000);
  rectMode(CORNER);
  scale(1, -1, 1);


  pushMatrix();
  translate(-300, 0);
  //Arm1.draw(theta);
  Arm1.draw();
  popMatrix();


  pushMatrix();
  translate(100, 0);
  Arm2.draw();
  popMatrix();
  noClip();

  popMatrix();


  if (toggleSaveLoadUIBool) {
    drawSaveLoadUI(width-325, 70);
  }
  //utils.drawResult(time[3], 10, 750);
  //Arm2.sendData();
}









void playSliderValues() {
  //theta[0] = theta1;
  //theta[1] = theta2;
  //theta[2] = theta3;
  //theta[3] = theta4;
  //theta[4] = theta5;
  //theta[5] = theta6;
  Arm1.pos[0] = theta1;
  Arm1.pos[1] = theta2;
  Arm1.pos[2] = theta3;
  Arm1.pos[3] = theta4;
  Arm1.pos[4] = theta5;
  Arm1.pos[5] = theta6;
}

//void saveThetaValues(int a) {
//  saveThetaValues[a][0] = theta1;
//  saveThetaValues[a][1] = theta2;
//  saveThetaValues[a][2] = theta3;
//  saveThetaValues[a][3] = theta4;
//  saveThetaValues[a][4] = theta5;
//  saveThetaValues[a][5] = theta6;
//}


//void playSavedThetaValues(int a) {
//  theta[0] = saveThetaValues[a][0];
//  theta[1] = saveThetaValues[a][1];
//  theta[2] = saveThetaValues[a][2];
//  theta[3] = saveThetaValues[a][3];
//  theta[4] = saveThetaValues[a][4];
//  theta[5] = saveThetaValues[a][5];
//}

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

void toggleConnectionUI() { //Will toggle the UI. Runs when "toggleUI" button is pressed.

  toggleUIBool = !toggleUIBool;
  connectionButton.setVisible(toggleUIBool);
  portlist.setVisible(toggleUIBool);
  baudlist.setVisible(toggleUIBool);
  receivedArea.setVisible(toggleUIBool);
  connectionButton1.setVisible(toggleUIBool);
  portlist1.setVisible(toggleUIBool);
  baudlist1.setVisible(toggleUIBool);
  receivedArea1.setVisible(toggleUIBool);
}



public void sendData() { //Sends some data to arduino. Søren write more comments.
  //try {
  //  if (theta[0] != lastSentValue[0] || theta[1] != lastSentValue[1] || theta[2] != lastSentValue[2] || theta[3] != lastSentValue[3] || theta[4] != lastSentValue[4] || theta[5] != lastSentValue[5]) {
  //    String message = "M1:" + theta[0] + "M1end| M2:" + theta[1] + "M2end| M3:" + theta[2] + "M3end| M4:" + theta[3] + "M4end| M5:" + theta[4] + "M5end| M6:" + theta[5] + "\n";
  //    serial.write(message);
  //    for (int i = 0; i < theta.length; i++) {
  //      lastSentValue[i] = theta[i];
  //    }
  //  }
  //  String data = serial.readStringUntil('\n');
  //  if (data != null) {
  //    data = data.trim();
  //    receivedArea.setText("Arduino: " + data);
  //    println("Arduino: " + data);
  //  }
  //}
  //catch (Exception e) {
  //  println("Error opening serial port: " + e.getMessage());
  //}
}

public void readData() {
}

void baudratelistFunction(int index) {
  String baudstring;
  baudstring = baudlist.getItem(index).get("name").toString();
  selectedbaudrate = Integer.parseInt(baudstring);
  println("Selected", selectedbaudrate);
}
void comportlistFunction(int index) {
  selectedport = portlist.getItem(index).get("name").toString();
  println("Selected", selectedport);
}
void connectButtonFunction() {
  if (!connectButtonStatus) {
    serial = new Serial(this, selectedport, selectedbaudrate);
    connectionButton.setLabel("Disconnect");
    connectButtonStatus = true;
    println("Connected", selectedport, "at", selectedbaudrate);
  } else {
    serial.stop();
    connectionButton.setLabel("Connect");
    connectButtonStatus = false;
    println("Disconnected from", selectedport);
  }
}
void baudratelistFunction1(int index) {
  String baudstring1;
  baudstring1 = baudlist1.getItem(index).get("name").toString();
  selectedbaudrate1 = Integer.parseInt(baudstring1);
  println("Selected", selectedbaudrate1);
}
void comportlistFunction1(int index) {
  selectedport1 = portlist1.getItem(index).get("name").toString();
  println("Selected", selectedport1);
}
void connectButtonFunction1() {
  if (!connectButtonStatus1) {
    serial1 = new Serial(this, selectedport1, selectedbaudrate1);
    connectionButton1.setLabel("Disconnect");
    connectButtonStatus1 = true;
    println("Connected", selectedport1, "at", selectedbaudrate1);
  } else {
    serial1.stop();
    connectionButton1.setLabel("Connect");
    connectButtonStatus1 = false;
    println("Disconnected from", selectedport1);
  }
}


void connectionUI(int x, int y) { //Function that creates the connection UI
  toggleConnectionUIButton = cp5.addButton("toggleConnectionUI") //Make button "toggleUI".
    .setLabel("Connection UI")
    .setSize(100, 30)
    .setPosition(x, y);
  y=y+50;
  connectionButton = cp5.addButton("connectButtonFunction")
    .setLabel("Connect")
    .setSize(70, 30)
    .setPosition(x, y);

  portlist = cp5.addScrollableList("comportlistFunction")
    .setLabel("select port")
    .setBarHeight(30)
    .setPosition(x+100, y)
    .setItemHeight(25);

  baudlist = cp5.addScrollableList("baudratelistFunction")
    .setLabel("select baudrate")
    .setBarHeight(30)
    .setPosition(x+220, y)
    .setItemHeight(24);

  baudlist.addItem("9600", 9600);
  baudlist.addItem("19200", 19200);
  baudlist.addItem("38400", 38400);
  baudlist.addItem("57600", 57600);
  baudlist.addItem("115200", 115200);

  receivedArea = cp5.addTextarea("receivedData")
    .setSize(360, 140)
    .setPosition(x, y+250)
    .setColorBackground(80);
  arduinoConsole = cp5.addConsole(receivedArea);
  x = x+400;
  connectionButton1 = cp5.addButton("connectButtonFunction1")
    .setLabel("Connect")
    .setSize(70, 30)
    .setPosition(x, y);

  portlist1 = cp5.addScrollableList("comportlistFunction1")
    .setLabel("select port")
    .setBarHeight(30)
    .setPosition(x+100, y)
    .setItemHeight(25);

  baudlist1 = cp5.addScrollableList("baudratelistFunction1")
    .setLabel("select baudrate")
    .setBarHeight(30)
    .setPosition(x+220, y)
    .setItemHeight(24);

  baudlist1.addItem("9600", 9600);
  baudlist1.addItem("19200", 19200);
  baudlist1.addItem("38400", 38400);
  baudlist1.addItem("57600", 57600);
  baudlist1.addItem("115200", 115200);

  receivedArea1 = cp5.addTextarea("receivedData1")
    .setSize(360, 140)
    .setPosition(x, y+250)
    .setColorBackground(80);
  arduinoConsole1 = cp5.addConsole(receivedArea1);

  String[] availableports = Serial.list(); //   <-------------------- Søren explain plz
  for (int i = 0; i < availableports.length; i++) {
    portlist.addItem(availableports[i], availableports[i]);
    portlist1.addItem(availableports[i], availableports[i]);
  }

  connectionButton.setVisible(false);
  portlist.setVisible(false);
  baudlist.setVisible(false);
  receivedArea.setVisible(false);
  connectionButton1.setVisible(false);
  portlist1.setVisible(false);
  baudlist1.setVisible(false);
  receivedArea1.setVisible(false);
}

void infoButton() {
  infoButtonVariable = !infoButtonVariable;
}

public void makeSlidersFunction(int x, int y, int space) { //Function that creates theta sliders.
  int start = 0;
  slider1 = cp5.addSlider("theta1")
    .setPosition(x, y)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider2 = cp5.addSlider("theta2")
    .setPosition(x, y + space)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider3 = cp5.addSlider("theta3")
    .setPosition(x, y + space * 2)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider4 = cp5.addSlider("theta4")
    .setPosition(x, y + space * 3)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider5 = cp5.addSlider("theta5")
    .setPosition(x, y + space * 4)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
  slider6 = cp5.addSlider("theta6")
    .setPosition(x, y + space * 5)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(start)
    .setColorCaptionLabel(color(20, 20, 20));
}

void keyPressed() {         //keyPressed is a built-in function that is called once every time a key is pressed.
  if (keyCode==65) {        //To check what key is pressed, simple "if".
    //keyVariableA = true;    //This variable is (at the time of writing this) being used for drawing something. It is therefore made like a flip-flop, to draw it every frame and not just once.
  }
  if (keyCode==66) {
    keyVariableB = !keyVariableB;
  }
  if (keyCode==67) {
    keyVariableC = !keyVariableC;
  }
  if (keyCode==68) {
    keyVariableD = !keyVariableD;
  }
  if (keyCode==69) {
    keyVariableE = !keyVariableE;
  }
  if (keyCode==70) {
    keyVariableF = !keyVariableF;
    //tempVariableForF += 1;
  }
  if (keyCode==49) {
    keyVariable1 = !keyVariable1;
  }
  if (keyCode==50) {
    keyVariable2 = !keyVariable2;
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




void checkKeyPressed() { //----------------------------------------------------------------------------------------
  if (keyVariableA) { //Movement program
    if (Arm2.executeProgram(movementProgram) == 1) {
      keyVariableA = false;
    }
    Arm2.sendData();
    time[1] = (double)millis()-time[0];
    time[0] = millis();
    utils.drawResult(time[1], 10, 700);
  }

  if (keyVariableB) {
    //movementProgram = Arm1.savePointToProgram(movementProgram, 1000);
    saveStrings("dataOut1", Arm1.messageArrayOut);
    saveStrings("dataIn1", Arm1.messageArrayIn);
    saveStrings("dataOut2", Arm2.messageArrayOut);
    saveStrings("dataIn2", Arm2.messageArrayIn);
    keyVariableB = false;
  }
  if (keyVariableD) {
    //saveProgramToFile(movementProgram, "movementProgram1");
    keyVariableD = false;
  }
  if (keyVariableE) {
    //movementProgram1232 = loadProgramFromFile("movementProgram1");
    keyVariableE = false;
  }
  if (keyVariableF) {
    time[1] = (double)millis()-time[0];
    time[0] = millis();
    utils.drawResult(time[1], 10, 700);
  }

  if (keyVariable1) {
    keyVariable1 = false;
  }
  if (keyVariable2) {
    keyVariable2 = false;
  }
  if (keyVariable3) {
    //saveThetaValues(2);
    keyVariable3 = false;
  }
  if (keyVariable4) {
    //saveThetaValues(3);
    keyVariable4 = false;
  }

  if (keyVariableC) {
    playSliderValues();
    pushStyle();
    fill(255, 0, 0);
    rect(width-50, height-50, 10, 10);
    popStyle();
  } else {

    if (keyVariable5) {
      //playSavedThetaValues(0);
    }
    if (keyVariable6) {
      //playSavedThetaValues(1);
    }
    if (keyVariable7) {
      //playSavedThetaValues(2);
    }
    if (keyVariable8) {
      //playSavedThetaValues(3);
    }
  }
}

void loop2() {
  if (loopVariable0) {
    if (Arm1.executeProgram(globalTemp2) == 1) {
      loopVariable0 = false;
    }
  }
  if (loopVariable1) {
  }
  if (loopVariable2) {
  }
  if (loopVariable3) {
  }
}

void toggleSaveLoadUI() { //Will toggle the UI. Runs when "toggleUI" button is pressed.

  toggleSaveLoadUIBool = !toggleSaveLoadUIBool;
  //showConnectionUI(toggleUIBool);
  saveProgramPointButton.setVisible(toggleSaveLoadUIBool);
  leftArrowButton.setVisible(toggleSaveLoadUIBool);
  rightArrowButton.setVisible(toggleSaveLoadUIBool);
  programSelectionTextField.setVisible(toggleSaveLoadUIBool);
  playProgramButton.setVisible(toggleSaveLoadUIBool);
  addPointButton.setVisible(toggleSaveLoadUIBool);
  saveProgramButton.setVisible(toggleSaveLoadUIBool);
  loadProgramButton.setVisible(toggleSaveLoadUIBool);
  editProgramLocationButton.setVisible(toggleSaveLoadUIBool);
  timeTextField.setVisible(toggleSaveLoadUIBool);
}

void drawSaveLoadUI(int x, int y) {
  pushStyle();
  fill(0);
  textSize(24);
  text("Current Program: ", x + 5, y);
  text(currentProgram, x + 5, y + 30);
  y = y + 80;
  textSize(30);
  text("Point " + (movementNumber + addingPoint), x + 5, y + 70);
  textSize(24);
  text("Time: ", x + 5, y+193);
  if (gripperBoolean) {
    fill(255, 0, 0);
    rect(width-110, height-575, 20, 20);
  }
  if (vialBoxBoolean) {
    fill(255, 0, 0);
    rect(width-110, height-535, 20, 20);
  }
  if (inversionBoolean) {
    fill(255, 0, 0);
    rect(width-110, height-495, 20, 20);
  }
  popStyle();
}

void saveLoadUI(int x, int y) {
  toggleSaveLoadUIButton = cp5.addButton("toggleSaveLoadUI") //Make button "toggleUI".
    .setLabel("Save/Load")
    .setSize(100, 30)
    .setPosition(240, 10);
  y = y + 40;
  programSelectionTextField = cp5.addTextfield("programSelectionTextFieldFunction")
    .setLabel("")
    .setPosition(x, y)
    .setSize(200, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("untitled program");
  saveProgramButton = cp5.addButton("saveProgramButtonFunction")
    .setLabel("Save Program")
    .setSize(60, 30)
    .setPosition(x + 140, y + 40);
  loadProgramButton = cp5.addButton("loadProgramButtonFunction")
    .setLabel("Load Program")
    .setSize(60, 30)
    .setPosition(x, y + 40);
  y = y + 40;
  leftArrowButton = cp5.addButton("leftArrowButtonFunction")
    .setLabel("<---")
    .setSize(60, 30)
    .setPosition(x, y + 90);
  playProgramButton = cp5.addButton("playProgramButtonFunction")
    .setLabel("Play")
    .setSize(60, 30)
    .setPosition(x + 70, y + 90);
  rightArrowButton = cp5.addButton("rightArrowButtonFunction")
    .setLabel("--->")
    .setSize(60, 30)
    .setPosition(x + 140, y + 90);
  addPointButton = cp5.addButton("addPointButtonFunction")
    .setLabel("Add Point")
    .setSize(60, 30)
    .setPosition(x + 140, y + 130);
  editProgramLocationButton = cp5.addButton("editProgramLocationButtonFunction")
    .setLabel("Edit Point")
    .setSize(60, 30)
    .setPosition(x + 70, y + 130);
  saveProgramPointButton = cp5.addButton("saveProgramPointButtonFunction")
    .setLabel("Save Point")
    .setSize(60, 30)
    .setPosition(x, y + 130);
  timeTextField = cp5.addTextfield("timeTextFieldFunction")
    .setLabel("")
    .setPosition(x+70, y+170)
    .setSize(60, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("1234");
  gripperButton = cp5.addButton("gripperOnButtonFunction")
    .setLabel("Gripper On")
    .setSize(60, 30)
    .setPosition(x+140, y+170);
  vialBoxButton = cp5.addButton("vialBoxButtonFunction")
    .setLabel("Box Out")
    .setSize(60, 30)
    .setPosition(x+140, y+210);
  inversionButton = cp5.addButton("inversionButtonFunction")
    .setLabel("Invert 3")
    .setSize(60, 30)
    .setPosition(x+140, y+250);
  moveMethodToggleButton = cp5.addButton("moveMethodToggleButtonFunction")
    .setLabel("Toggle Input Method")
    .setSize(120, 30)
    .setPosition(x, y + 230);
  y = y + 20;
  int waka1232 = 20;
  pxTextField = cp5.addTextfield("pxTextFieldFunction")
    .setLabel("")
    .setPosition(x+waka1232, y+270)
    .setSize(60, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("192.43");
  pyTextField = cp5.addTextfield("pyTextFieldFunction")
    .setLabel("")
    .setPosition(x+waka1232, y+310)
    .setSize(60, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("0");
  pzTextField = cp5.addTextfield("pzTextFieldFunction")
    .setLabel("")
    .setPosition(x+waka1232, y+350)
    .setSize(60, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("238.14");
  y = y + 120;
  rTextField = cp5.addTextfield("rTextFieldFunction")
    .setLabel("")
    .setPosition(x+waka1232, y+270)
    .setSize(60, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("0");
  pTextField = cp5.addTextfield("pTextFieldFunction")
    .setLabel("")
    .setPosition(x+waka1232, y+310)
    .setSize(60, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("180");
  yTextField = cp5.addTextfield("yTextFieldFunction")
    .setLabel("")
    .setPosition(x+waka1232, y+350)
    .setSize(60, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("0");
  conformXYZRPYButton = cp5.addButton("conformXYZRPYButtonFunction")
    .setLabel("Confirm")
    .setSize(120, 30)
    .setColorBackground(color(0, 170, 0))
    .setColorForeground(color(30, 245, 30))
    .setColorActive(color(255, 255, 255))
    .setPosition(x, y + 400);
  sendToRobotButton = cp5.addButton("sendToRobotButtonFunction")
    .setLabel("SEND TO ROBOT!")
    .setSize(120, 30)
    .setColorBackground(color(200, 0, 0))
    .setColorForeground(color(255, 50, 50))
    .setPosition(x, y + 450);


  pxTextField.setVisible(false);
  pyTextField.setVisible(false);
  pzTextField.setVisible(false);
  rTextField.setVisible(false);
  pTextField.setVisible(false);
  yTextField.setVisible(false);
  conformXYZRPYButton.setVisible(false);
}

void leftArrowButtonFunction() {
  keyVariableC = false;
  addingPoint = 0;
  movementNumber -= 1;
  movementNumber = constrain(movementNumber, 1, movementProgram.length);
  double[][] temp = {movementProgram[movementNumber-1][1], movementProgram[movementNumber-1][2], movementProgram[movementNumber-1][3], movementProgram[movementNumber-1][4]};
  //Arm1.executeMovement(temp, 1);
  double[] temp2 = Arm1.anglesFromIK(temp);
  gripperVariable = (int)movementProgram[movementNumber-1][0][1];
  vialBoxVariable = (int)movementProgram[movementNumber-1][0][2];
  inversionVariable = (int)movementProgram[movementNumber-1][0][3];
  for (int i = 0; i < temp2.length; i++) {
    Arm1.pos[i] = (float)temp2[i];
  }
}
void rightArrowButtonFunction() {
  keyVariableC = false;
  addingPoint = 0;
  movementNumber += 1;
  movementNumber = constrain(movementNumber, 1, movementProgram.length);
  if (movementNumber <= movementProgram.length) {
    double[][] temp = {movementProgram[movementNumber-1][1], movementProgram[movementNumber-1][2], movementProgram[movementNumber-1][3], movementProgram[movementNumber-1][4]};
    double[] temp2 = Arm1.anglesFromIK(temp);
    gripperVariable = (int)movementProgram[movementNumber-1][0][1];
    vialBoxVariable = (int)movementProgram[movementNumber-1][0][2];
    inversionVariable = (int)movementProgram[movementNumber-1][0][3];
    for (int i = 0; i < temp2.length; i++) {
      Arm1.pos[i] = (float)temp2[i];
    }
  }
}
void gripperOnButtonFunction() {
  gripperBoolean = !gripperBoolean;
  if (gripperBoolean) {
    gripperVariable = 10;
  } else {
    gripperVariable = 0;
  }
}
void vialBoxButtonFunction() {
  vialBoxBoolean = !vialBoxBoolean;
  if (vialBoxBoolean) {
    vialBoxVariable = 10;
  } else {
    vialBoxVariable = 0;
  }
}
void inversionButtonFunction() {
  inversionBoolean = !inversionBoolean;
  if (inversionBoolean) {
    inversionVariable = 10;
  } else {
    inversionVariable = 0;
  }
}
void saveProgramPointButtonFunction() {
  int temp;
  if (cp5.get(Textfield.class, "timeTextFieldFunction").getText().isEmpty()) {
    temp = 1234;
  } else {
    temp = Integer.parseInt(cp5.get(Textfield.class, "timeTextFieldFunction").getText());
  }
  movementProgram = Arm1.savePointToProgram(movementProgram, temp, gripperVariable, vialBoxVariable, inversionVariable, movementNumber-1+addingPoint);
  addingPoint = 0;
  rightArrowButtonFunction();
}
void editProgramLocationButtonFunction() {
  addingPoint = 0;
  keyVariableC = true;
}
void addPointButtonFunction() {
  movementNumber = movementProgram.length;
  addingPoint = 1;
  keyVariableC = true;
}
void playProgramButtonFunction() {
  //if (fastThread == null || !fastThread.isAlive()) {
  //  runFastLoop = true;

  //  fastThread = new Thread(new Runnable() {
  //    public void run() {
  //      while (runFastLoop) {
  //        if (Arm2.executeProgram(movementProgram) == 1) {
  //          runFastLoop = false;
  //        }

  //        time[3] = (double)millis()-time[2];
  //        time[2] = millis();
  //        Arm2.sendData();
  //      }
  //    }
  //  }
  //  );
  //  fastThread.start();
  //}
  keyVariableA = true;
}
void saveProgramButtonFunction() {
  String temp;
  if (cp5.get(Textfield.class, "timeTextFieldFunction").getText().isEmpty()) {
    temp = "untitled program";
    //programSelectionTextField.setText("untitled program");
  } else {
    temp = cp5.get(Textfield.class, "programSelectionTextFieldFunction").getText();
  }
  currentProgram = temp;
  saveProgramToFile(movementProgram, temp);
}
void loadProgramButtonFunction() {
  String temp;
  if (cp5.get(Textfield.class, "timeTextFieldFunction").getText().isEmpty()) {
    temp = "untitled program";
  } else {
    temp = cp5.get(Textfield.class, "programSelectionTextFieldFunction").getText();
  }
  currentProgram = temp;
  movementProgram = loadProgramFromFile(temp);
}
void moveMethodToggleButtonFunction() {
  toggleInputMethodBool = !toggleInputMethodBool;
  slider1.setVisible(toggleInputMethodBool);
  slider2.setVisible(toggleInputMethodBool);
  slider3.setVisible(toggleInputMethodBool);
  slider4.setVisible(toggleInputMethodBool);
  slider5.setVisible(toggleInputMethodBool);
  slider6.setVisible(toggleInputMethodBool);
  pxTextField.setVisible(!toggleInputMethodBool);
  pyTextField.setVisible(!toggleInputMethodBool);
  pzTextField.setVisible(!toggleInputMethodBool);
  rTextField.setVisible(!toggleInputMethodBool);
  pTextField.setVisible(!toggleInputMethodBool);
  yTextField.setVisible(!toggleInputMethodBool);
  conformXYZRPYButton.setVisible(!toggleInputMethodBool);
}

double[][] rotationMatrixFromAngles12332 = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};


void conformXYZRPYButtonFunction() {
  rotationMatrixFromAngles12332 = rotationMatrixFromAngles(Double.parseDouble(cp5.get(Textfield.class, "rTextFieldFunction").getText()), Double.parseDouble(cp5.get(Textfield.class, "pTextFieldFunction").getText()), Double.parseDouble(cp5.get(Textfield.class, "yTextFieldFunction").getText()));
  double[][] temp = rotationMatrixFromAngles12332;
  double[][] temp1 = {{temp[0][0], temp[0][1], temp[0][2], Double.parseDouble(cp5.get(Textfield.class, "pxTextFieldFunction").getText())}, {temp[1][0], temp[1][1], temp[1][2], Double.parseDouble(cp5.get(Textfield.class, "pyTextFieldFunction").getText())}, {temp[2][0], temp[2][1], temp[2][2], Double.parseDouble(cp5.get(Textfield.class, "pzTextFieldFunction").getText())}, {0, 0, 0, 1}};
  double[][][] Temp2 = {{{1000, 0, 0, 0}, temp1[0], temp1[1], temp1[2], temp1[3]}};
  globalTemp2 = Temp2;
  text("rpyxyz", 500, 500);
  keyVariableC = false;
  rotationMatrixFromAngles12332 = temp1;
  loopVariable0 = true;
}
void sendToRobotButtonFunction() {
  //double[][] temp1 = Arm1.resultMatrix.getData();
  //double[][][] temp = {{{1000, gripperVariable, vialBoxVariable, inversionVariable}, temp1[0], temp1[1], temp1[2], temp1[3]}};
  //Arm1.executeProgram(temp);
  //delay(10);
  Arm1.sendData();
}

void saveProgramToFile(double[][][] movementProgram, String programName) {

  String[] movementProgramString = new String[movementProgram.length];
  for (int i = 0; i < movementProgram.length; i++) {
    String slice = "";
    for (int j = 0; j < movementProgram[i].length; j++) {
      String row = "";
      for (int k = 0; k < movementProgram[i][j].length; k++) {
        row = row + Double.toString(movementProgram[i][j][k]);
        if (k < movementProgram[i][j].length-1) {
          row = row + "'";
        }
      }
      slice = slice + row;
      if (j < movementProgram[i].length-1) {
        slice = slice + "|";
      }
    }
    movementProgramString[i] = slice;
  }
  saveStrings(programName, movementProgramString);
}

double[][][] loadProgramFromFile(String programToLoad) {

  String[] loadedProgramSlice = loadStrings(programToLoad);
  double[][][] movementProgram = new double[loadedProgramSlice.length][][];

  for (int i = 0; i < loadedProgramSlice.length; i++) {
    String[] row = split(loadedProgramSlice[i], "|");
    movementProgram[i] = new double[row.length][];
    for (int j = 0; j < row.length; j++) {
      String[] wak = split(row[j], "'");
      movementProgram[i][j] = new double[wak.length];
      for (int k = 0; k < wak.length; k++) {
        movementProgram[i][j][k] = Double.parseDouble(wak[k]);
      }
    }
  }
  return movementProgram;
}
public double[][] rotationMatrixFromAngles(double gammaT, double betaT, double alphaT) {
  double alpha = Math.toRadians(alphaT); //alpha is rotation around z, beta around y, and gamma around x.
  double beta = Math.toRadians(betaT);
  double gamma = Math.toRadians(gammaT);
  RealMatrix matrix = new Array2DRowRealMatrix(new double[][]{
    {Math.cos(alpha)*Math.cos(beta), Math.cos(alpha)*Math.sin(beta)*Math.sin(gamma)-Math.sin(alpha)*Math.cos(gamma), Math.cos(alpha)*Math.sin(beta)*Math.cos(gamma)+Math.sin(alpha)*Math.sin(gamma)},
    {Math.sin(alpha)*Math.cos(beta), Math.sin(alpha)*Math.sin(beta)*Math.sin(gamma)+Math.cos(alpha)*Math.cos(gamma), Math.sin(alpha)*Math.sin(beta)*Math.cos(gamma)-Math.cos(alpha)*Math.sin(gamma)},
    {Math.sin(beta)*(-1), Math.cos(beta)*Math.sin(gamma), Math.cos(beta)*Math.cos(gamma)}});
  double[][] matrix1 = matrix.getData();
  return matrix1;
}
