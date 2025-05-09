import controlP5.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.geometry.euclidean.threed.*;
import processing.serial.*;


Serial serial;  //used to communicate with the arduino. dunno how. sourse: Søren.
Textarea receivedArea; //sourse: Søren to explain. Think it is a CP5 thing.
Println arduinoConsole;//Søren
ScrollableList portlist;
ScrollableList baudlist;
float[] lastSentValue = new float[6]; //Track what values was last sent to the arduino.
boolean connectButtonStatus = false; //Status of the connect button
String selectedport; //Søren
int selectedbaudrate; //Søren

Button connectionButton, toggleConnectionUIButton, infoButton, saveProgramPointButton, leftArrowButton, rightArrowButton, addPointButton, saveProgramButton, editProgramLocationButton, playProgramButton, toggleSaveLoadUIButton;
Textfield programSelectionTextField, timeTextField;
boolean toggleUIBool = false; //Status of the "toggleUI" button.
boolean toggleSaveLoadUIBool = true;
boolean infoButtonVariable = false;
boolean keyVariableA, keyVariableB, keyVariable1, keyVariable2, keyVariable3, keyVariable4, keyVariable5, keyVariable6, keyVariable7, keyVariable8; //Track key A and B
boolean keyVariableC = true;
boolean keyVariableD = false;
boolean keyVariableE = false;
boolean keyVariableF = false;
int tempVariableForF = 0;
//String globalTextVariable = "";
int movementNumber = 0;
int addingPoint = 0;

int posX, posY; //Declaring a kind of center position of the coordinate system. Initialized in setup.
float panX = 0, panY = 0; //Declaring and making the panning offsets. Startvalue is 0.
float rotX = -1, rotY = -0.75; //Rotation angles (in radians). Start value != 0, so the view starts at a nice angle.
float lastMouseX, lastMouseY; //Used to track where the mouse was last. Used when the mouse is dragged to pan and rotate.
boolean rightMousePressed = false; //Used to track which mouse button is the one being pressed.
boolean leftMousePressed = false;  //Used to track which mouse button is being pressed.
int menuHeight = 50;      //The height the "menu" goes down to. The "menu" is not a object at the moment.
int menuWidth;      //The menu is just some boxes where the color is different and the "mouseDragged()" function doesn't do anything. Initialized in setup.
int zoom = 800;          //Start zoom / start distande from the view.


float saveThetaValues[][] = new float[4][6];


ControlP5 cp5;
Utils utils = new Utils();
Arm Arm1, Arm2;

Slider slider1, slider2, slider3, slider4, slider5, slider6;
float theta1, theta2, theta3, theta4, theta5, theta6; //Theta values
float[] theta = {0, 0, 0, 0, 0, 0};
//float[] startTheta;
double[] time = {millis(), 0};

double[][] MDH = { //Alpha, a, d, theta offset
  {0, 0, 122.65, 0},
  {90, 39.43, 0, 90},
  {0, 115.49, 0, 0},
  {90, 0, 115.49*2, 0},
  {-90, 0, 0, 0},
  {90, 0, 0, 0}};

double[][][] movementProgram = {{
    {1000, 0, 0, 0}, //Time, nothing, nothing, nothing
    {0, -0.9178, 0.3971, 107}, //r, r, r, Px
    {0, -0.3971, -0.9178, -248}, //r, r, r, Py
    {1, 0, 0, 238}, //r, r, r, Pz
    {0, 0, 0, 1}                  //0, 0, 0, 1
  }, {
    {1000, 0, 0, 0},
    {0, -0.9178, 0.3971, 107},
    {0, -0.3971, -0.9178, -248},
    {1, 0, 0, 150},
    {0, 0, 0, 1}
  }, {
    {1000, 0, 0, 0},
    {0, 0, 1, 270.41},
    {0, -1, 0, 0},
    {1, 0, 0, 238.14},
    {0, 0, 0, 1}
}};

double[][][] movementProgram1232;

int switchProgramVariable = 0;

RealMatrix Matrix1232 = new Array2DRowRealMatrix(new double[][] {{15, 20, 30, 40}, {1, 2, 3.5, 4.1}, {10, 29, 30, 40}, {1, 2, 3.2, 4}});
RealMatrix Matrix1233 = new Array2DRowRealMatrix(new double[][] {{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}, {13, 14, 15, 16}});


PShape[] textures = new PShape[7];  //The .obj files of the component models will be loaded in to this "PShape" data type.


void setup() {
  size(1625, 900, P3D);           //Make the canvas/window. Size 1625x by 900y. P3D means it is a 3D "canvas"
  posX = width/2-325;             //x and y positions of the new orego of the coordinate system in terms of the window.
  posY = height-100;                  //Used in a "translate" function in "draw".
  menuWidth = width-375;

  //textures[0] = loadShape("obj_files/Base.obj");
  //textures[1] = loadShape("obj_files/Link1.obj");
  //textures[2] = loadShape("obj_files/Link2.obj");

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
  
}

void draw() {
  background(200);                                                  //Make background color 200. It goes between black 0-255 white. It is also possible to use (R,G,B) as input.
  directionalLight(126, 126, 126, 0, 0, -1);                        //Make some random ass light. Needed so we can get a perception of the depth of the PShapes.
  ambientLight(102, 102, 102);                                      //Some light shit.

  fill(200, 200, 255);                                              //Fill kinda sets a global variable that shapes use as color. In this case the next rectangle will be colored (R,G,B) (200, 200, 255).
  rect(0, 0, width, menuHeight);                                    //Make rectangle at position 0x 0y with a width of "width" and height of menuHeight.
  rect(menuWidth, menuHeight, width-menuHeight, height-menuHeight); //Look up https://processing.org/reference/ for more information about these kind of things.



  if (infoButtonVariable) {
    utils.drawResult("Slider angles", 30, 70);
    utils.drawResult(theta, 30, 120); //Draw slider values
    utils.drawResult("Arm1 result matrix with slider angles (T06)", 175, 100);
    utils.drawResult(Arm1.resultMatrix, 175, 150);
    utils.drawResult("IK angles", 800, 70);
    utils.drawResult(Arm1.anglesFromIK(Arm1.resultMatrix.getData()), 800, 120);
    utils.drawResult("Arm1 result matrix with IK angles (T06)", 1000, 100);
    utils.drawResult(Arm2.resultMatrix, 1000, 150);
    utils.drawResult(Arm2.speed, 900, 450);
    utils.drawResult(Arm2.pos, 1050, 450);
    utils.drawResult(Arm2.acc, 1200, 450);
  }

  checkKeyPressed();


  //Under this is where the transformations from the rotate pan zoom functionality happens.
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
  translate(-300, 0);
  Arm1.draw(theta);
  popMatrix();


  pushMatrix();
  translate(100, 0);
  Arm2.draw();
  popMatrix();


  popMatrix();
  if (toggleSaveLoadUIBool) {
    drawSaveLoadUI(width-325, 70);
  }
  sendData();
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
  //showConnectionUI(toggleUIBool);
  connectionButton.setVisible(toggleUIBool);
  portlist.setVisible(toggleUIBool);
  baudlist.setVisible(toggleUIBool);
  receivedArea.setVisible(toggleUIBool);
}



public void sendData() { //Sends some data to arduino. Søren write more comments.
  //try {
  //  if (theta[0] != lastSentValue[0] || theta[1] != lastSentValue[1] || theta[2] != lastSentValue[2] || theta[3] != lastSentValue[3] || theta[4] != lastSentValue[4] || theta[5] != lastSentValue[5]) {
  //    String message = "M1:" + theta[0] + "M1end| M2:" + theta[1] + "M2end| M3:" + theta[2] + "M3end| M4:" + theta[3] + "M4end| M5:" + theta[4] + "M5end| M6:" + theta[5] + "M6end| S1" + speed[1] "\n";
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

//public void readData() {
//}

//void baudratelistFunction(int index) {
//  String baudstring;
//  baudstring = baudlist.getItem(index).get("name").toString();
//  selectedbaudrate = Integer.parseInt(baudstring);
//  println("Selected", selectedbaudrate);
//}
//void comportlistFunction(int index) {
//  selectedport = portlist.getItem(index).get("name").toString();
//  println("Selected", selectedport);
//}
//void connectButtonFunction() {
//  if (!connectButtonStatus) {
//    serial = new Serial(this, selectedport, selectedbaudrate);
//    connectionButton.setLabel("Disconnect");
//    connectButtonStatus = true;
//    println("Connected", selectedport, "at", selectedbaudrate);
//  } else {
//    serial.stop();
//    connectionButton.setLabel("Connect");
//    connectButtonStatus = false;
//    println("Disconnected from", selectedport);
//  }
//}


void connectionUI(int x, int y) { //Function that creates the connection UI
  //toggleConnectionUIButton = cp5.addButton("toggleConnectionUI") //Make button "toggleUI".
  //  .setLabel("Connection UI")
  //  .setSize(100, 30)
  //  .setPosition(x, y);
  //y=y+50;
  //connectionButton = cp5.addButton("connectButtonFunction")
  //  .setLabel("Connect")
  //  .setSize(70, 30)
  //  .setPosition(x, y);

  //portlist = cp5.addScrollableList("comportlistFunction")
  //  .setLabel("select port")
  //  .setBarHeight(30)
  //  .setPosition(x+100, y)
  //  .setItemHeight(25);

  //baudlist = cp5.addScrollableList("baudratelistFunction")
  //  .setLabel("select baudrate")
  //  .setBarHeight(30)
  //  .setPosition(x+220, y)
  //  .setItemHeight(24);

  //baudlist.addItem("9600", 9600);
  //baudlist.addItem("19200", 19200);
  //baudlist.addItem("38400", 38400);
  //baudlist.addItem("57600", 57600);

  //receivedArea = cp5.addTextarea("receivedData")
  //  .setSize(360, 140)
  //  .setPosition(x, y+250)
  //  .setColorBackground(80);
  //arduinoConsole = cp5.addConsole(receivedArea);

  //String[] availableports = Serial.list(); //   <-------------------- Søren explain plz
  //for (int i = 0; i < availableports.length; i++) {
  //  portlist.addItem(availableports[i], availableports[i]);
  //}

  //connectionButton.setVisible(false);
  //portlist.setVisible(false);
  //baudlist.setVisible(false);
  //receivedArea.setVisible(false);
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
    keyVariableA = true;    //This variable is (at the time of writing this) being used for drawing something. It is therefore made like a flip-flop, to draw it every frame and not just once.
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
    keyVariableF = true;
    tempVariableForF += 1;
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

void checkKeyPressed() { //----------------------------------------------------------------------------------------
  if (keyVariableA) { //Movement program
    //switch(switchProgramVariable) {
    //case 0:
    //  switchProgramVariable += Arm2.executeMovement(movementProgram[1], 1000);
    //  break;
    //case 1:
    //  switchProgramVariable += Arm2.executeMovement(movementProgram[2], 1000);
    //  break;
    //case 2:
    //  switchProgramVariable += Arm2.executeMovement(movementProgram[0], 1000);
    //  break;
    //case 3:
    //  switchProgramVariable = 0;
    //}
    if (Arm2.executeProgram(movementProgram) == 1) {
      keyVariableA = false;
    }
  }

  if (keyVariableB) {
    //movementProgram = Arm1.savePointToProgram(movementProgram, 1000);
    keyVariableB = false;
  }
  if (keyVariableD) {
    saveProgramToFile(movementProgram, "movementProgram1");
    keyVariableD = false;
  }
  if (keyVariableE) {
    movementProgram1232 = loadProgramFromFile("movementProgram1");
    keyVariableE = false;
  }
  if (keyVariableF) {
    utils.drawResult(movementProgram1232[tempVariableForF], 10, 400);
    //time[1] = (double)millis()-time[0];
    //time[0] = millis();
    //utils.drawResult(time, 10, 100);
  }

  if (keyVariable1) {
    saveThetaValues(0);
    keyVariable1 = false;
  }
  if (keyVariable2) {
    saveThetaValues(1);
    keyVariable2 = false;
  }
  if (keyVariable3) {
    saveThetaValues(2);
    keyVariable3 = false;
  }
  if (keyVariable4) {
    saveThetaValues(3);
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
      playSavedThetaValues(0);
    }
    if (keyVariable6) {
      playSavedThetaValues(1);
    }
    if (keyVariable7) {
      playSavedThetaValues(2);
    }
    if (keyVariable8) {
      playSavedThetaValues(3);
    }
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
  editProgramLocationButton.setVisible(toggleSaveLoadUIBool);
  timeTextField.setVisible(toggleSaveLoadUIBool);
}

void drawSaveLoadUI(int x, int y) {
  fill(0);
  textSize(30);
  text("Point " + (movementNumber + addingPoint), x + 5, y + 70);
}

void saveLoadUI(int x, int y) {
  toggleSaveLoadUIButton = cp5.addButton("toggleSaveLoadUI") //Make button "toggleUI".
    .setLabel("Save/Load")
    .setSize(100, 30)
    .setPosition(240, 10);
  saveProgramPointButton = cp5.addButton("saveProgramPointButtonFunction")
    .setLabel("Save Point")
    .setSize(60, 30)
    .setPosition(x, y + 130);
  leftArrowButton = cp5.addButton("leftArrowButtonFunction")
    .setLabel("<---")
    .setSize(60, 30)
    .setPosition(x, y + 90);
  rightArrowButton = cp5.addButton("rightArrowButtonFunction")
    .setLabel("--->")
    .setSize(60, 30)
    .setPosition(x + 140, y + 90);
  programSelectionTextField = cp5.addTextfield("programSelectionTextFieldFunction")
    .setLabel("")
    .setPosition(x, y)
    .setSize(200, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("untitled program");
  editProgramLocationButton = cp5.addButton("editProgramLocationButtonFunction")
    .setLabel("Edit Point")
    .setSize(60, 30)
    .setPosition(x + 70, y + 130);
  saveProgramButton = cp5.addButton("saveProgramButtonFunction")
    .setLabel("Save Program")
    .setSize(60, 30)
    .setPosition(x + 140, y + 40);
  addPointButton = cp5.addButton("addPointButtonFunction")
    .setLabel("Add Point")
    .setSize(60, 30)
    .setPosition(x + 140, y + 130);
  timeTextField = cp5.addTextfield("timeTextFieldFunction")
    .setLabel("")
    .setPosition(x, y+170)
    .setSize(60, 30)
    .setFocus(true)
    .setColor(color(255))
    .setAutoClear(false)
    .setText("1234");
  playProgramButton = cp5.addButton("playProgramButtonFunction")
    .setLabel("Play")
    .setSize(60, 30)
    .setPosition(x + 70, y + 90);
}

void leftArrowButtonFunction() {
  keyVariableC = false;
  addingPoint = 0;
  movementNumber -= 1;
  movementNumber = constrain(movementNumber, 1, movementProgram.length);
  double[][] temp = {movementProgram[movementNumber-1][1], movementProgram[movementNumber-1][2], movementProgram[movementNumber-1][3], movementProgram[movementNumber-1][4]};
  //Arm1.executeMovement(temp, 1);
  double[] temp2 = Arm1.anglesFromIK(temp);
  for (int i = 0; i < temp2.length; i++) {
    theta[i] = (float)temp2[i];
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
    for (int i = 0; i < temp2.length; i++) {
      theta[i] = (float)temp2[i];
    }
  }
}
void saveProgramPointButtonFunction() {
  int temp;
  if (cp5.get(Textfield.class, "timeTextFieldFunction").getText().isEmpty()) {
    temp = 1234;
  } else {
    temp = Integer.parseInt(cp5.get(Textfield.class, "timeTextFieldFunction").getText());
  }
  movementProgram = Arm1.savePointToProgram(movementProgram, temp, movementNumber-1+addingPoint);
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
  keyVariableA = true;
}
void saveProgramButtonFunction() {
  String temp;
  if (cp5.get(Textfield.class, "timeTextFieldFunction").getText().isEmpty()) {
    temp = "untitled program";
  } else {
    temp = cp5.get(Textfield.class, "programSelectionTextFieldFunction").getText();
  }
  saveProgramToFile(movementProgram, temp);
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
