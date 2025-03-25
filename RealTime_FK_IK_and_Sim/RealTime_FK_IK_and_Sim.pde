import controlP5.*;//------------------------Libaries--------------------------
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.geometry.euclidean.threed.*;
import processing.serial.*;

//------------------------Objects-----------(/ in some cases called data types)------
Utils WP = new Utils(); //Make instance(object) of the utils class, so we can use it's public functions.
SimStuff SS = new SimStuff(); //Same here.
UserInterface UI = new UserInterface(); //Same here.
ControlP5 cp5; //Same
Joint[] joints = new Joint[7]; //Make 7 objects for the 7 coordinate systems/joints. The joint object currently just contains a transformationmatrix corresponding to a row in DH. 0th is counted.

Serial serial;  //used to communicate with the arduino. dunno how. sourse: Søren.
Textarea receivedArea; //sourse: Søren to explain. Think it is a CP5 thing.
Println arduinoConsole;//Søren
ScrollableList portlist;
ScrollableList baudlist;

Button cntbutton, toggleUI; //Declaring button objects. They are most likely made in "class UserInterface" somewhere.
Slider slider1, slider2, slider3, slider4, slider5, slider6; //Declaring sliders, also made in UserInterface.
RealMatrix matrix01, matrix02, matrix03, matrix04, matrix05, matrix06, result; //Declaring some global variables of the "RealMatrix" type.


int posX, posY; //Declaring a kind of center position of the coordinate system. Initialized in setup.
float panX = 0, panY = 0; //Declaring and making the panning offsets. Startvalue is 0.
float rotX = -1, rotY = -0.75; //Rotation angles (in radians). Start value != 0, so the view starts at a nice angle.
float lastMouseX, lastMouseY; //Used to track where the mouse was last. Used when the mouse is dragged to pan and rotate.
boolean rightMousePressed = false; //Used to track which mouse button is the one being pressed.
boolean leftMousePressed = false;  //Used to track which mouse button is being pressed.
int menuHeight = 50;      //The height the "menu" goes down to. The "menu" is not a object at the moment.
int menuWidth;      //The menu is just some boxes where the color is different and the "mouseDragged()" function doesn't do anything. Initialized in setup.
int zoom = 800;          //Start zoom / start distande from the view.
int tempHWForWiz = 20;   //Size of the rectangles drawn at each coordinate tystem for the joints.
PShape Base, Link1, Link2; //The .obj files of the component models will be loaded in to this "PShape" data type.


boolean connectButtonStatus = false; //Status of the connect button
int uiSwNum = 1; //Status of the "toggleUI" button.
String selectedport; //Søren
int selectedbaudrate; //Søren

float theta0, theta1, theta2, theta3, theta4, theta5, theta6; //Theta values.
float lastSentValue1, lastSentValue2, lastSentValue3, lastSentValue4, lastSentValue5, lastSentValue6; //Track what values was last sent to the arduino.
double[][] MDH; //MDH parameters are declared here as global variables, but only initialized in the "updateMDH" function.

double[] cordinateOutput = {0, 0, 0, 0, 0, 0}; //Used for printing "XYZRPY" coordinates.
String[] coordinateNames = {"X", "Y", "Z", "Roll", "Pitch", "Yaw"}; //XYZRPY

boolean keyVariableA, keyVariableB; //Track key A and B


void setup() {
  size(1625, 900, P3D);           //Make the canvas/window. Size 1625x by 900y. P3D means it is a 3D "canvas"
  posX = width/2-325;             //x and y positions of the new orego of the coordinate system in terms of the window.
  posY = height;                  //Used in a "translate" function in "draw".
  menuWidth = width-375;          //Initialized here because it has to be after the "size()" function, to know what "width" is.
  cp5 = new ControlP5(this);      //Initialize cp5.
  UI.makeUI();                    //Call the function that loads / makes the UI (Buttons and sliders etc.)
  UI.showConnectionUI(false);     //Hide the connection UI again. Don't need it right now. toggleUI button will show it.
  WP.updateMDH();                 //Updating / making the MDH parameters for the first time
  Base = loadShape("Base.obj");   //Initialize the "PShape" with a .obj file.
  Link1 = loadShape("Link1.obj"); //same
  Link2 = loadShape("Link2.obj"); //same
}

void draw() {
  background(200);                                                  //Make background color 200. It goes between black 0-255 white. It is also possible to use (R,G,B) as input.
  directionalLight(126, 126, 126, 0, 0, -1);                        //Make some random ass light. Needed so we can get a perception of the depth of the PShapes.
  ambientLight(102, 102, 102);                                      //Some light shit.
  fill(200, 200, 255);                                              //Fill kinda sets a global variable that shapes use as color. In this case the next rectangle will be colored (R,G,B) (200, 200, 255).
  rect(0, 0, width, menuHeight);                                    //Make rectangle at position 0x 0y with a width of "width" and height of menuHeight.
  rect(menuWidth, menuHeight, width-menuHeight, height-menuHeight); //Look up https://processing.org/reference/ for more information about these kind of things.
  UI.sendData();                                                    //Call function to send data to the arduino.
  WP.calculateResultMatrix();                                       //Calculating a
  WP.drawMatrix(width-325, 70, cordinateOutput);                    //Drawing a matrix with "drawMatrix" function from "class Utils". The instance of "class Utils" is called "WP".
  WP.drawMatrix(width-225, 70, coordinateNames);                    //same

  if (keyVariableA == true) {
    SS.applyRealMatrix(joints[2].iMatrix);
  }
  if (keyVariableB == true) {
    WP.drawMatrix(200, 50, result);
  }

  //Under here is where the transformations from the rotate pan zoom functionality happens.
  pushMatrix();                                //Look up the reference sheet "processing.org/reference". It is like making a quicksave before making changes.
  translate(posX + panX, posY + panY, -zoom);  //Translate will move the coordinate system in XYZ. See reference sheet.
  rotateX(-rotX);                              //Self explanatory.
  rotateZ(rotY);
  rectMode(CENTER);
  noStroke();                    //https://processing.org/reference/
  fill(255);
  rect(0, 0, 1000, 1000);      //Ground/talbe/build area/white plate/motherfuga
  rectMode(CORNER);

  //After the rotate pan zoom transformations have taken place, we draw the joint coordinate systems.
  //We draw the first joint, apply the transformation matrix belonging to the next joint, and then draw the next joint.
  fill(255, 0, 0); //red
  rect(0, 0, tempHWForWiz, tempHWForWiz);
  SS.coordSystem(); //A coordinate system. Base if you will.

  SS.applyRealMatrix(joints[0].iMatrix); //Apply transformation matrix.
  SS.coordSystem(); //Draw 0th coordinate system.
  fill(0, 255, 0); //change color
  rect(0, 0, tempHWForWiz, tempHWForWiz); //plane between X and Y. Just to see the orientation of the coordinate system easier.
  pushMatrix(); //The origins of the .obj files does not line up with the origins of the joint.
  rotateX(3.14159265/2); //This fixes that offset for this particular .obj.
  shape(Base);  //base.obj
  popMatrix(); //Go back to before the "fix .obj" transformations. so we can keep drawing from that reference frame.

  SS.applyRealMatrix(joints[1].iMatrix); //apply matrix
  SS.coordSystem(); //1st coordinate system.
  fill(0, 0, 255);
  rect(0, 0, tempHWForWiz, tempHWForWiz);
  pushMatrix(); //Fix move .obj to line up.
  rotateX(3.14159265/2);
  rotateY(-3.14159265/2);
  shape(Link1);
  popMatrix();

  SS.applyRealMatrix(joints[2].iMatrix);
  SS.coordSystem();
  fill(255, 255, 0);
  rect(0, 0, tempHWForWiz, tempHWForWiz);
  pushMatrix();
  rotateY(3.14159265/2);
  rotateX(3.14159265/2);
  translate(0, 0, 0);
  shape(Link2);
  popMatrix();

  SS.applyRealMatrix(joints[3].iMatrix);
  SS.coordSystem();
  fill(255, 0, 255);
  rect(0, 0, tempHWForWiz, tempHWForWiz);
  pushMatrix();
  rotateY(3.14159265/2);
  shape(Link2);
  popMatrix();

  SS.applyRealMatrix(joints[4].iMatrix);
  SS.coordSystem();
  fill(0, 255, 255);
  rect(0, 0, tempHWForWiz, tempHWForWiz);
  pushMatrix();
  rotateX(3.14159265/2);
  rotateY(3.14159265/2);
  translate(0, -115.49, 0);
  shape(Link2);
  popMatrix();

  SS.applyRealMatrix(joints[5].iMatrix);
  SS.coordSystem();
  fill(255, 125, 125);
  rect(0, 0, tempHWForWiz, tempHWForWiz);

  SS.applyRealMatrix(joints[6].iMatrix);
  SS.coordSystem();
  fill(125, 125, 255);
  rect(0, 0, tempHWForWiz, tempHWForWiz);
  pushMatrix();
  rotateX(3.14159265/2);
  rotateY(3.14159265/2);
  shape(Link2);
  popMatrix();
  
  popMatrix();
}


//---------------------Key and Mouse press functions----------------------------------------------
//Filled with functions that built-in and specific to processing.

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
  float e = event.getCount();               //"event.getCount()" detects scroll direction. Returns "1" or "-1".
  zoom += e * 40;                           //Zoom is also just used in a "translate()" in "draw()". Just on the Z axis (towards and away from view), instead of XY.
  zoom = constrain(zoom, -350, 3000);       //Set zoom limits
  panX += (mouseX-(width/2))*0.1*e;         //This is just a little extra to make the camera zoom kinda towards the mouse position.
  panY += (mouseY-(height/2))*0.1*e;        //It is not accurate, it is just better that nothing.
}



//---------------------Some button stuff-----------------------------------
void toggleUI() { //Will toggle the UI. Runs when "toggleUI" button is pressed.
  if (uiSwNum == 1) {
    UI.showConnectionUI(true);
    uiSwNum = 0;
  } else {
    UI.showConnectionUI(false);
    uiSwNum = 1;
  }

  //switch(uiSwNum) {
  //case 0:
  //  UI.showConnectionUI(true);
  //  UI.showThetaSlidersUI(true);
  //  break;
  //case 1:
  //  UI.showConnectionUI(true);
  //  UI.showThetaSlidersUI(false);
  //  break;
  //case 2:
  //  UI.showConnectionUI(false);
  //  UI.showThetaSlidersUI(true);
  //  break;
  //default:
  //  uiSwNum = 0;
  //  toggleUI();
  //  uiSwNum = 0;
  //}
  //uiSwNum+=1;
}



//----------Stuff for the Arduino Connection Stuff-----------------------

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
