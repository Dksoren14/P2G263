import controlP5.*;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.geometry.euclidean.threed.*;
import processing.serial.*;


ControlP5 cp5;
Utils utils = new Utils();
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


  slider1 = cp5.addSlider("theta1")
    .setPosition(150, 100)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
  slider2 = cp5.addSlider("theta2")
    .setPosition(150, 150)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
  slider3 = cp5.addSlider("theta3")
    .setPosition(150, 200)
    .setSize(200, 20)
    .setRange(-180, 180)
    .setValue(0)
    .setColorCaptionLabel(color(20, 20, 20));
}

void draw() {
  background(125, 125, 250);
  utils.drawResult(theta, 500, 150);
  utils.drawResult(MDH, 650, 150);
}
