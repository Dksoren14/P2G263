class UserInterface {

  public void makeUI() { //Constructor. Creates all the sliders
    thetaSliders(width-325, 300, 50); //Function.
    connectionUI(25, 50);             //Function.
    toggleUI = cp5.addButton("toggleUI") //Make button "toggleUI".
      .setLabel("Toggle UI")
      .setSize(100, 30)
      .setPosition(0, 0);
  }

  public void showConnectionUI(boolean uiVisible) { //Function that sets visibility of the buttons, menus and other UI elements that are made in the "connectionUI" function.
    cntbutton.setVisible(uiVisible);
    portlist.setVisible(uiVisible);
    baudlist.setVisible(uiVisible);
    receivedArea.setVisible(uiVisible);
  }
  public void connectionUI(int x, int y) {

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

    String[] availableports = Serial.list();
    for (int i = 0; i < availableports.length; i++) {
      portlist.addItem(availableports[i], availableports[i]);
    }
  }

  public void showThetaSlidersUI(boolean uiVisible) { //Function that sets visibility of the sliders that are made in the "thetaSliders" function.
    slider1.setVisible(uiVisible);
    slider2.setVisible(uiVisible);
    slider3.setVisible(uiVisible);
    slider4.setVisible(uiVisible);
    slider5.setVisible(uiVisible);
    slider6.setVisible(uiVisible);
  }
  public void thetaSliders(int x, int y, int ya) {
    slider1 = cp5.addSlider("theta1")
      .setPosition(x, y)
      .setSize(200, 20)
      .setRange(-180, 180)
      .setValue(0)
      .setColorCaptionLabel(color(20, 20, 20));
    slider2 = cp5.addSlider("theta2")
      .setPosition(x, y + ya)
      .setSize(200, 20)
      .setRange(-180, 180)
      .setValue(0)
      .setColorCaptionLabel(color(20, 20, 20));
    slider3 = cp5.addSlider("theta3")
      .setPosition(x, y + ya * 2)
      .setSize(200, 20)
      .setRange(-180, 180)
      .setValue(0)
      .setColorCaptionLabel(color(20, 20, 20));
    slider4 = cp5.addSlider("theta4")
      .setPosition(x, y + ya * 3)
      .setSize(200, 20)
      .setRange(-180, 180)
      .setValue(0)
      .setColorCaptionLabel(color(20, 20, 20));
    slider5 = cp5.addSlider("theta5")
      .setPosition(x, y + ya * 4)
      .setSize(200, 20)
      .setRange(-180, 180)
      .setValue(0)
      .setColorCaptionLabel(color(20, 20, 20));
    slider6 = cp5.addSlider("theta6")
      .setPosition(x, y + ya * 5)
      .setSize(200, 20)
      .setRange(-180, 180)
      .setValue(0)
      .setColorCaptionLabel(color(20, 20, 20));
  }

  public void sendData() { //Sends some data to arduino. Søren write more comments.
    try {
      if (theta1 != lastSentValue1 || theta2 != lastSentValue2 || theta3 != lastSentValue3 || theta4 != lastSentValue4 || theta5 != lastSentValue5 || theta6 != lastSentValue6) {
        String message = "M1:" + (int) theta1 + "M1end| M2:" + (int) theta2 + "M2end| M3:" + (int) theta3 + "M3end| M4:" + (int) theta4 + "M4end| M5:" + (int) theta5 + "M5end| M6:" + (int) theta6 + "\n";
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
}
