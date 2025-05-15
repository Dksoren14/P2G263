class Arm {

  RealMatrix resultMatrix;
  Joint[] jointArray;
  double[][] MDH;
  double startTime;
  double currentTime;
  double[] targetAngle = {0, 0, 0, 0, 0, 0};
  double[] startAngle = new double[6];
  double[] speed = new double[6];
  double[] pos = new double[6];
  double[] acc = new double[6];
  boolean executingMovement = false;
  boolean executingProgram = false;
  double[] startPos = {0, 0, 0, 0, 0, 0}; //Tracks the current start position
  int trackMovement = 0; //Tracks what movement the program is currentrly executing.
  double lasMillis = 0;
  boolean someBoolValue = true;
  String[] messageArrayOut = {"a", "b", "c", "d"};
  String[] messageArrayIn = {"a", "b", "c", "d"};
  int dataNumber = 0;
  double[] lastSendTime;

  Arm(double[][] MDHT) {
    MDH = MDHT;
    jointArray = new Joint[MDHT.length];

    for (int i = 0; i < MDHT.length; i++) {
      jointArray[i] = new Joint(MDHT[i]);
    }
    calculateFinalMatrix();
  }
  Arm(double[][] MDHT, PShape[] textures) {
    MDH = MDHT;
    jointArray = new Joint[MDHT.length];

    for (int i = 0; i < MDHT.length; i++) {
      jointArray[i] = new Joint(MDHT[i]);
      if (textures[i] != null) {
        jointArray[i].texture = textures[i+1];
      }
    }
    calculateFinalMatrix();
  }

  void draw(float[] a) {
    if (textures[0] != null) {
      scale(1, -1, 1);
      shape(textures[0]);
      scale(1, -1, 1);
    }
    for (int i = 0; i < jointArray.length; i++) {
      jointArray[i].updateTransformationMatrix(Math.toRadians(a[i]));
      utils.applyRealMatrix(jointArray[i].realTransformationMatrix);
      jointArray[i].draw();
    }
    calculateFinalMatrix();
  }

  void draw(double[] a) {
    if (textures[0] != null) {
      scale(1, -1, 1);
      shape(textures[0]);
      scale(1, -1, 1);
    }
    for (int i = 0; i < jointArray.length; i++) {
      jointArray[i].updateTransformationMatrix(Math.toRadians(a[i]));
      utils.applyRealMatrix(jointArray[i].realTransformationMatrix);
      jointArray[i].draw();
    }
    calculateFinalMatrix();
  }

  void draw() {
    if (textures[0] != null) {
      scale(1, -1, 1);
      shape(textures[0]);
      scale(1, -1, 1);
    }
    for (int i = 0; i < jointArray.length; i++) {
      jointArray[i].updateTransformationMatrix(Math.toRadians(pos[i]));
      utils.applyRealMatrix(jointArray[i].realTransformationMatrix);
      jointArray[i].draw();
    }
    calculateFinalMatrix();
  }


  void calculateFinalMatrix() {

    resultMatrix = jointArray[0].realTransformationMatrix;
    for (int i=1; i<jointArray.length; i++) {
      resultMatrix = resultMatrix.multiply(jointArray[i].realTransformationMatrix);
    }
  }

  int executeMovement(double[][] targetMatrix, double targetTime) {
    int a = 0;
    if (!executingMovement) {
      for (int i = 0; i < startPos.length; i++) {
        startAngle[i] = startPos[i];
      }
      startTime = millis();
      currentTime = millis()-startTime;
      targetAngle = anglesFromIK(targetMatrix);
      executingMovement = true;
    }

    if (currentTime < targetTime) {
      currentTime = millis()-startTime;
      for (int i = 0; i < 6; i++) {
        speed[i] = abs((float)getSpeed(targetAngle[i], targetTime, startAngle[i], currentTime));
        pos[i] = getPos(targetAngle[i], targetTime, startAngle[i], currentTime);
        acc[i] = getAcc(targetAngle[i], targetTime, startAngle[i], currentTime);
      }
    }
    if (millis() > startTime+targetTime) {
      executingMovement = false;
      a = 1;
      for (int i = 0; i < startPos.length; i++) {
        startPos[i] = pos[i];
      }
    }
    return a;
  }

  int executeProgram(double[][][] movementProgram) {
    int temp = 0;
    if (!executingProgram) {
      executingProgram = true;
    }

    if (executingProgram) {
      int i = trackMovement;
      double[][] targetMatrix = {movementProgram[i][1], movementProgram[i][2], movementProgram[i][3], movementProgram[i][4]};
      trackMovement += executeMovement(targetMatrix, movementProgram[i][0][0]);
    }
    if (trackMovement > movementProgram.length - 1) {
      trackMovement = 0;
      executingProgram = false;
      temp = 1;
    }
    return temp;
  }

  double[][][] savePointToProgram(double[][][] program, int time, int pointNumber) {
    double[][] temp = resultMatrix.getData();
    double[][] targetMatrix = {{time, 0, 0, 0}, temp[0], temp[1], temp[2], temp[3]};
    if (time == 0) {
      time = 1000;
    }

    if (pointNumber <= program.length-1 && pointNumber >= 0) {
      program[pointNumber] = targetMatrix;
    } else {
      program = (double[][][])append(program, targetMatrix);
    }
    return program;
  }

  double[] anglesFromIK(double[][] targetMatrix) {
    double[] angle = new double[6];

    angle[0] = Math.atan2(targetMatrix[1][3], targetMatrix[0][3]);
    double a = Math.sqrt(Math.pow(targetMatrix[0][3], 2)+Math.pow(targetMatrix[1][3], 2))-MDH[1][1]; //(inputMatrix[0][3]/Math.cos(angle[1]))-MDH[0][2]
    double b = targetMatrix[2][3]-MDH[0][2];
    double c = Math.sqrt(a*a+b*b);
    angle[1] = Math.acos((Math.pow(MDH[2][1], 2)+Math.pow(c, 2)-Math.pow(MDH[3][2], 2))/(2*MDH[2][1]*c))+Math.atan2(b, a)-Math.toRadians(90);

    angle[2] = Math.acos((Math.pow(MDH[2][1], 2)+Math.pow(MDH[3][2], 2)-(c*c))/(2*MDH[2][1]*MDH[3][2]))-Math.toRadians(90);

    jointArray[0].updateTransformationMatrix(angle[0]);
    RealMatrix Matrix03FromIK = jointArray[0].realTransformationMatrix;
    jointArray[1].updateTransformationMatrix(angle[1]);
    Matrix03FromIK = Matrix03FromIK.multiply(jointArray[1].realTransformationMatrix);
    jointArray[2].updateTransformationMatrix(angle[2]);
    Matrix03FromIK = Matrix03FromIK.multiply(jointArray[2].realTransformationMatrix);

    RealMatrix Matrix06 = new Array2DRowRealMatrix(targetMatrix);
    RealMatrix Matrix30 = new LUDecomposition(Matrix03FromIK).getSolver().getInverse();
    RealMatrix Matrix36 = Matrix30.multiply(Matrix06);
    double[][] inputMatrix36 = Matrix36.getData();

    double d = Math.sqrt(Math.pow(inputMatrix36[0][2], 2)+Math.pow(inputMatrix36[2][2], 2));

    angle[4] = Math.atan2(d, -inputMatrix36[1][2]);
    //angle[4] = Math.acos(-inputMatrix36[1][2]);
    angle[3] = Math.atan2(inputMatrix36[2][2], inputMatrix36[0][2]);
    angle[5] = Math.atan2(-inputMatrix36[1][1], inputMatrix36[1][0]);

    double[] angleDegrees = new double[angle.length];
    for (int i = 0; i<angle.length; i++) {
      angleDegrees[i] = Math.toDegrees(angle[i]);
    }
    return angleDegrees;
  }

  double getSpeed(double endAngle, double targetTimeT, double startAngle, double currentTimeT) {
    double targetTime = targetTimeT/1000;
    double currentTime = currentTimeT/1000;

    double a_0 = startAngle;
    double a_1 = 0;
    double a_2 = (3/Math.pow(targetTime, 2)*(endAngle - startAngle));
    double a_3 = (-2/Math.pow(targetTime, 3)*(endAngle - startAngle));

    double speed = a_1 + 2*a_2*currentTime + 3*a_3*(Math.pow(currentTime, 2));
    return speed;
  }

  double getPos(double endAngle, double targetTimeT, double startAngle, double currentTimeT) {
    double targetTime = targetTimeT/1000;
    double currentTime = currentTimeT/1000;

    double a_0 = startAngle;
    double a_1 = 0;
    double a_2 = (3/Math.pow(targetTime, 2)*(endAngle - startAngle));
    double a_3 = (-2/Math.pow(targetTime, 3)*(endAngle - startAngle));

    double pos = a_0 + a_1 * currentTime + a_2*Math.pow(currentTime, 2) + a_3*(Math.pow(currentTime, 3));
    return pos;
  }

  double getAcc(double endAngle, double targetTimeT, double startAngle, double currentTimeT) {
    double targetTime = targetTimeT/1000;
    double currentTime = currentTimeT/1000;

    double a_0 = startAngle;
    double a_1 = 0;
    double a_2 = (3/Math.pow(targetTime, 2)*(endAngle - startAngle));
    double a_3 = (-2/Math.pow(targetTime, 3)*(endAngle - startAngle));

    double acc = 2*a_2 + 6*a_3*currentTime;
    return acc;
  }

  public void sendData() {
    try {
      if ((float)targetAngle[0] != lastSentValue[0] || (float)targetAngle[1] != lastSentValue[1] || (float)targetAngle[2] != lastSentValue[2] || (float)targetAngle[3] != lastSentValue[3] || (float)targetAngle[4] != lastSentValue[4] || (float)targetAngle[5] != lastSentValue[5] || (float)speed[0] != lastSentValue[6] || (float)speed[1] != lastSentValue[7] || (float)speed[2] != lastSentValue[8] || (float)speed[3] != lastSentValue[9] || (float)speed[4] != lastSentValue[10] || (float)speed[5] != lastSentValue[11] && someBoolValue) {
        String message = "" ; // "M1" + Float.toString((float)targetAngle[0]) + "1MM2" + Float.toString((float)targetAngle[1]) + "2MM3" + Float.toString((float)targetAngle[2]) + "3MM4" + Float.toString((float)targetAngle[3]) + "4MM5" + Float.toString((float)targetAngle[4]) + "5MM6" + Float.toString((float)targetAngle[5]) + "6MS1" + Float.toString((float)speed[0]) + "1SS2" + Float.toString((float)speed[1]) + "2SS3" + Float.toString((float)speed[2]) + "3SS4" + Float.toString((float)speed[3]) + "4SS5" + Float.toString((float)speed[4]) + "5SS6" + Float.toString((float)speed[5]) + "6S\n";

        for (int i = 0; i < 6; i++) {
          String angle_value = Integer.toString(round(((float) targetAngle[i]+180)*10));
          message = message+angle_value;
          message = message + ",";
        }
        for (int i = 0; i < 6; i++) {
          String speed_value = Integer.toString(round(((float) speed[i]+180)*10));
          message = message + speed_value;
          if (i<5) message = message + ",";
        }

        //utils.drawResult(message, 10, 500);

        serial.write(message);
        serial.write(10);
        serial1.write(message);
        serial1.write(10);
        messageArrayOut = append(messageArrayOut, message);

        for (int i = 0; i < lastSentValue.length; i++) {
          if (i < 6) {
            lastSentValue[i] = (float)targetAngle[i];
          } else {
            lastSentValue[i] = (float)speed[i-6];
          }
        }
      }

      //String data = serial.readStringUntil(10);
      //if (data != null) {
      //  receivedArea.setText("Arduino: " + data);

      //  messageArrayIn = append(messageArrayIn, data);
      //}
      String data = "";
      while (serial.available() > 0) {
        data = serial.readStringUntil(10);
        if (data != null) {
          receivedArea.setText("Arduino: " + data);
          messageArrayIn = append(messageArrayIn, data);
          //utils.drawResult(data, 10, 600);
        }
      }
      String data1 = "";
      while (serial1.available() > 0) {
        data1 = serial1.readStringUntil(10);
        if (data1 != null) {
          receivedArea1.setText("Arduino1: " + data1);
          messageArrayIn = append(messageArrayIn, data1);
          //utils.drawResult(data1, 10, 700);
        }
      }

      //String data = " ";
      //while (serial.available()>0) {
      //  data = data + serial.readString(); //StringUntil('\n');
      //}
      ////String data = " ";
      //utils.drawResult(data, 10, 600);

      //if (data != null) {
      //  data = data.trim();
      //  //data = "t" + millis() + "d" + data;
      //  receivedArea.setText("Arduino: " + data);

      //  messageArrayIn = append(messageArrayIn, data);
      //}

      //someBoolValue = !someBoolValue;
    }
    catch (Exception e) {
      //messageArrayIn = append(messageArrayIn, e.getMessage());

      println("Serial port error: " + e.getMessage());
    }
    //return message;
  }

  //public void armData() { //Sends some data to arduino. Søren write more comments.
  //  try {
  //    if (theta[0] != lastSentValue[0] || theta[1] != lastSentValue[1] || theta[2] != lastSentValue[2] || theta[3] != lastSentValue[3] || theta[4] != lastSentValue[4] || theta[5] != lastSentValue[5]) {
  //      String message = "M1:" + theta[0] + "M1end| M2:" + theta[1] + "M2end| M3:" + theta[2] + "M3end| M4:" + theta[3] + "M4end| M5:" + theta[4] + "M5end| M6:" + theta[5] + "\n";
  //      serial.write(message);
  //      for (int i = 0; i < theta.length; i++) {
  //        lastSentValue[i] = theta[i];
  //      }
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
}
