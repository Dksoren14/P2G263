class Arm {

  RealMatrix resultMatrix;
  Joint[] jointArray;
  double[][] MDH;
  double startTime;
  double currentTime;
  double[] targetAngle;
  double[] startAngle = new double[6];
  double[] speed = new double[6];
  double[] pos = new double[6];
  double[] acc = new double[6];
  boolean executingMovement = false;
  boolean executingProgram = false;
  double[] currentPos = {0, 0, 0, 0, 0, 0}; //Tracks the current position
  int trackMovement = 0; //Tracks what movement the program is currentrly executing.

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
      shape(textures[0]);
    }
    for (int i = 0; i < jointArray.length; i++) {
      jointArray[i].updateTransformationMatrix(Math.toRadians(a[i]));
      utils.applyRealMatrix(jointArray[i].realTransformationMatrix);
      jointArray[i].display();
    }
    calculateFinalMatrix();
  }

  void draw(double[] a) {
    if (textures[0] != null) {
      shape(textures[0]);
    }
    for (int i = 0; i < jointArray.length; i++) {
      jointArray[i].updateTransformationMatrix(Math.toRadians(a[i]));
      utils.applyRealMatrix(jointArray[i].realTransformationMatrix);
      jointArray[i].display();
    }
    calculateFinalMatrix();
  }

  void draw() {
    for (int i = 0; i < jointArray.length; i++) {
      jointArray[i].updateTransformationMatrix(Math.toRadians(pos[i]));
      utils.applyRealMatrix(jointArray[i].realTransformationMatrix);
      jointArray[i].display();
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
      for (int i = 0; i < currentPos.length; i++) {
        startAngle[i] = currentPos[i];
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
      for (int i = 0; i < currentPos.length; i++) {
        currentPos[i] = pos[i];
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

  void armData() { //not done
    String message = "t1" + targetAngle[0] + "1ts1" + speed[0] + "1st2" + targetAngle[2] + "2ts3" + targetAngle[3] + "3st4" + targetAngle[4] + "M5end| M6:" + targetAngle[5] + "M6end| S1" + speed[1] + "\n";
  }
}
