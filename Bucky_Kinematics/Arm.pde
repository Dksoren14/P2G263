class Arm {

  RealMatrix resultMatrix;
  RealMatrix resultMatrix03;
  RealMatrix Matrix03FromIK;
  Joint[] jointArray;
  double[][] MDH;
  double startTime;
  double currentTime;
  double[] targetAngle;
  double[] speed = new double[6];
  float[] pos = new float[6];
  boolean haveRun = false;

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

  void moveAndDraw(float[] a) {
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

  void moveAndDraw(double[] a) {
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


  void calculateFinalMatrix() {

    resultMatrix = jointArray[0].realTransformationMatrix;
    for (int i=1; i<jointArray.length; i++) {
      resultMatrix = resultMatrix.multiply(jointArray[i].realTransformationMatrix);
    }

    resultMatrix03 = jointArray[0].realTransformationMatrix;
    for (int i=1; i<3; i++) {
      resultMatrix03 = resultMatrix03.multiply(jointArray[i].realTransformationMatrix);
    }
  }

  int executeMovement(double[][] targetMatrix, double targetTime, double[] startAngle) {
    int temp = 0; 
    if (!haveRun) {
      startTime = millis();
      currentTime = millis()-startTime;
      targetAngle = anglesFromIK(targetMatrix);
      haveRun = true;
    }

    if (currentTime < targetTime) {
      currentTime = millis()-startTime;
      for (int i = 0; i < 6; i++) {
        speed[i] = abs((float)getSpeed(targetAngle[i], targetTime, startAngle[i], currentTime));
        pos[i] = (float)getPos(targetAngle[i], targetTime, startAngle[i], currentTime);
      }
      theta = pos;
      sendData();
    }
    if (millis() > startTime+targetTime) {
      keyVariableA = false;
      haveRun = false;
      temp = 1; 
    }
    return temp; 
  }




  double[] anglesFromIK(double[][] inputMatrix) {
    double[] angle = new double[6];

    angle[0] = Math.atan2(inputMatrix[1][3], inputMatrix[0][3]);
    double a = Math.sqrt(Math.pow(inputMatrix[0][3], 2)+Math.pow(inputMatrix[1][3], 2))-MDH[1][1]; //(inputMatrix[0][3]/Math.cos(angle[1]))-MDH[0][2]
    double b = inputMatrix[2][3]-MDH[0][2];
    double c = Math.sqrt(a*a+b*b);
    angle[1] = Math.acos((Math.pow(MDH[2][1], 2)+Math.pow(c, 2)-Math.pow(MDH[3][2], 2))/(2*MDH[2][1]*c))+Math.atan2(b, a)-Math.toRadians(90);

    angle[2] = Math.acos((Math.pow(MDH[2][1], 2)+Math.pow(MDH[3][2], 2)-(c*c))/(2*MDH[2][1]*MDH[3][2]))-Math.toRadians(90);

    jointArray[0].updateTransformationMatrix(angle[0]);
    Matrix03FromIK = jointArray[0].realTransformationMatrix;
    jointArray[1].updateTransformationMatrix(angle[1]);
    Matrix03FromIK = Matrix03FromIK.multiply(jointArray[1].realTransformationMatrix);
    jointArray[2].updateTransformationMatrix(angle[2]);
    Matrix03FromIK = Matrix03FromIK.multiply(jointArray[2].realTransformationMatrix);

    RealMatrix Matrix06 = new Array2DRowRealMatrix(inputMatrix);
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
}
