class Arm {

  RealMatrix resultMatrix;
  RealMatrix resultMatrix03;
  RealMatrix Matrix03FromIK;
  Joint[] jointArray;
  double[][] MDH;

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

  void executeMovement(double[][] targetMatrix, double targetTime, double[] startAngle) {
    double startTime = millis();
    double currentTime = millis()-startTime;
    double[] targetAngle = anglesFromIK(targetMatrix);
    

    while (currentTime < targetTime) {
      currentTime = millis()-startTime;
      for (int i = 0; i < 6; i++) {
        speed[i] = (float)trajectoryPlanning(targetAngle[i], targetTime, startAngle[i], currentTime);
        println("Speed " + i + " = " + speed[i]);
        //println("targetAngle " + i + " = " + targetAngle[i]);
      }
      utils.drawResult(speed, 900, 450);
      utils.drawResult(123, 800,450);
      //println("currentTime = " + currentTime);
      sendData();
    }
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

    //for (int i=1; i<3; i++) {
    //  jointArray[i].updateTransformationMatrix(angle[i]);
    //  Matrix03FromIK = Matrix03FromIK.multiply(jointArray[i].realTransformationMatrix);
    //}

    RealMatrix Matrix06 = new Array2DRowRealMatrix(inputMatrix);
    RealMatrix Matrix30 = new LUDecomposition(Matrix03FromIK).getSolver().getInverse();
    RealMatrix Matrix36 = Matrix30.multiply(Matrix06);
    double[][] inputMatrix36 = Matrix36.getData();
    utils.drawResult(inputMatrix36, 150, 450);
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

  double trajectoryPlanning(double endAngle, double targetTimeT, double startAngle, double currentTimeT) {
    double targetTime = targetTimeT/1000;
    double currentTime = currentTimeT/1000;
    //println(currentTime);
    double a_0 = startAngle;
    double a_1 = 0;
    double a_2 = (3/Math.pow(targetTime, 2)*(endAngle - startAngle));
    double a_3 = (-2/Math.pow(targetTime, 3)*(endAngle - startAngle));

    double speed = a_1 + 2*a_2*currentTime + 3*a_3*(Math.pow(currentTime, 2));
    //println(speed);
    return speed;
  }
}
