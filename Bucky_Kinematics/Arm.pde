class Arm {

  RealMatrix resultMatrix;
  Joint[] jointArray;
  double[][] MDH;

  Arm(double[][] MDHT) {
    MDH = MDHT;
    jointArray = new Joint[MDHT.length];

    for (int i = 0; i < MDHT.length; i++) {
      jointArray[i] = new Joint(MDHT[i]);
    }
  }

  void moveArm(float a[]) {
    jointArray[1].updateTransformationMatrix(Math.toRadians(a[1]));
    jointArray[2].updateTransformationMatrix(Math.toRadians(a[2]));
    jointArray[3].updateTransformationMatrix(Math.toRadians(a[3]));
    jointArray[4].updateTransformationMatrix(Math.toRadians(a[4]));
    jointArray[5].updateTransformationMatrix(Math.toRadians(a[5]));
    jointArray[6].updateTransformationMatrix(Math.toRadians(a[6]));
    calculateFinalMatrix();
  }


  void calculateFinalMatrix() {

    resultMatrix = jointArray[0].realTransformationMatrix;
    for (int i=1; i<jointArray.length; i++) {
      resultMatrix = resultMatrix.multiply(jointArray[i].realTransformationMatrix);
    }
  }

  double[] IK(double[][] inputMatrix) {
    double[] angle = new double[7];
    
    angle[1] = Math.atan2(inputMatrix[1][3], inputMatrix[0][3]);
    double a = Math.sqrt(Math.pow(inputMatrix[0][3],2)+Math.pow(inputMatrix[1][3],2)); //(inputMatrix[0][3]/Math.cos(angle[1]))-MDH[0][2]
    text(nf((float)a, 0, 4), 200, 500);
    double b = inputMatrix[2][3]-MDH[0][2];
    text(nf((float)b, 0, 4), 200, 550);
    double c = Math.sqrt(a*a+b*b);
    text(nf((float)Math.pow(MDH[4][2],2), 0, 4), 200, 600);
    System.out.println(Math.pow(MDH[4][2],2));
    angle[2] = Math.acos((Math.pow(MDH[3][1],2)+Math.pow(c,2)+Math.pow(MDH[4][2],2))/2*MDH[3][1]*c)+Math.atan2(b,a)-Math.toRadians(90);
    text(nf((float)angle[2], 0, 4), 200, 650);
    angle[3] = Math.acos((Math.pow(MDH[3][1],2)+c*c+Math.pow(MDH[4][2],2))/2*MDH[3][1]*MDH[4][2])-Math.toRadians(90);
    
    RealMatrix Matrix03 = jointArray[0].realTransformationMatrix;
    for (int i=1; i<3; i++) {
      jointArray[i].updateTransformationMatrix(angle[i]);
      Matrix03 = Matrix03.multiply(jointArray[i].realTransformationMatrix);
    }
    RealMatrix Matrix06 = new Array2DRowRealMatrix(inputMatrix);
    RealMatrix Matrix30 = new LUDecomposition(Matrix03).getSolver().getInverse();
    RealMatrix Matrix36 = Matrix30.multiply(Matrix06);
    double[][] inputMatrix36 = Matrix36.getData();
    
    angle[5] = Math.acos(inputMatrix36[0][2]);
    angle[4] = Math.atan2(inputMatrix36[2][2],inputMatrix[0][2]);
    angle[6] = Math.atan2(inputMatrix36[1][1],inputMatrix[1][0]);
    return angle;
  }
}
