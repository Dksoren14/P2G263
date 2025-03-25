class SimStuff { //Utils for simulation stuff
  double[][] temp;

  public void coordSystem() {
    stroke(255, 0, 0);
    line(0, 0, 0, 100, 0, 0);
    stroke(0, 255, 0);
    line(0, 0, 0, 0, 100, 0);
    stroke(0, 0, 255);
    line(0, 0, 0, 0, 0, 100);
    noStroke();
  }

  public void applyRealMatrix(RealMatrix matrix) {
    temp = matrix.getData();

    float[] iPMatrix = new float[16];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        iPMatrix[i*4+j] = (float)temp[i][j];
      }
    }
    applyMatrix(iPMatrix[0], iPMatrix[1], iPMatrix[2], iPMatrix[3], iPMatrix[4], iPMatrix[5], iPMatrix[6], iPMatrix[7], iPMatrix[8], iPMatrix[9], iPMatrix[10], iPMatrix[11], iPMatrix[12], iPMatrix[13], iPMatrix[14], iPMatrix[15]);
  }

}


class Joint {

  double[] MHDRow;
  int jointNr;
  RealMatrix iMatrix;

  Joint(double[] MHDRowT, int jointNrT) {
    MHDRow = MHDRowT;
    jointNr = jointNrT;
    updateMatrix();
  }

  public void updateMatrix() {
    iMatrix = WP.transformationMatrixFromMDH0(MHDRow);
  }


}



//  public RealMatrix rotationMatrixFromAngles(double gammaT, double betaT, double alphaT) {
//    double alpha = Math.toRadians(alphaT); //alpha is rotation around z, beta around y, and gamma around x.
//    double beta = Math.toRadians(betaT);
//    double gamma = Math.toRadians(gammaT);
//    RealMatrix matrix = new Array2DRowRealMatrix(new double[][]{
//      {Math.cos(alpha)*Math.cos(beta), Math.cos(alpha)*Math.sin(beta)*Math.sin(gamma)-Math.sin(alpha)*Math.cos(gamma), Math.cos(alpha)*Math.sin(beta)*Math.cos(gamma)+Math.sin(alpha)*Math.sin(gamma)},
//      {Math.sin(alpha)*Math.cos(beta), Math.sin(alpha)*Math.sin(beta)*Math.sin(gamma)+Math.cos(alpha)*Math.cos(gamma), Math.sin(alpha)*Math.sin(beta)*Math.cos(gamma)-Math.cos(alpha)*Math.sin(gamma)},
//      {Math.sin(beta)*(-1), Math.cos(beta)*Math.sin(gamma), Math.cos(beta)*Math.cos(gamma)}});
//    return matrix;
//  }

//  public RealMatrix transformationMatrixFromMDH1(double[] MDH) {
//    double alpha = Math.toRadians(MDH[0]);
//    double a = MDH[1];
//    double d = MDH[2];
//    double theta = Math.toRadians(MDH[3]);

//    RealMatrix matrix = new Array2DRowRealMatrix(new double[][]{
//      {Math.cos(theta), -Math.sin(theta)*Math.cos(alpha), Math.sin(theta)*Math.sin(alpha), a*Math.cos(theta)},
//      {Math.sin(theta), Math.cos(theta)*Math.cos(alpha), -Math.cos(theta)*Math.sin(alpha), a*Math.sin(theta)},
//      {0, Math.sin(alpha), Math.cos(alpha), d},
//      {0, 0, 0, 1}});
//    return matrix;
//  }


//  public void drawMatrixAny(Object unknownValue) {
//    if (unknownValue instanceof Integer) {
//      int knownValue = (Integer) unknownValue;
//      println("It's an int: " + knownValue);
//    } else if (unknownValue instanceof Float) {
//      float knownValue = (Float) unknownValue;
//      println("It's a float: " + knownValue);
//    } else if (unknownValue instanceof Double) {
//      double knownValue = (Double) unknownValue;
//      println("It's a double: " + knownValue);
//    } else {
//      println("Unknown type!");
//    }
//    fill(0);
//    textSize(30);
//    for (int i = 0; i < matrix.length; i++) {
//      for (int j = 0; j < matrix[i].length; j++) {
//        text(nf((float)matrix[i][j], 0, 4), x + j * 100, y + i * 40);
//      }
//    }
    
//  }
  
