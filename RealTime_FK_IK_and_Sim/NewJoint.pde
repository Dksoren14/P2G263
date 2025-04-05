
class NewJoint {

  double[] MDHRow;
  double[][] transformationMatrix;

  double alpha, cosAlpha, sinAlpha;
  double a;
  double d;
  double thetaOffset;

  NewJoint(double[] MDHRowT) {
    MDHRow = MDHRowT;
    alpha = Math.toRadians(MDHRow[0]);
    a = MDHRow[1];
    d = MDHRow[2];
    thetaOffset = Math.toRadians(MDHRow[3]);
    sinAlpha = Math.sin(alpha);
    cosAlpha = Math.cos(alpha);
    
    transformationMatrix = angle(0);
  }

  public double[][] angle(double thetaT) {
   
    double cosTheta = Math.cos(thetaOffset+thetaT);
    double sinTheta = Math.sin(thetaOffset+thetaT);
    
    double[][] matrix = {
      {cosTheta, -sinTheta, 0, a},
      {sinTheta*cosAlpha, cosTheta*cosAlpha, -sinAlpha, -sinAlpha*d},
      {sinTheta*sinAlpha, cosTheta*sinAlpha, cosAlpha, cosAlpha*d},
      {0, 0, 0, 1}};
      transformationMatrix = matrix;
    return matrix;
  }

  public void translate() {
    float[] iPMatrix = new float[16];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        iPMatrix[i*4+j] = (float)transformationMatrix[i][j];
      }
    }
    applyMatrix(iPMatrix[0], iPMatrix[1], iPMatrix[2], iPMatrix[3], iPMatrix[4], iPMatrix[5], iPMatrix[6], iPMatrix[7], iPMatrix[8], iPMatrix[9], iPMatrix[10], iPMatrix[11], iPMatrix[12], iPMatrix[13], iPMatrix[14], iPMatrix[15]);
  }

  public void draw() {
    SS.coordSystem();
  }
}
