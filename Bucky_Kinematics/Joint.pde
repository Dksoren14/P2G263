class Joint {

  double[] MDHRow;
  double alpha, a, d, thetaOffset, sinAlpha, cosAlpha;
  double[][] transformationMatrix;
  RealMatrix realTransformationMatrix;
  PShape texture;

  Joint(double[] MDHRowT) {

    MDHRow = MDHRowT;
    alpha = Math.toRadians(MDHRow[0]);
    a = MDHRow[1];
    d = MDHRow[2];
    thetaOffset = Math.toRadians(MDHRow[3]);
    sinAlpha = Math.sin(alpha);
    cosAlpha = Math.cos(alpha);
    updateTransformationMatrix(0);
  }

  void updateTransformationMatrix(double thetaT) {

    double cosTheta = Math.cos(thetaOffset+thetaT);
    double sinTheta = Math.sin(thetaOffset+thetaT);

    double[][] matrix = {
      {cosTheta, -sinTheta, 0, a},
      {sinTheta*cosAlpha, cosTheta*cosAlpha, -sinAlpha, -sinAlpha*d},
      {sinTheta*sinAlpha, cosTheta*sinAlpha, cosAlpha, cosAlpha*d},
      {0, 0, 0, 1}};

    transformationMatrix = matrix;
    realTransformationMatrix = new Array2DRowRealMatrix(matrix);
  }

  void display() {
    //utils.applyRealMatrix(realTransformationMatrix);
    pushStyle();
    stroke(255, 0, 0);
    line(0, 0, 0, 100, 0, 0);
    stroke(0, 255, 0);
    line(0, 0, 0, 0, 100, 0);
    stroke(0, 0, 255);
    line(0, 0, 0, 0, 0, 100);
    noStroke();
    popStyle();
    pushMatrix();
    rotateX(radians(90));
    if (texture != null) {
      shape(texture);
    }
    popMatrix();
  }
}
