class Joint {

  double[] MDHRow;
  double alpha, a, d, thetaOffset, sinAlpha, cosAlpha;
  double[][] transformationMatrix;
  RealMatrix realTransformationMatrix;

  Joint(double[] name) {

    MDHRow = name;
    alpha = Math.toRadians(MDHRow[0]);
    a = MDHRow[1];
    d = MDHRow[2];
    thetaOffset = Math.toRadians(MDHRow[3]);
    sinAlpha = Math.sin(alpha);
    cosAlpha = Math.cos(alpha);
  }

  void trans(double thetaT) {
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
}
