class Joint {

  double[] MDHRow;
  double alpha, a, d, thetaOffset, sinAlpha, cosAlpha;
  double[][] transformationMatrix;
  RealMatrix realTransformationMatrix;

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
    double theta = thetaT+thetaOffset;

    RealMatrix matrix = new Array2DRowRealMatrix(new double[][]{ //Inputting the values in this link transformation formula, and making it into a "RealMatrix".
      {Math.cos(theta), -Math.sin(theta), 0, a},
      {Math.sin(theta)*Math.cos(alpha), Math.cos(theta)*Math.cos(alpha), -Math.sin(alpha), -Math.sin(alpha)*d},
      {Math.sin(theta)*Math.sin(alpha), Math.cos(theta)*Math.sin(alpha), Math.cos(alpha), Math.cos(alpha)*d},
      {0, 0, 0, 1}});


    //double tarsda = Math.toRadians(thetaT);
    //double cosTheta = Math.cos(thetaOffset+tarsda);
    //double sinTheta = Math.sin(thetaOffset+tarsda);
    //double var1 = cosTheta  ;
    //double var2 = -sinTheta;
    //double var3 = sinTheta*cosAlpha;
    //double var4 = cosTheta*cosAlpha;
    //double var5 = -sinAlpha;
    //double var6 = -sinAlpha*d;
    //double var7 = sinTheta*sinAlpha;
    //double var8 = cosTheta*sinAlpha;
    //double var9 =  cosAlpha;
    //double var10 = cosAlpha*d;

    //double[][] matrix = {
    //  {var1, var2, 0, a},
    //  {var3, var4, var5, var6},
    //  {var7, var8, var9, var10},
    //  {0, 0, 0, 1}};

    //double[][] matrix = {
    //{var1, var2, 0, a},
    //{sinTheta*cosAlpha, cosTheta*cosAlpha, -sinAlpha, -sinAlpha*d},
    //{sinTheta*sinAlpha, cosTheta*sinAlpha, cosAlpha, cosAlpha*d},
    //{0, 0, 0, 1}};

    //    transformationMatrix = matrix;
    //    realTransformationMatrix = new Array2DRowRealMatrix(matrix);

    transformationMatrix = matrix.getData();
    realTransformationMatrix = matrix;
  }
}
