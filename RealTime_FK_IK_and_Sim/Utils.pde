class Utils { //This class contains a lot of the

  public void drawMatrix(int x, int y, double[][] matrix) { //Draw Matrix from data type "2d double array (double[][])", at position "x" and "y".
    fill(0); //Make the text black.
    textSize(30);
    for (int i = 0; i < matrix.length; i++) { //For every row.
      for (int j = 0; j < matrix[i].length; j++) {  //For every element in said row.
        text(nf((float)matrix[i][j], 0, 4), x + j * 100, y + i * 40); //Draw said element in said row.
      }
    }
  }
  public void drawMatrix(int x, int y, RealMatrix matrixT) { //Function with same name (fucntion overloading). Draw Matrix from data type "RealMatrix", at position "x" and "y".
    fill(0);
    textSize(30);
    if (matrixT != null) {
      double[][] matrix = matrixT.getData();
      for (int i = 0; i < matrix.length; i++) {
        for (int j = 0; j < matrix[i].length; j++) {
          text(nf((float)matrix[i][j], 0, 4), x + j * 100, y + i * 40);
        }
      }
    }
  }
  public void drawMatrix(int x, int y, double[] matrix) { //Function with same name (fucntion overloading). Draw Vector from data type "1d double array", at position "x" and "y".
    fill(0);
    textSize(30);
    for (int i = 0; i < matrix.length; i++) {
      fill(255);
      rect(x-5, y - 30 + i * 40, 100, 40);
      fill(0);
      text(nf((float)matrix[i], 0, 2), x, y + i * 40);
    }
  }
  public void drawMatrix(int x, int y, String[] matrix) { //Function with same name (fucntion overloading). Draw Vector from data type "1d string array", at position "x" and "y".
    textSize(30);
    for (int i = 0; i < matrix.length; i++) {
      //fill(255);
      //rect(x-10, y - 30 + i * 40, 100, 40);
      fill(0);
      text("Coordinate " + matrix[i], x, y + i * 40);
    }
  }

  public RealMatrix transformationMatrixFromMDH0(double[] MDH) { //This function returns a transformation matrix made from a row from modified Denavit-Hartenberg parameters.
    double alpha = Math.toRadians(MDH[0]); //Convert the MDH[0] value corresponding to alpha from degrees to radians, because cos and sin functions input is in radians.
    double a = MDH[1]; //Getting the other MDH values for this joint
    double d = MDH[2];
    double theta = Math.toRadians(MDH[3]);

    RealMatrix matrix = new Array2DRowRealMatrix(new double[][]{ //Inputting the values in this link transformation formula, and making it into a "RealMatrix".
      {Math.cos(theta), -Math.sin(theta), 0, a},
      {Math.sin(theta)*Math.cos(alpha), Math.cos(theta)*Math.cos(alpha), -Math.sin(alpha), -Math.sin(alpha)*d},
      {Math.sin(theta)*Math.sin(alpha), Math.cos(theta)*Math.sin(alpha), Math.cos(alpha), Math.cos(alpha)*d},
      {0, 0, 0, 1}});
    return matrix;  //Returns that "RealMatrix".
  }

  public double[] coordinateOutput(double[][] howdi) {  //Søren

    double y = -(howdi[2][0]);
    double x = Math.sqrt(Math.pow(howdi[0][0], 2) + Math.pow(howdi[1][0], 2));
    double Roty = Math.atan2(y, x);

    double i = howdi[2][1]/Math.cos(Roty);
    double j = howdi[2][2]/Math.cos(Roty);
    double Rotx = Math.atan2(i, j);

    double k = howdi[1][0]/Math.cos(Roty);
    double l = howdi[0][0]/Math.cos(Roty);
    double Rotz = Math.atan2(k, l);

    double[] coords = {howdi[0][3], howdi[1][3], howdi[2][3], Math.toDegrees(Rotx), Math.toDegrees(Roty), Math.toDegrees(Rotz)};
    return coords;
  }
}
