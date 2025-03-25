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

  public void updateMDH() { //This is where the Denavit-Hartenberg parameters is inserted. It make a "new"(overwrites) MDH 2D double every time it is called.
    MDH = new double[][] { //The reason why it overwrites the old MDH every time is because it is the easiest way update the theta values.
      {0, 0, 0, theta0},
      {0, 0, 122.65, theta1},
      {-90, 39.43, 0, theta2-90},
      {0, 115.49, 0, theta3},
      {-90, 0, 115.49*2, theta4},
      {90, 0, 0, theta5},
      {-90, 0, 0, theta6}};

    for (int i = 0; i < joints.length; i++) {
      if (joints[i] == null) {                //Initializing joints if they are not allready.
        joints[i] = new Joint(MDH[i], i);
      } else {
        joints[i].MHDRow = MDH[i];      //inputting the updated MDH values into the corrosponding joint.
        joints[i].updateMatrix();      //Updating the joint's transformation matrix, now based on the updated MDH.
      }
    }
  }

  public void calculateResultMatrix() {    //This calculates the result transformation matrix from joint 1 to the last joint in MDH.
    WP.updateMDH();
    result = WP.transformationMatrixFromMDH0(MDH[1]);
    for (int i = 2; i < MDH.length; i++) {
      result = result.multiply(WP.transformationMatrixFromMDH0(MDH[i]));   //the "result" matrix is a global variable.
    }
    //matrix01 = WP.transformationMatrixFromMDH1(MDH[1]);                      //This is what it does:
    //matrix02 = matrix01.multiply(WP.transformationMatrixFromMDH1(MDH[2]));   //Multiply the last result matrix, with the next transformation matrix.
    //matrix03 = matrix02.multiply(WP.transformationMatrixFromMDH1(MDH[3]));
    //matrix04 = matrix03.multiply(WP.transformationMatrixFromMDH1(MDH[4]));
    //matrix05 = matrix04.multiply(WP.transformationMatrixFromMDH1(MDH[5]));
    //matrix06 = matrix05.multiply(WP.transformationMatrixFromMDH1(MDH[6]));
    //result = matrix06;
    double[][] data = result.getData();
    WP.coordinateOutput(data);
  }

  public void coordinateOutput(double[][] howdi) {  //Søren

    double y = -(howdi[2][0]);
    double x = Math.sqrt(Math.pow(howdi[0][0], 2) + Math.pow(howdi[1][0], 2));
    double Roty = Math.atan2(y, x);

    double i = howdi[2][1]/Math.cos(Roty);
    double j = howdi[2][2]/Math.cos(Roty);
    double Rotx = Math.atan2(i, j);

    double k = howdi[1][0]/Math.cos(Roty);
    double l = howdi[0][0]/Math.cos(Roty);
    double Rotz = Math.atan2(k, l);

    cordinateOutput[0] = howdi[0][3];
    cordinateOutput[1] = howdi[1][3];
    cordinateOutput[2] = howdi[2][3];
    cordinateOutput[3] = Math.toDegrees(Rotx);
    cordinateOutput[4] = Math.toDegrees(Roty);
    cordinateOutput[5] = Math.toDegrees(Rotz);
  }
}
