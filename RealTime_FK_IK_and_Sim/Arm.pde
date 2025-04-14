class Arm {
  NewJoint[] joints;

  Arm(double[][] MDHT) {

    joints = new NewJoint[MDHT.length+1];
    double[] coordSystem0 = {0, 0, 0, 0};
    joints[0] = new NewJoint(coordSystem0);
    for (int i = 0; i < MDHT.length; i++) {
      joints[i+1] = new NewJoint(MDHT[i]);
    }
  }

  public void moveAndDraw() {
    double[] theta = {theta0, theta1, theta2, theta3, theta4, theta5, theta6};
    for (int i = 0; i < joints.length; i++) {
      joints[i].angle(Math.toRadians(theta[i]));
      joints[i].translate();
      joints[i].draw();
    }
  }

  public void drawResult(int x, int y) {
    RealMatrix resultMatrix;
    resultMatrix = joints[0].realTransformationMatrix;
    for (int i = 1; i < joints.length; i++) {
      resultMatrix = resultMatrix.multiply(joints[i].realTransformationMatrix);
    }
    double[][] asdhkjh = resultMatrix.getData();
    WP.drawMatrix(x, y, WP.coordinateOutput(asdhkjh));                    //Drawing a matrix with "drawMatrix" function from "class Utils". The instance of "class Utils" is called "WP".
    WP.drawMatrix(x+100, y, coordinateNames);                    //same
  }
}
