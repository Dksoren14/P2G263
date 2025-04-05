class Arm {
  NewJoint[] joints1;

  Arm(double[][] MDHT) {
    joints1 = new NewJoint[MDHT.length+1];
    double[] coordSystem0 = {0,0,0,0};
    joints1[0] = new NewJoint(coordSystem0);
    for (int i = 0; i < MDHT.length; i++) {
      joints1[i+1] = new NewJoint(MDHT[i]);
    }
  }

  void moveAndDraw() {
    double[] theta = {theta0, theta1, theta2, theta3, theta4, theta5, theta6};
    for (int i = 0; i < joints1.length; i++) {
      joints1[i].angle(Math.toRadians(theta[i]));
      joints1[i].translate();
      joints1[i].draw();
    }
  }
}
