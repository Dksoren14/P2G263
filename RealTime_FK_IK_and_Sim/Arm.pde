class Arm {
  Joint[] joints;

  Arm(double[][] MDHT) {

    joints = new Joint[MDHT.length+1];
    double[] coordSystem0 = {0, 0, 0, 0};
    joints[0] = new Joint(coordSystem0);
    for (int i = 0; i < MDHT.length; i++) {
      joints[i+1] = new Joint(MDHT[i]);
    }
  }

  public void moveAndDraw(double[] a) {
    
    for (int i = 0; i < joints.length; i++) {
      joints[i].angle(Math.toRadians(a[i]));
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
  
  public void IK(double[][] T06Input) {
    double L0 = 122.65;
    double L1 = 39.43;
    double L2 = 115.49;
    double L3 = 115.49*2;
    double offset2 = 90;
    theta[1] = Math.atan2(T06Input[1][3],T06Input[0][3]);
    double a = (T06Input[0][3]/Math.cos(Math.toRadians(theta[1])))-L1;
    double b = T06Input[2][3]-L0;
    double c = Math.sqrt(a*a+b*b);
    theta[2] = Math.atan2(b,a)+Math.acos((L2*L2+c*c+L3*L3)/2*L2*c)-offset2;
    theta[3] = Math.acos((L2*L2+L3*L3+c*c)/2*L2*L3)-offset2;
    RealMatrix Matrix03 = joints[0].realTransformationMatrix;
    for (int i = 1; i < 3; i++) {
      Matrix03 = Matrix03.multiply(joints[i].realTransformationMatrix);
    }
    
  }
  
  
  
}
