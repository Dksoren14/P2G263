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
