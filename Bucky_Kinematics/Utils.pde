class Utils {



  void drawResult(float[] a, int x, int y) {
    fill(0);
    textSize(30);
    for (int i = 0; i<a.length; i++) {
      text(nf(a[i], 0, 4), x, y + i * 50);
    }
  }
  void drawResult(double[][] a, int x, int y) {
    fill(0);
    textSize(30);
    for (int i = 0; i<a.length; i++) {
      for (int j = 0; j<a[i].length; j++)
        text(nf((float)a[i][j], 0, 4), x + j * 150, y + i * 50);
    }
  }
  void drawResult(double[] a, int x, int y) {
    fill(0);
    textSize(30);
    for (int i = 0; i<a.length; i++) {
      text(nf((float)a[i], 0, 4), x, y + i * 50);
    }
  }
  void drawResult(RealMatrix r, int x, int y) {
    double[][] a = r.getData();
    fill(0);
    textSize(30);
    for (int i = 0; i<a.length; i++) {
      for (int j = 0; j<a[i].length; j++)
        text(nf((float)a[i][j], 0, 4), x + j * 150, y + i * 50);
    }
  }
  void drawResult(String[] a, int x, int y) { //Function with same name (fucntion overloading). Draw Vector from data type "1d string array", at position "x" and "y".
    textSize(30);
    for (int i = 0; i < a.length; i++) {
      //fill(255);
      //rect(x-10, y - 30 + i * 40, 100, 40);
      fill(0);
      text(a[i], x, y + i * 50);
    }
  }
  void drawResult(String a, int x, int y) { //Function with same name (fucntion overloading). Draw Vector from data type "1d string array", at position "x" and "y".
    textSize(30);
    text(a, x, y);
  }
}
