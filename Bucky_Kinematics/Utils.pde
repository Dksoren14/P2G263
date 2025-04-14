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
      for(int j = 0; j<a[i].length; j++)
      text(nf((float)a[i][j], 0, 4), x + j * 150, y + i * 50);
    }
  }
}
