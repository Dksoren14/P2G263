class Utils {



  void drawResult(float[] a, int x, int y) {
    pushStyle();
    fill(0);
    textSize(30);
    for (int i = 0; i<a.length; i++) {
      text(nf(a[i], 0, 4), x, y + i * 50);
    }
    popStyle();
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
  void drawResult(double a, int x, int y) {
    fill(0);
    textSize(30);
    text(nf((float)a, 0, 4), x, y);
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
    fill(0);
    textSize(30);
    for (int i = 0; i < a.length; i++) {
      //fill(255);
      //rect(x-10, y - 30 + i * 40, 100, 40);
      fill(0);
      text(a[i], x, y + i * 50);
    }
  }
  void drawResult(String a, int x, int y) { //Function with same name (fucntion overloading). Draw Vector from data type "1d string array", at position "x" and "y".
    fill(0);
    textSize(30);
    text(a, x, y);
  }
  
   public void applyRealMatrix(RealMatrix matrix) {
    double[][] temp = matrix.getData();

    float[] iPMatrix = new float[16];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        iPMatrix[i*4+j] = (float)temp[i][j];
      }
    }
    applyMatrix(iPMatrix[0], iPMatrix[1], iPMatrix[2], iPMatrix[3], iPMatrix[4], iPMatrix[5], iPMatrix[6], iPMatrix[7], iPMatrix[8], iPMatrix[9], iPMatrix[10], iPMatrix[11], iPMatrix[12], iPMatrix[13], iPMatrix[14], iPMatrix[15]);
  }
  void nextButton(){

  if(isMouseOverNextCircle()){
    fill(nextCircleColor[0],nextCircleColor[1]-100,nextCircleColor[2]);
    if(nextButtonPress()){
      fill(nextCircleColor[0]-155,nextCircleColor[1]-105,nextCircleColor[2]);
      //if (showPopup) {
      //  fill(255);
      //  stroke(0);
      //  rect(100, 80, 200, 140);
      //  fill(0);
      //  text("This is a popup!", 200, 120);
      //  text("Click to close", 200, 160);
      //}
    }
   }else{
     fill(nextCircleColor[0],nextCircleColor[1],nextCircleColor[2]);
   }
   
   ellipse(buttonNextX, buttonNextY, buttonNextW, buttonNextH);
   fill(0,0,0); textSize(24); text("Next", 1175, 805);
}
void stopButton(){

  if(isMouseOverStopCircle()){
    fill(stopCircleColor[0]-100,stopCircleColor[1],stopCircleColor[2]);
    if(stopButtonPress()){
    fill(stopCircleColor[0],stopCircleColor[1]+100,stopCircleColor[2]);
    //if (showPopup) {
    //  fill(255);
    //  stroke(0);
    //  rect(100, 80, 200, 140);
    //  fill(0);
    //  text("This is a popup!", 200, 120);
    //  text("Click to close", 200, 160);
    //}
    }
   }else{
     fill(stopCircleColor[0],stopCircleColor[1],stopCircleColor[2]);
   }
   
   ellipse(buttonStopX, buttonStopY, buttonStopW, buttonStopH);
   fill(0,0,0); textSize(24); text("STOP", 523, 805);
}
void goButton(){
  
  
    if (isMouseOverGoCircle()) {
      fill(GocircleColor[0],GocircleColor[1]-100,GocircleColor[2]);
      if(goButtonPress()){
        fill(GocircleColor[0],GocircleColor[1]-155,GocircleColor[2]);
        showPopup = true;  
        buttonText = "Confirming";
      } 
    }else {
      fill(GocircleColor[0],GocircleColor[1],GocircleColor[2]);
    
  }
   
  ellipse(buttonGoX, buttonGoY, buttonGoW, buttonGoH);
  fill(0,0,0);textSize(24); text(buttonText, 1310, 805);
  textSize(24); text("Go", 1485, 805);
}


  
  
}
