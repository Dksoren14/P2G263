class Arm {



RealMatrix resultMatrix;




  void finalMatrix() {
    
    resultMatrix = jointArray[0].realTransformationMatrix;
    for (int i=1; i<jointArray.length; i++) {
      resultMatrix = resultMatrix.multiply(jointArray[i].realTransformationMatrix);
    }
  }
}
