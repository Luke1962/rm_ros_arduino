/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   

	///////////////////////////////////////////////////////////////////////
	///	Encoder
	///////////////////////////////////////////////////////////////////////

	#include <Encoder.h>

	// Change these two numbers to the pins connected to your encoder.
	//   Best Performance: both pins have interrupt capability
	//   Good Performance: only the first pin has interrupt capability
	//   Low Performance:  neither pin has interrupt capability
	Encoder encL(2, 4);
	Encoder encR(3, 5);
	//   avoid using pins with LEDs attached
	///////////////////////////////////////////////////////////////////////


 
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    //if (i == LEFT) return encoders.YAxisGetCount();
    //else return encoders.XAxisGetCount();
     if (i == LEFT) return encL.read();
    else return -encR.read(); 
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    //if (i == LEFT) return encoders.YAxisReset();
    //else return encoders.XAxisReset();
     if (i == LEFT) return encL.write(0);
    else return encR.write(0);
 }
 


/* Wrap the encoder reset function */
void resetEncoders() {
	resetEncoder(LEFT);
	resetEncoder(RIGHT);

}

 

// ritorna true se si muove nell'arco di t msec
bool isMovingL(int t){
	int enc1 = encL.read();
	delay(t);
	int enc2 = encL.read();
	if(enc2 =! enc1) 
		return true ;
	else 
		return false;
}
bool isMovingR(int t){
	int enc1 = encR.read();
	delay(t);
	int enc2 = encR.read();
	if(enc2 =! enc1) 
		return true; 
	else 
		return false;
}
bool isMoving(int t=10){
	if( isMovingL(t) || isMovingR(t)) 
		return true;
	else 
		return false;
}

 double tps2rps(double tps){ // converte i thick al secondo in rotazioni al secondo
	return tps/ENC_TPR;
}
 double rps2tps(double rps){ // converte rotazioni al secondo  in thick al secondo
	return rps * ENC_TPR;
}
