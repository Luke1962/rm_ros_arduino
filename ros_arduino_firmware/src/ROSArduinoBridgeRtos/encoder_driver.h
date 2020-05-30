/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
#ifndef __ENCODER_DRIVER__
#define __ENCODER_DRIVER__

	#define ENC_TPR 800 /*Thicks per revolution*/   
	   
	long readEncoder(int i);
	void resetEncoder(int i);
	void resetEncoders();
	double tps2rps(double tps); // converte i thick al secondo in rotazioni al secondo
	double rps2tps(double rps); // converte rotazioni al secondo  in thick al secondo
#endif
