/***************************************************************
ABSTRACTION LAYER DI PILOTAGGIO COMBINATO DELLA COPPIA DI MOTORI
CON VALORI DI VELOCITA' DA  0 A 63
   *************************************************************/

#ifdef USE_BASE

	/// ///////////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////
	/// DRIVER MOTORI 
	/// ///////////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////
	#include "mydcmotor.h"
	
	
	// Create the motor driver object
	// DualVNH5019MotorShield drive;	
  	static mydcmotor_c mot_L(LEFT);
	static mydcmotor_c mot_R(RIGHT);




  
  // Wrap the motor driver initialization 
  void initMotorController() {
    SERIAL_MOTOR.begin(SERIAL_MOTOR_BAUDRATE);
    setMotorSpeeds(0,0);
  }

  // Wrap the drive motor set speed function 
  void setMotorSpeed(int i, int spd0_63_signed) {
    //if (i == LEFT) drive.setM1Speed(spd);
    //else drive.setM2Speed(spd);
    
    
    //if (i == LEFT) mot_L.goCmdVel(spd) ;
    //else mot_R.goCmdVel(spd) ;//  fattore -1 per tenere conto del motore ruotato di 180° rispetto all'altro
    if (i == LEFT) mot_L.motorsSetSpeed0_63signed(spd0_63_signed) ;
    else mot_R.motorsSetSpeed0_63signed(spd0_63_signed) ;//  fattore -1 per tenere conto del motore ruotato di 180° rispetto all'altro

  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }




#endif
