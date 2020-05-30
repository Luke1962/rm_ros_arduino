/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
#ifndef __MOTOR_DRIVER__
#define __MOTOR_DRIVER__

	#ifdef L298_MOTOR_DRIVER
	  #define RIGHT_MOTOR_BACKWARD 22
	  #define LEFT_MOTOR_BACKWARD  23
	  #define RIGHT_MOTOR_FORWARD  24
	  #define LEFT_MOTOR_FORWARD   25
	  #define RIGHT_MOTOR_ENABLE 26
	  #define LEFT_MOTOR_ENABLE  27
	#endif

	void initMotorController();
	void setMotorSpeed(int i, int spd);
	void setMotorSpeeds(int leftSpeed, int rightSpeed);
	void testMotors(int speed, int sec);

	#define LEFT 0
	#define RIGHT 1
	

#endif
