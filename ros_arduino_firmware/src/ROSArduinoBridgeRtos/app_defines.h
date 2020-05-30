// collects all DEFINES options at application level
#ifndef __APP_DEFINES__
#define __APP_DEFINES__

	#define DEBUG_ON	// enable debug on serial

	#define USE_BASE      // Enable the base controller code
	//#undef USE_BASE     // Disable the base controller code
	
	
	/* Define the motor controller and encoder library you are using */
	#ifdef USE_BASE
		/* The Pololu VNH5019 dual motor driver shield */
		//#define POLOLU_VNH5019

		/* The Pololu MC33926 dual motor driver shield */
		//#define POLOLU_MC33926

		/* The RoboGaia encoder shield */
		//#define ROBOGAIA
		#define    MYENCODERS

		/* Encoders directly attached to Arduino board */
		//#define ARDUINO_ENC_COUNTER

		/* L298 Motor driver*/
		//#define L298_MOTOR_DRIVER
	#endif

	#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
	//#undef USE_SERVOS     // Disable use of PWM servos

	//#define DEBUG_SERIAL Serial1
#endif
