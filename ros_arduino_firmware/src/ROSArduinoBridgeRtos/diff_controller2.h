
/* 
 * Sostituito Pid con Libreria Pid_V1
 * File che fa da abstraction layer tra l'applicazione e la specifica libreria utilizzata
*/
#ifndef __DIFF_CONTROLLER2__
#define __DIFF_CONTROLLER2__

	#include <ChibiOS_AVR.h>
	#include <PID_v1.h>
	#include "motor_driver.h"
	#include "encoder_driver.h"
	#include "dbg.h"	
	/* Maximum PWM signal */
	#define MAX_PWM        63

	/* Run the PID loop at 30 times per second */
	#define PID_RATE_DEFAULT  50    // => rotazione 1 Hz = 1 TargetTicksPerFrame
	
	/* Stop the robot if it hasn't received a movement command
	 in this number of milliseconds */
	#define AUTO_STOP_INTERVAL 2000

	typedef struct{
		long lastMotorCommand;

		int pid_rate;					// Hz
		int pid_interval;				// msec 

		double TargetTicksPerFrame;		// target speed in ticks per frame
		double TargetTicksPerSec;    	// target speed in ticks per frame
		double TargetRPS;				// target rotation per sec

		double currentTicksPerFrame;	// Encoder-PrevEnc 
		double currentTicksPerSec;		// 
		double currentRPS;				// computed encoder speed in rot/s
		
		long Encoder;               	// encoder count
		long PrevEnc;            		// last encoder count

		double Kp;
		double Ki;
		double Kd;

		double motorOutput;				// last motor setting

		int lastLoopTime_ms;			// ms necessary to compute loop --> max pid frequency

	}
	PidData;

 
	
	
	void controllerSetTargetRPS1Mot(int leftRight, double rps);
	void controllerSetTargetRPS(double rpsL, double rpsR);

	void controllerSetTargetTicksPerFrame(float speedL, float speedR);

	void setPidsParameters(double Kp,double Ki,double Kd);
	void setDefaultPidsParameters();
	void setPidsRate(int hz);
	void setPidsIntervalMsec(int pidInterval_msec);  // must be > lastLoopTime_ms + command processing time
	void init_PID();
	void setup_PID(int pidInterval_msec, int outputLimits_Low , int outputLimit_High);

	void resetPID();//?


	//Called by main Loop.  Read the encoder values and call the PID routine 
	void updatePID();

	void dbgPrintPidsParameters();
	void controllerPrintData();	


	void diffControllerLoop();	//esegue il loop di controllo
	
	
	int diffControllerGetInterval();
	

	void diffControllerSafeStopSet (int on); //imposta SafetyStop
	void diffControllerSafeStopCheck(); // Se on <> 0, in assenzza di comandi sui motori per [on] mSec ,li ferma 
	void getPidsParameters(double *Kp,double *Ki,double *Kd); 

#endif
