/* 
 * Sostituito Pid con Libreria Pid_V1
 * File che fa da abstraction layer tra l'applicazione e la specifica libreria utilizzata
*/
#include "diff_controller2.h"


/*
// Maximum PWM signal 
#define MAX_PWM        63

//Define the aggressive and conservative Tuning Parameters
double aggKp=40000, aggKi=2000, aggKd=0;
//double consKp=1, consKi=0.05, consKd=0.25;
double consKp=1, consKi=1, consKd=0.001;

// Run the PID loop at 30 times per second 
int PID_RATE = 48 ;    // => rotazione 1 Hz = 1 TargetTicksPerFrame

//Convert the rate into an interval 
int PID_INTERVAL = 1000 / PID_RATE;

//Define Variables we'll be connecting to
//double SetpointL, InputL, OutputL;
//double SetpointR, InputR, OutputR;

// Dati in input al PID
typedef struct{
	double TargetTicksPerFrame;    // target speed in ticks per frame
	double CurrentTicksPerFrame;  // Encoder-PrevEnc
	double motorOutput;                    // last motor setting
	long Encoder;                  // encoder count
	long PrevEnc;                  // last encoder count
	double ticksPerSec;				//computed encoder speed
	double Kp;
	double Ki;
	double Kd;

}
PidData;
PidData pidDataL, pidDataR;

//Specify the links and initial tuning parameters
//PID myPIDL(&InputL, &OutputL, &SetpointL, consKp, consKi, consKd, DIRECT);
//PID myPIDR(&InputR, &OutputR, &SetpointR, consKp, consKi, consKd, DIRECT);
//Specify the links and initial tuning parameters

// 
PID myPIDL(&pidDataL.currentTicksPerFrame , &pidDataL.motorOutput, &pidDataL.TargetTicksPerFrame, consKp, consKi, consKd, DIRECT);
PID myPIDR(&pidDataR.currentTicksPerFrame , &pidDataR.motorOutput, &pidDataR.TargetTicksPerFrame, consKp, consKi, consKd, DIRECT);

*/

	//Define the aggressive and conservative Tuning Parameters
	double aggKp=40000, aggKi=2000, aggKd=0;
	//double consKp=1, consKi=0.05, consKd=0.25;
//	double consKp=1, consKi=1, consKd=0.001; //ci mette un po' a fermarsi
	double consKp=1.2, consKi=0, consKd=0.020;


	/* Convert the rate into an interval */
	//int PID_INTERVAL = 1000 / PID_RATE;




		long lastMotorCommand = 0;


	//Define Variables we'll be connecting to
	//double SetpointL, InputL, OutputL;
	//double SetpointR, InputR, OutputR;

	// Dati in input al PID

static	PidData pidDataL, pidDataR;

	//Specify the links and initial tuning parameters
	//PID myPIDL(&InputL, &OutputL, &SetpointL, consKp, consKi, consKd, DIRECT);
	//PID myPIDR(&InputR, &OutputR, &SetpointR, consKp, consKi, consKd, DIRECT);
	//Specify the links and initial tuning parameters

	PID myPIDL(&pidDataL.currentTicksPerFrame , &pidDataL.motorOutput, &pidDataL.TargetTicksPerFrame, consKp, consKi, consKd, DIRECT);
	PID myPIDR(&pidDataR.currentTicksPerFrame , &pidDataR.motorOutput, &pidDataR.TargetTicksPerFrame, consKp, consKi, consKd, DIRECT);


void init_PID(){
	setup_PID(1000.0/PID_RATE_DEFAULT, -MAX_PWM, MAX_PWM);
}



void setup_PID(int pidInterval_msec, int outputLimits_Low , int outputLimit_High){
	//turn the PID on
	myPIDL.SetMode(AUTOMATIC);
    myPIDR.SetMode(AUTOMATIC);
    
    myPIDL.SetOutputLimits(outputLimits_Low, outputLimit_High);
    myPIDR.SetOutputLimits(outputLimits_Low, outputLimit_High);
    
    setPidsParameters(consKp,consKi, consKd);
    setPidsIntervalMsec(pidInterval_msec);
    
    pidDataL.Encoder =0;
	pidDataR.Encoder =0;	
    pidDataL.PrevEnc =0;
	pidDataR.PrevEnc =0;

}

void setDefaultPidsParameters(){
	setPidsParameters(consKp,consKi,consKd);
}
/*Called by main Loop.  Read the encoder values and call the PID routine */
void updatePID() {
	// Setpoint in 
	// pidDataL.TargetTicksPerFrame = targetTiks/s / PID_RATE
	
	/* Read the encoders */
	pidDataL.Encoder = readEncoder(LEFT);
	pidDataR.Encoder = readEncoder(RIGHT);	
	
	pidDataL.currentTicksPerFrame  = pidDataL.Encoder - pidDataL.PrevEnc;
	pidDataR.currentTicksPerFrame  = pidDataR.Encoder - pidDataR.PrevEnc;
	pidDataL.currentTicksPerSec = pidDataL.currentTicksPerFrame  * pidDataL.pid_rate;
	pidDataR.currentTicksPerSec = pidDataR.currentTicksPerFrame  * pidDataR.pid_rate;
	/*	
	 * double gapL = abs(leftPID.TargetTicksPerFrame-leftPID.EncoderSpeed); //distance away from setpoint

	 	
	 	*
	if(gap<10)
	{  //we're close to setpoint, use conservative tuning parameters
		myPID.SetTunings(consKp, consKi, consKd);
	}
	else
	{
		 //we're far from setpoint, use aggressive tuning parameters
	 	myPID.SetTunings(aggKp, aggKi, aggKd);
	}*/


	/* Compute PID update for each motor */
	myPIDL.Compute();
	myPIDR.Compute();
	
		/* Set the motor speeds accordingly to PID output*/
	setMotorSpeeds(pidDataL.motorOutput, pidDataR.motorOutput);
	
	pidDataL.PrevEnc = pidDataL.Encoder;
	pidDataR.PrevEnc = pidDataR.Encoder;
}


void setPidsParameters(double Kp,double Ki,double Kd){
    pidDataL.Kp = Kp;
    pidDataL.Ki = Ki;
    pidDataL.Kd = Kd;
    
    pidDataR.Kp = Kp;
    pidDataR.Ki = Ki;
    pidDataR.Kd = Kd;  

    myPIDL.SetTunings(pidDataL.Kp, pidDataL.Ki, pidDataL.Kd);
    myPIDR.SetTunings(pidDataR.Kp, pidDataR.Ki, pidDataR.Kd);
    
	dbgPrintPidsParameters();
}

void getPidsParameters(double *Kp,double *Ki,double *Kd){
    *Kp = pidDataL.Kp ;
    *Ki = pidDataL.Ki ;
    *Kd = pidDataL.Kd ;
    
}


//--Frequenza PID  (Hz) --------------------------------------
void setPidsRate(int hz){
	pidDataL.pid_rate = hz;
	pidDataR.pid_rate = hz;
	
	pidDataL.pid_interval = 1000.0/ hz;
	pidDataR.pid_interval = 1000.0/ hz;	
	
    myPIDL.SetSampleTime(pidDataL.pid_interval); // in mSec
    myPIDR.SetSampleTime(pidDataR.pid_interval ); // in mSec
    
    
  	dbgPrintPidsParameters(); 

}
void setPidsIntervalMsec(int pidInterval_msec){

	pidDataL.pid_interval = pidInterval_msec;
	pidDataR.pid_interval = pidInterval_msec;	

	pidDataL.pid_rate = 1000.0 /pidInterval_msec;
	pidDataR.pid_rate = 1000.0 /pidInterval_msec;

    myPIDL.SetSampleTime(pidInterval_msec); // in mSec
    myPIDR.SetSampleTime(pidInterval_msec); // in mSec
    
	
	dbgPrintPidsParameters();
}


void dbgPrintPidsParameters(){

			dbg("\nPID Parameters_____________ \n");
		    dbg2("Kp= ",pidDataL.Kp);
			dbg2("Ki= ",pidDataL.Ki);
			dbg2("Kd= ",pidDataL.Kd);
			dbg("\n");
		    dbg2("pid_interval......ms ",pidDataL.pid_interval);
		    dbg2("pid_rate..........Hz ",pidDataL.pid_rate);
		    
			dbg("\n");
		    dbg2("TargetTicksPerFrame  ",pidDataL.TargetTicksPerFrame);
		    dbg2("TargetTicksPerSec..  ",pidDataL.TargetTicksPerSec);
		    dbg2("TargetRPS..........  ",pidDataL.TargetRPS);
			dbg("\n");
		    dbg2("currentTicksPerFrame.... ",pidDataL.currentTicksPerFrame);
		    dbg2("currentTicksPerSec...... ",pidDataL.currentTicksPerSec);	
		    dbg2("currentRPS.............. ",pidDataL.currentRPS);
		    		    
		    	    
		    dbg2("motorOutput......... ",pidDataL.motorOutput);

			dbg( "_____________________________\n");
		    
}

int diffControllerGetInterval(){
	return pidDataL.pid_interval;
}

void resetPID(){
	pidDataL.lastMotorCommand = millis();
	pidDataR.lastMotorCommand = millis();
	
	pidDataL.Encoder = readEncoder(LEFT);
	pidDataR.Encoder = readEncoder(RIGHT);	
	pidDataL.PrevEnc = pidDataL.Encoder;
	pidDataR.PrevEnc = pidDataR.Encoder;

}





// chiamato da 'm' -----------------------------------------------
void controllerSetTargetTicksPerFrame(float tpfL, float tpfR){

	
	//pidDataL.TargetTicksPerFrame = (double)speedL/pidDataL.pid_rate;
	//pidDataR.TargetTicksPerFrame = (double)speedR/pidDataR.pid_rate;
	pidDataL.TargetTicksPerFrame = (double)tpfL;
	pidDataR.TargetTicksPerFrame = (double)tpfR;
	
	pidDataL.TargetTicksPerSec	= pidDataL.TargetTicksPerFrame * pidDataL.pid_rate;
	pidDataR.TargetTicksPerSec	= pidDataR.TargetTicksPerFrame * pidDataR.pid_rate;
	
	pidDataL.TargetRPS	= tps2rps( pidDataL.TargetTicksPerFrame );
	pidDataR.TargetRPS	= tps2rps( pidDataR.TargetTicksPerFrame );
		
	pidDataL.lastMotorCommand = millis();
	pidDataR.lastMotorCommand = millis();
	
	//dbgPrintPidsParameters();

	
}

// velocità in rotazioni al secondo asse motore 
// chiamato dal comando 'M'
void controllerSetTargetRPS(double rpsL, double rpsR){
	controllerSetTargetRPS1Mot( 0 ,  rpsL);
	controllerSetTargetRPS1Mot( 1 ,  rpsR);

	/*
		pidDataL.TargetRPS	= rpsL;
		pidDataR.TargetRPS	= rpsR;
			
		pidDataL.TargetTicksPerSec =  rps2tps(rpsL)	;
		pidDataR.TargetTicksPerSec =  rps2tps(rpsR)	;
		
		pidDataL.TargetTicksPerFrame = pidDataL.TargetTicksPerSec / pidDataL.pid_rate;
		pidDataR.TargetTicksPerFrame = pidDataR.TargetTicksPerSec / pidDataR.pid_rate;

		pidDataL.lastMotorCommand = millis();
		pidDataR.lastMotorCommand = millis();
	*/
	//dbgPrintPidsParameters();

}
void controllerSetTargetRPS1Mot(int leftRight, double rps){
	if (leftRight ==0)
	{
		pidDataL.TargetRPS	= rps;
		pidDataL.TargetTicksPerSec =  rps2tps(rps)	;
		pidDataL.TargetTicksPerFrame = pidDataL.TargetTicksPerSec / pidDataL.pid_rate;
		pidDataL.lastMotorCommand = millis();
 
	}else
	{
		pidDataR.TargetRPS	= rps;
		pidDataR.TargetTicksPerSec =  rps2tps(rps)	;
		pidDataR.TargetTicksPerFrame = pidDataR.TargetTicksPerSec / pidDataR.pid_rate;
		pidDataR.lastMotorCommand = millis();

	}

}

void controllerPrintData(){

 		dbg("\nEncL  ");dbg(pidDataL.Encoder); dbg("  R ");dbg(pidDataR.Encoder); dbg2("    Diff. ", abs(pidDataL.Encoder)-abs(pidDataR.Encoder));
		 
		//dbg2("\nThicks/s L ",pidDataL.currentTicksPerSec);
		//dbg2("   rot/s L ",tps2rps(pidDataL.currentTicksPerSec));
		dbg2("PID loop:", pidDataL.lastLoopTime_ms);
}	



//---------------------------------------------
// SAFETY STOP GUARD MECHANISM
//---------------------------------------------
bool blSafeStop = true;
int safeStopMsec = 2000;


void diffControllerSafeStopSet (int on){ //comando 'S'
	if (on > 0)
	{
		blSafeStop =true;
		safeStopMsec = on;
	}else
	{
		blSafeStop =false;
		safeStopMsec = 0;
	}

}
 // Se on <> 0, in assenzza di comandi sui motori per [on] mSec ,li ferma 
 void diffControllerSafeStopCheck(){
 	if (blSafeStop)
	 {
	 	if ( (millis() - lastMotorCommand ) > safeStopMsec  )
		 {
			controllerSetTargetTicksPerFrame(0,0);
			dbg2("\n ### SAFETY STOP AFTER mSec: ", safeStopMsec);
		 }
		 
		
	 }
	 
 }

//---------------------------------------------



//loop di controllo chiamato dal task RTOS 
void diffControllerLoop(){

		//diffControllerSafeStopCheck();

		long t1 = millis();

		/* Read the encoders */
		pidDataL.Encoder = readEncoder(LEFT);
		pidDataR.Encoder = readEncoder(RIGHT);	
		
		
		
		pidDataL.currentTicksPerFrame  = pidDataL.Encoder - pidDataL.PrevEnc;
		pidDataR.currentTicksPerFrame  = pidDataR.Encoder - pidDataR.PrevEnc;
		
		pidDataL.currentTicksPerSec =pidDataL.currentTicksPerFrame  * pidDataL.pid_rate; 
		pidDataR.currentTicksPerSec =pidDataR.currentTicksPerFrame  * pidDataR.pid_rate;
		
		pidDataL.currentRPS =tps2rps(pidDataL.currentTicksPerSec);
		pidDataR.currentRPS =tps2rps(pidDataR.currentTicksPerSec);



		
		/*	 tpf2tps()
		 * double gapL = abs(leftPID.TargetTicksPerFrame-leftPID.EncoderSpeed); //distance away from setpoint

			
			*
		if(gap<10)
		{  //we're close to setpoint, use conservative tuning parameters
			myPID.SetTunings(consKp, consKi, consKd);
		}
		else
		{
			 //we're far from setpoint, use aggressive tuning parameters
			myPID.SetTunings(aggKp, aggKi, aggKd);
		}*/


		/* Compute PID update for each motor */
		myPIDL.Compute();
		myPIDR.Compute();
		
		/* Set the motor speeds accordingly to PID output*/
		setMotorSpeeds(pidDataL.motorOutput, pidDataR.motorOutput);
		
		pidDataL.PrevEnc = pidDataL.Encoder;
		pidDataR.PrevEnc = pidDataR.Encoder;

		pidDataL.lastLoopTime_ms = millis()-t1;
		pidDataR.lastLoopTime_ms = pidDataL.lastLoopTime_ms;
}


