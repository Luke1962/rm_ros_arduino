
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	INTERFACCIA SERIALE ALLA SCHEDA DRIVER MOTORI  MDDS10
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////

	//#include <cstdlib>
	#include <math.h> /// per fabs()
	//#include "parameters.h"
	//#include "MyRobotModelSmall.h"
	//#include <wiringPi.h>
#include "Arduino.h"

#include "mydcmotor.h" /// per fabs()

// sign function for float numbers
static inline int8_t fsgn(float val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}
/*
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
template <class T> const T& min (const T& a, const T& b) {
  return !(b<a)?a:b;     // or: return !comp(b,a)?a:b; for version (2)
}

int map(float x, float in_min, float in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/
mydcmotor_c::mydcmotor_c(  int leftRight) //leftRight = LEFT or RIGHT
{
	if(leftRight == LEFT)
		_channel = 0;
	else
		_channel = 128; /*1000 0000*/
	//init();
	//stop();
};

void mydcmotor_c::init(int fd)
{

	//_fd = fd;	// file descriptor della seriale
	SERIAL_MOTOR.begin(SERIAL_MOTOR_BAUDRATE);
	
	isMoving=false;
	_targetVelocity = 0.0;

	//_commandDir = DIR_S ; //stop;
	_blSpeedLimiterOn = true;
	_rpm = 0.0;
	stop();
}

void mydcmotor_c::setLimiter(int OnOff){
	if(OnOff==1){
	_blSpeedLimiterOn = true;
	}
	else{
	_blSpeedLimiterOn = false;
	}
}
/// ---------------------------------------------------------
/// invia la richiesta di velocità tra 0 e 63 alla seriale
/// ---------------------------------------------------------
void mydcmotor_c::motorsSetSpeed0_63signed(int signedSpeed){

	
	 // 0 =reverse 1000000= 64=forward
	uint8_t speed0_63 =0 ;
	uint8_t Bit6_direction = FORWARD; 
	uint8_t cmd = 0;
	if (signedSpeed  != 0)
	{
		//limit input speed between -63 and 63
		if(signedSpeed >  63){signedSpeed =  63;}
		if(signedSpeed < -63){signedSpeed = -63;}

		//uint8_t speedcmd = speed; 
		//speedcmd = 30; //solo per test
	
		if (signedSpeed > 0 ) { //forward
			speed0_63 = signedSpeed;
			Bit6_direction = FORWARD ; //bit 6 =0
		
		}else //reverse
		{
			speed0_63 = -signedSpeed;
			Bit6_direction = REVERSE; // bit 6 = 1
			
		}
		
		//printf("\n\t\t[Driver cmd %d: Speed %d ]\n",cmd, speedcmd);
	}
	
	cmd = _channel + Bit6_direction + speed0_63;
	SERIAL_MOTOR.write(cmd);

}

/////////////////////////////////////////////////////
///  COMANDO CHIAMATO DALL'ESTERNO
///////////////////////////////////////////////////
// targetVelocity è la velocità tangenziale (e non di rotazione) di ogni ruota rispetto al suolo

void mydcmotor_c::goCmdVel(float targetVelocity) 
{
	const float maxSpeedLinear = 1.0; //m/s
	if (_blSpeedLimiterOn)
	{
		_targetVelocity = _cmdVelLimit(targetVelocity);
		
	}else
	{
		_targetVelocity = targetVelocity;
		
	}
	if (fabs(_targetVelocity) !=  0)
	{
		
		/// converto da velocità lineare in rpm
		isMoving = true;	/// segnala che si sta muovendo
		// v = [omega_rad/s] * ROBOT_WHEEL_DIAMETER/2
		//_rpm= _targetVelocity/MAX ;	///ROBOT_M2STEPS 3536.7765	
		
	//	int speed_0_63 =(int8_t)( (_targetVelocity/maxSpeedLinear)*63.0 ); 
		int speed_0_63 =(int8_t)map(_targetVelocity,-1.0,1,-63,63);
		
		dbg2("Set Target Vel ", _targetVelocity);
		dbg2("\t speed_0_63: ", speed_0_63);
		motorsSetSpeed0_63signed(speed_0_63); 
	}
	else///stop generazione clock
	{
		_en = 0;
		isMoving = false;	// segnala che si sta muovendo
		//_commandDir = DIR_S;
		_targetVelocity=0.0;
		motorsSetSpeed0_63signed(0); 
		//dbg("Stop");
		
	}
	

}

/// Ferma i motori disabilitando gli avvolgimenti
void mydcmotor_c::stop() {
	// disabilito l'alimentazione degli avvolgimenti
	motorsSetSpeed0_63signed(0);
}

/// abilita i motori 
void mydcmotor_c::_enable() {
//	digitalWrite(_Pin_MotEN, ROBOT_MOTORENABLE_ACTIVE);
	_blEnabled = true;
}

/*
/// ritorna la direzione corrente in formato stringa
char mydcmotor_c::getCmdDirStr() {

	switch (_commandDir)
	{
	case DIR_F:
		return 'F';
		break;
	case DIR_B:
		return 'B';
		break;

	case DIR_S:
		return 'S';//stop
		break;
	default:
		return '?';
		break;
	}

}

*/



////////////////////////////////////////////////////////
/// limitatore di velocità
////////////////////////////////////////////////////////

float mydcmotor_c::_cmdVelLimit( float cmdVel){

	if (fabs(cmdVel) != 0){
		if (fabs(cmdVel) <  (float)MOTOR_MINIMUM_ANGULAR_VELOCITY ){
				cmdVel = fsgn(cmdVel)*MOTOR_MINIMUM_ANGULAR_VELOCITY;
				dbg2("WARNING: MINIMUM SPEED SET TO %f", cmdVel);	
				
		}
		else if ( fabs(cmdVel ) >  (float)1 ){
				cmdVel = fsgn(cmdVel)*MOTOR_MAXIMUM_ANGULAR_VELOCITY;
				dbg2("WARNING: SPEED LIMITED TO:  %f",cmdVel);				
				
		}						
	}
	return cmdVel;		
}




