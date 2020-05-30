///////////////////////////////////////////////////////////////////////
///	DEBUG
///////////////////////////////////////////////////////////////////////
//#define DEBUG_OFF
//#define DEBUG_ON

#ifndef __DBG__
#define __DBG__

	#include "app_defines.h"
	#ifdef DEBUG_ON
		#include "Arduino.h"
		#define DEBUG_SERIAL Serial1
		#define DEBUG_SERIAL_BAUD_RATE 9600

		#define dbgF(t)		 DEBUG_SERIAL.println(F(t));	 DEBUG_SERIAL.flush() ;
		#define dbg(v)		 DEBUG_SERIAL.print(v);	
		#define dbg2(t,v)	 DEBUG_SERIAL.print(F(t));DEBUG_SERIAL.println(v);  
		// #define dbg2(t,cha)    SERIAL_PC.print(t);SERIAL_PC.println(cha);//  SERIAL_SPEAK.print(t);SERIAL_SPEAK.println(cha);  


	#else
		#define dbg(cha)
		#define dbg2(t,cha)	
 
		
	#endif // DEBUG_ON
	
/*	
	void setup_debug(){
		#ifdef DEBUG_ON
			DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD_RATE);
		#endif
	}
*/	
#endif


