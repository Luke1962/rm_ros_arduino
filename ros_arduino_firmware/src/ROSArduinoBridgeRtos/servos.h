#ifndef __SERVOS_H__
#define __SERVOS_H__


		#include "app_defines.h"
	
		#include <Arduino.h>
		#include "dbg.h" 	
		#include <VarSpeedServo.h> 
		
		#define N_SERVOS 1
		// block moved to .cpp
		//VarSpeedServo servos [N_SERVOS];
		//int servoSpeeds[N_SERVOS] = { 100 }; // 0 = low speed 255 = max speed
		//byte servoPins[N_SERVOS] = { 9 };// Pins
		//byte servoInitPosition [N_SERVOS] = { 140 }; // [0, 180] degrees// Initial Position
		//byte servoOffsets [N_SERVOS] = { 40 }; //offset: angolo di rotazione del frame 'servo' rispetto asse y del robot
		#define DEFAULT_SERVO_SPEED 160

		/*
		 *Ho impostato un offset fisico in modo da proteggere il flat cable della raspicam
		 * Per orientare la RASPICAM in orizzontale, va considerato che
		 * 
		 * A 0° il servo punta tutto indietro (flat cable completamente teso)
		 * A 180 il servo punta in avanti con un angolo di -40°  
		 * L'angolo desiderato invece viene espresso nel frame del Robot tra 0 (avanti) e 140° (indietro)
		 */
		 
		 
		#define SERVO_OFFSET_DEG 40
		#define SERVO_MIN 0
		//SERVO_MAX= 180-servoOffsets[i]

		#define SERVO_DEFAULT 140 // 180-alfa  

		void setup_servos();


		// in:angolo rispetto al robot
		//out: angolo rispetto al servo
		// servo id = 1....
		//cmd tra SERVO_MIN e 180-servoOffsets[servo_id-1];
		void servoWrite(int servo_id, int cmd , int speed, bool blWait);
		
		
		int servoRead(int servo_id);// ritorna l'angolo del servo nelle coordinate robot

		void servoSweeptest(int servo_id, int speed);



#endif
