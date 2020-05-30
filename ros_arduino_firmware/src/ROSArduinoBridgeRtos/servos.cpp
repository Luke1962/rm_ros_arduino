#ifndef SERVOS_H
#define SERVOS_H



		#include "servos.h"
		
			VarSpeedServo servos [N_SERVOS];
			int servoSpeeds[N_SERVOS] = { 100 }; // 0 = low speed 255 = max speed
			byte servoPins[N_SERVOS] = { 9 };// Pins
			byte servoInitPosition [N_SERVOS] = { 140 }; // [0, 180] degrees// Initial Position
			byte servoOffsets [N_SERVOS] = { 40 }; //offset: angolo di rotazione del frame 'servo' rispetto asse y del robot


		void setup_servos(){
			for (int i = 0; i < N_SERVOS; i++) {
				servos[i].attach(servoPins[i]);
			}
		}



		// in:angolo rispetto al robot
		//out: angolo rispetto al servo
		// servo id = 1....
		//cmd tra SERVO_MIN e 180-servoOffsets[servo_id-1];
		void servoWrite(int servo_id, int cmd , int speed, bool blWait){
			
			//check servo id
			if ((servo_id >0 )&& (servo_id <= N_SERVOS))
			{
				servo_id = servo_id -1;
				//limiter
				int servoMax = 180 - servoOffsets[servo_id];
				if (cmd > servoMax)	{cmd = servoMax;}
				if (cmd < 0)	{cmd = 0;}
			
				// transform demand to servo reference
				int servoRot = (180- servoOffsets[servo_id] - cmd );
				
				//command servo
				servos[servo_id].write(servoRot,speed,  blWait);
				
			}else
			{
				dbg2("Error servo id: ", servo_id );
			}
							
		}
		
		// ritorna l'angolo del servo nelle coordinate robot
		int servoRead(int servo_id){
			//check servo id
			if ((servo_id >0 )&& (servo_id <= N_SERVOS))
			{
				servo_id = servo_id -1;
				int sr= servos[servo_id].read();
				int angle = 180- servoOffsets[servo_id] - sr  ;		
			}	
		}

		void servoSweeptest(int servo_id, int speed){

			//	servoWrite(servo_id,SERVO_MIN, speed, true);
			//	servoWrite(servo_id,180 -servoOffsets[servo_id-1], speed, true);
			//	servoWrite(servo_id,servoInitPosition[servo_id-1], speed, true);
			dbg("\n ---Test Servo sweep start ---\n");
			dbg("Servo -> 0");servos[servo_id-1].write(0 , speed, false); delay(1000);
			dbg("Servo -> 180");servos[servo_id-1].write(180, speed, false); delay(2000);
			dbg("\nServo -> 90");servos[servo_id-1].write(  90, speed,false);delay(2000);
			
			dbg("\n--End  Servo Sweep test---\n");
			
		}





#endif
