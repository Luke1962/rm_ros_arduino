/*********************************************************************
 *  ROSArduinoBridgeRtos
 * 
 * A set of simple serial commands to control a differential drive
 * robot and receive back sensor and odometry data. Default 
 * configuration assumes use of an Arduino Mega +  motor
 * controller shield +  Encoder shield.  
 
 * Derived from the Pi Robot Project: http://www.pirobot.org
 * and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
 * 
 * Authors: Luca Cesarini
 * 
 * Inspired and modeled after the ArbotiX driver by Michael Ferguson
 * 
 * Software License Agreement (BSD License)
 * 
 * Copyright (c) 2012, Luca Cesarini
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
 
 // Nel main file vanno incluse le librerie usate dai moduli
 
#include "app_defines.h"
#include <ChibiOS_AVR.h>
#define yield() chThdYield()
#include "Arduino.h"
#include "dbg.h"
#include <PID_v1.h>

#include <digitalWriteFast.h>
#include "led.h"

#ifdef USE_BASE
	#include "diff_controller2.h"

#endif
#include "sensors.h"

#include "commands.h"	//Include definition of serial commands 
#include "serialCommands.h"

// Encoder driver function definitions
#include "encoder_driver.h"

#ifdef USE_SERVOS
	#include <VarSpeedServo.h> 
	#include "servos.h"
#endif	




static int bumbers=0;

// attenda dalla seriale un carattere qualsiasi
void waitForGo(){

	while (!Serial.available()) //wait for any char
	{
		yield();    // yield so other threads can run
    	
	}
	
	//bool blGo = false;
	//while (!blGo)
	while (true)	
	{
		Serial.println('@'); // send ready string 	
		while (!Serial.available()){delay(100);} //wait for any char
		
		char c = Serial.read();

		if (c == '@')
		{	
			Serial.flush();
			Serial.println('@'); // send ready string 	
			Serial.flush();
			//blGo =true;
			return;
		}else
		{
			yield(); 
		}		
	}
}

// ////////////////////////////////////////////////////////////////////////////////////////////
// OS Setup (non cambia se non si aggiungono task)
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region [CHIBIOS RTOS]

	extern unsigned int __bss_end;
	extern unsigned int __heap_start;
	extern void *__brkval;

	uint16_t getFreeSram() {
		uint8_t newVariable;
		// heap is empty, use bss as start memory address 
		if ((uint16_t)__brkval == 0)
			return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
		// use heap end as the start of the memory address 
		else
			return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
	};

	//////////////////////////////////////////////////////////////////////////////////
	//  blinking LED       ///////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	static THD_WORKING_AREA(wa_pid, 200); //size of working area
	//static THD_FUNCTION(thd_pid, arg);    //thread

	//////////////////////////////////////////////////////////////////////////////////
	//  print info/debug data        
	//////////////////////////////////////////////////////////////////////////////////
	static THD_WORKING_AREA(wa_dbg, 200); //size of working area
	//static THD_FUNCTION(thd_dbg, arg) ;   //thread





	//////////////////////////////////////////////////////////////////////////////////
	//  blinking LED       ///////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	//static THD_WORKING_AREA(wa_led, 200); //size of working area
	static THD_FUNCTION(thd_led, arg) {   //thread

		while (true)
		{
			LED_G_ON;//LED_ON;
			
			chThdSleepMilliseconds(20);

			LED_G_OFF;//LED_OFF;
			
			chThdSleepMilliseconds(1000);

		}
	}


//////////////////////////////////////////////////////////////////////////////////
// CONTROLLER THREAD
//////////////////////////////////////////////////////////////////////////////////
//static THD_WORKING_AREA(wa_pid, 200); //size of working area
static THD_FUNCTION(thd_pid, arg) {   //thread

	while (true)
	{
		

		
		diffControllerLoop();

		int pid_interval = diffControllerGetInterval();
		chThdSleepMilliseconds(pid_interval); // 20ms di default

	}
}


//////////////////////////////////////////////////////////////////////////////////
//  print info/debug data        
//////////////////////////////////////////////////////////////////////////////////
//static THD_WORKING_AREA(wa_dbg, 200); //size of working area
static THD_FUNCTION(thd_dbg, arg) {   //thread

	while (true)
	{
		// commands on debug serial
		#ifdef DEBUG_ON		
		
			//controllerPrintData();
		


			if (DEBUG_SERIAL.available() > 0)  {
				processSerial(&DEBUG_SERIAL);
			}
			chThdSleepMilliseconds(1000);
		#endif
	}
}


//////////////////////////////////////////////////////////////////////////////////
//  SERIAL COMMANDS  processing and execution thread  
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(wa_cmd, 400); //size of working area
static THD_FUNCTION(thd_cmd, arg) {   //thread

/* 	LED_B_ON;
	waitForGo();
	chThdSleepMilliseconds(1000);
	LED_B_OFF;	
 */
	while (true)
	{

           	// commands on debug serial
          	if (DEBUG_SERIAL.available() > 0)  {
          		processSerial(&DEBUG_SERIAL);
          	}

  		// commands on main serial
		if (Serial.available() > 0) {
			processSerial(&Serial);
		}
		yield(); // chThdSleepMilliseconds(25);

	}

}	

//////////////////////////////////////////////////////////////////////////////////
//  BUMPERS MONITOR     
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(wa_bmp, 400); //size of working area
static THD_FUNCTION(thd_bmp, arg) {   //thread

 
	while (true)
	{

		if (readBumpers()>1)
		{
			controllerSetTargetRPS(0, 0);
		} 
		chThdSleepMilliseconds(200);//yield(); //

	}

}	



//////////////////////////////////////////////////////////////////////////////////
// THREADS SETUP
//////////////////////////////////////////////////////////////////////////////////

void chSetup() {
  // schedule thread LED
	chThdCreateStatic(wa_pid, sizeof(wa_pid), NORMALPRIO + 4, thd_pid, NULL);  //was +3
	chThdCreateStatic(wa_cmd, sizeof(wa_cmd), NORMALPRIO + 3, thd_cmd, NULL);
	chThdCreateStatic(wa_bmp, sizeof(wa_bmp), NORMALPRIO + 4, thd_bmp, NULL); //was +3
	chThdCreateStatic(wa_led, sizeof(wa_led), NORMALPRIO + 3, thd_led, NULL); //was +3
	chThdCreateStatic(wa_dbg, sizeof(wa_dbg), NORMALPRIO + 2, thd_dbg, NULL);


  dbg("Thread chibios started");

}


//********************************************************
//********************************************************
/* Setup function--runs once at startup. */
//********************************************************
//********************************************************
void setup() {

	setup_LED_RGB();
	FARETTO_OFF;
	Serial.begin(BAUDRATE);
	LED_R_ON;
	
	#ifdef DEBUG_ON
		DEBUG_SERIAL.begin(DEBUG_SERIAL_BAUD_RATE);//setup_debug();
		dbg("----RosArduinoBridge------\n");
		dbg2("baudrate :", BAUDRATE);
		dbg("--------------------------\n");
		printHelp(&DEBUG_SERIAL);
	#endif


	// disattivo il LIDAR di default
	#define PIN_LIDAR_ON 22
	pinMode(PIN_LIDAR_ON,OUTPUT);    
	digitalWrite(PIN_LIDAR_ON, 0);
	
	
	#ifdef USE_SERVOS
		setup_servos();
		//servoSweeptest(1, 160);
		servoWrite(0,90, DEFAULT_SERVO_SPEED,false);
		
	#endif

	

	// Initialize the motor controller if used */
	#ifdef USE_BASE
		initMotorController();
		init_PID();
		resetPID(); 
		setDefaultPidsParameters();

			
		setMotorSpeeds(0,0);
	#endif


	
	pinMode(PIN_FARETTO,OUTPUT);    
	FARETTO_OFF;
	
	LED_R_OFF;
	
	initSensors();
	
   //initialize and start ChibiOS
  chBegin(chSetup);

   //should not return
  while(1);

}



/* ==============================================================
 * Enter the main loop.  Read and parse input from the serial port
 and run any valid commands. Run a PID calculation at the target
 interval and check for auto-stop conditions.
 ==============================================================
 */
 long last_print_dbg =0;
 long print_dbg_interval_ms =5000;
 long last_loop;
 long current_loop;
 long loop_comp_start ; // inizio calcoli
 long loop_remain_time ; //margine 
 long loop_comp_duration_msec ;
 
 
 
 
 //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



void loop() {

/*NOT USED*/ 
}



/*
void _loop() {
	// store loop start time
	loop_comp_start = millis();
	
	// commands on main serial
	if (Serial.available() > 0) {
		processSerial(&Serial);
	}

	// commands on debug serial
	if (DEBUG_SERIAL.available() > 0)  {
		processSerial(&DEBUG_SERIAL);
	}


  	// If we are using base control, run a PID calculation at the appropriate intervals
	// Includes: Encoder read, PID 
	updatePID();
	
	
	// faccio in modo che il loop abbia una frequenza  esattamente pari a PID_RATE
	loop_comp_duration_msec = loop_comp_start - millis();	
	loop_remain_time = PID_INTERVAL - loop_comp_duration_msec;
	
	
	if(loop_remain_time < 0 ){
		dbg2("!! PID_RATE TOO hIGH! Loop missing time msec: ", loop_remain_time);
	}
	delay(loop_remain_time);
	
}
*/
