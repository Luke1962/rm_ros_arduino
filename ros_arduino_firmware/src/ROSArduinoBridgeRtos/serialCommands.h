#ifndef __SERIALCOMMANDS__
#define __SERIALCOMMANDS__

	#include "app_defines.h"

	#include "Arduino.h"
	#include "encoder_driver.h"
	#include <ChibiOS_AVR.h>
	#include "sensors.h"
	#include "commands.h"
	#include "led.h"
	#include "dbg.h"
	#include "servos.h"
	#include "diff_controller2.h"	
	#include "motor_driver.h"	
	
	/* Serial port baud rate */
	#define BAUDRATE  115200 //   57600

	void printHelp(HardwareSerial * Ser);

	/* Clear the current command parameters */
	void resetCommand();


	//Run a command on specific Serial.  Commands are defined in commands.h 
	int runCommand2(HardwareSerial *Ser); 

	// Process incoming chars on serial
	void processSerial(HardwareSerial* ser );



#endif
