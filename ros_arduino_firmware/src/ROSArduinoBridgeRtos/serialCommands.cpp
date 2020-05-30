	#include "serialCommands.h"
   
  	void printHelp(HardwareSerial * Ser){
		Ser->println("------- Help-------");
		Ser->print(" SERIAL BAUD RATE: ");Ser->println(BAUDRATE);
		Ser->println(" ANALOG_READ    'a'");
		Ser->println(" GET_BAUDRATE   'b'");
		Ser->println(" PIN_MODE       'c'");
		Ser->println(" DIGITAL_READ   'd'");
		Ser->println(" READ_ENCODERS  'e'");
		Ser->println(" HELP              'h'");
		Ser->println(" MOTOR_SPEEDS      'm'");
		Ser->println("    es. m 63 -63  '");
		Ser->println(" PING              'p'");
		Ser->println(" RESET_ENCODERS    'r'");
		Ser->println(" SERVO_WRITE       's'  Es s 1 30");
		Ser->println(" SERVO_READ        't'");
		Ser->println(" UPDATE_PID x1000  'u'");
		Ser->println("   Es. u 10500 2000 300 => Kp=10.5,Kd=2.0 Kd=0.3");
		Ser->println(" DIGITAL_WRITE     'w'");
		Ser->println(" ANALOG_WRITE      'x'");
		Ser->println(" PIDRATE (Hz)      'R'");
		Ser->println(" TESTMOTORS        'T'");
		Ser->println(" MOTOR SPEEDS rps  'M'");
		Ser->println(" GET PID INFO      'U'");
		Ser->println(" CMD_DIGITAL_READ_BUMPERS	 'B'");	
		Ser->println(" MOTOR_CONTROLLER 	 'C'");	
		
		Ser->println("-------------------");
	}



//Variable initialization 


  	void printHelp(HardwareSerial * Ser);
  	
	/* Variable initialization */

	// encoders reading
	long newPosL, newPosR,oldPosL,oldPosR;
	// servo reading
	int sr;
	
	// A pair of varibles to help parse serial commands (thanks Fergs)
	int arg = 0;
	int index = 0;

	// Variable to hold an input character
	char chr;

	// Variable to hold the current single-character command
	char cmd;

	// Character arrays to hold the first and second arguments
	char argv1[16];
	char argv2[16];
	char argv3[16];
	// The arguments converted to integers
	long arg1;
	long arg2;
	long arg3;


/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg = 0;
  index = 0;
}


/* Run a command on specific Serial.  Commands are defined in commands.h */
int runCommand2(HardwareSerial *Ser) {
	int i = 0;
	char *p = argv1;
	char *str;
	int pid_args[4];
	arg1 = atoi(argv1);
	arg2 = atoi(argv2);
	arg3 = atoi(argv3);	
	long eL=0; // var di appoggio per lettura encoders on demand
	long eR=0;
	LED_B_ON
	double Kp =0;
	double Ki =0;
	double Kd =0;
	
	//valori temporanei
	double Kp_ =0.0;
	double Ki_ =0.0;
	double Kd_ =0.0;
	
    int bumpers =0;
													//dbg2(cmd, argv1);
	switch(cmd) {

		case GET_BAUDRATE:
			Ser->println(BAUDRATE);
			break;

		case ANALOG_READ:
			Ser->println(analogRead(arg1));
			break;

		case DIGITAL_READ:
			Ser->println(digitalRead(arg1));
			break;

		case ANALOG_WRITE:
			analogWrite(arg1, arg2);
			Ser->println("OK"); 
			break;

		case DIGITAL_WRITE:
			pinMode(arg1, OUTPUT);
			if (arg2 == 0) digitalWrite(arg1, LOW);
			else if (arg2 == 1) digitalWrite(arg1, HIGH);
			Ser->println("OK"); 
			//dbg2("\nDIGITAL_WRITE port ",arg1);
			//dbg2("  >new val: ", arg2);
			break;

		case PIN_MODE:
			if (arg2 == 0) pinMode(arg1, INPUT);
			else if (arg2 == 1) pinMode(arg1, OUTPUT);
			Ser->println("OK");
			//dbg2("\PIN_MODE port ",arg1);
			//dbg2("  >new val: ", arg2);
			break;

		case PING:
			Ser->println(Ping(arg1));
			break;

		case SERVO_WRITE:
			if ((	arg1 <= N_SERVOS) && (arg1>0))
			{
				// trasformo arg2 da coordinate robot a Sevo				
				servoWrite(arg1,arg2, DEFAULT_SERVO_SPEED,false);
			  
				//dbg2("\nServo ",arg1);
				//dbg2("  >new pos:",arg2);
				Ser->println("OK");		  
			}
			else
			{
				dbg2("ERROR Max Servo ",N_SERVOS);
				dbg2("  >you want to command servo :",arg1);  	
			}
			break;
			
		case SERVO_READ:
			//Ser->println(servos[arg1].read());
			sr=	servoRead(arg1);
			Ser->println(sr);
			
			
			break;


		case READ_ENCODERS:
			eL = readEncoder(LEFT);
			eR = readEncoder(RIGHT);


			Ser->print(eL);
			Ser->print(" ");
			Ser->println(eR);
			//dbg2("ENCL: ",eL); 
			//dbg2("   R: ",eR);
			break;

		case RESET_ENCODERS:
			//leggo e salvo i parametri pid			
			//getPidsParameters( &Kp_,&Ki_,&Kd_);
			
			
			///li azzero perchè altrimenti azzerando gli encoder il robot si muove
			//setPidsParameters(0.0, 0.0, 0.0);	
			resetEncoders();
			resetPID();
			// reimposto i parametri PID
			//setPidsParameters( Kp_,Ki_,Kd_);			
			
			Ser->println("OK");
			dbg("RESET Encoders OK");
			break;



		case UPDATE_PID:
			Kp = (double)arg1/1000.0;
			Ki = (double)arg2/1000.0;
			Kd = (double)arg3/1000.0;
			 			
			Ser->println("OK");
			setPidsParameters(Kp,Ki, Kd);		

			
		    dbg2("\nKp= ",Kp);
			dbg2("Ki= ",Ki);
			dbg2("Kd= ",Kd);
			break;

		case HELP:
			printHelp(Ser);
			break;

		case TESTMOTORS:
			resetPID();
            setMotorSpeeds(arg1,arg2);
			delay(5000);
			
            dbg("\nTesting Motors");
			break;
            
		case PIDRATE:
 
			setPidsRate( arg1);
			Ser->println("OK");
			
			
			break;

		case MOTOR_SPEEDS: // in tps

			 
			controllerSetTargetTicksPerFrame(arg1, arg2);
			Ser->println("OK"); 			
			
			break;

		case MOTOR_RPS:/* rotazioni al secondo - comando usato dal ckb_cmdVel*/
			bumpers =readBumpers();
			if (bumpers == 0) {
				controllerSetTargetRPS(arg1, arg2);
			}else if (bumpers == 1) //back bumper
			{
				controllerSetTargetRPS(arg1, arg2);
				dbg("Bumper back");
			}else //fermo i motori
			{
				controllerSetTargetRPS(0, 0);
				dbg2("Bumper(s): ", bumpers);
			}
			//Ser->println("OK"); 			

			break;

		case MOTOR_CONTROLLER:  // deve ricevere il codice motore Left =0 o Right=1 e il valore da -63 a 63
		
			bumpers =readBumpers();
			if (bumpers == 0){
				controllerSetTargetRPS1Mot( arg1, arg2);
			}else
			{
				controllerSetTargetRPS(0, 0);
				dbg2("STOP! Bumper(s): ", bumpers);
			}					
			break;

		case READ_PID:
			dbgPrintPidsParameters(); //su seriale di dbg only
			break;
	 		
			case SAFE_CMD: //'S'
			diffControllerSafeStopSet (arg1); //su seriale di dbg only
			Ser->println("OK"); 			
			break;


		// Bumpers
		//#define PIN_BUMP_B 52
		//#define PIN_BUMP_R 49
		//#define PIN_BUMP_L 51
		//#define PIN_BUMP_F 53
		case CMD_DIGITAL_READ_BUMPERS:
                        //bumpers = readBumbers();
                        Ser->println(readBumpers());
                        break;
                        
		default:
			Ser->println("Invalid Command");
			dbg2("Invalid Command: ",cmd);
			break;
			
	}
	
	LED_B_OFF;
}



void processSerial(HardwareSerial* ser ){
  while (ser->available() > 0) {

    // Read the next character
    chr = ser->read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      else if (arg == 3) argv3[index] = NULL;
      runCommand2(ser);//was: runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      else if (arg == 2)  {
        argv2[index] = NULL;
        arg = 3;
        index = 0;
      }      
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
      else if (arg == 3) {
        argv3[index] = chr;
        index++;
      }
    }
  }

}


