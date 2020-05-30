/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef __COMMANDS_H__
#define __COMMANDS_H__
	#define GO				'@'
	#define ANALOG_READ    'a'
	#define GET_BAUDRATE   'b'
	#define PIN_MODE       'c'
	#define DIGITAL_READ   'd'
	#define READ_ENCODERS  'e'
	#define HELP           'h'
	#define MOTOR_SPEEDS   'm'
	#define PING           'p'
	#define RESET_ENCODERS 'r'
	#define SERVO_WRITE    's'
	#define SERVO_READ     't'
	#define UPDATE_PID     'u'
	#define DIGITAL_WRITE  'w'
	#define ANALOG_WRITE   'x'
	#define LEFT            0
	#define RIGHT           1
	#define TESTMOTORS      'T'
	#define PIDRATE			'R'
	#define MOTOR_RPS		'M'
	#define READ_PID		'U'
	#define SAFE_CMD		'S'
    #define CMD_DIGITAL_READ_BUMPERS 'B'
    #define MOTOR_CONTROLLER 'C' /* CONTROLLA IL SINGOLO MOTORE */

#endif


