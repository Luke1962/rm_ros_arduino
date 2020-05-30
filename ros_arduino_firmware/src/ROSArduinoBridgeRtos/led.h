///////////////////////////////////////////////////////////////////////
///	LED RGB
///////////////////////////////////////////////////////////////////////
#ifndef __LED__
#define __LED__

	#include <Arduino.h>
	#include <ChibiOS_AVR.h>
	#include <digitalWriteFast.h>

	#define PIN_LED_R 30
	#define PIN_LED_G 31
	#define PIN_LED_B 32
	#define PIN_FARETTO 12
	
	//Per i led con Vcc common
	#define LED_ON_LOGIC 0
	#define LED_OFF_LOGIC 1

	#define LED_R_ON  digitalWrite(PIN_LED_R, LED_ON_LOGIC);
	#define LED_R_OFF digitalWrite(PIN_LED_R, LED_OFF_LOGIC);
	#define LED_G_ON  digitalWrite(PIN_LED_G, LED_ON_LOGIC);
	#define LED_G_OFF digitalWrite(PIN_LED_G, LED_OFF_LOGIC);
	#define LED_B_ON  digitalWrite(PIN_LED_B, LED_ON_LOGIC);
	#define LED_B_OFF digitalWrite(PIN_LED_B, LED_OFF_LOGIC);
	#define LED_RGB_OFF   LED_R_OFF;LED_G_OFF;LED_B_OFF;


	#define FARETTO_ON  digitalWrite(PIN_FARETTO, 1);
	#define FARETTO_OFF digitalWrite(PIN_FARETTO, 0);

	#define LED_ON digitalWrite(LED_BUILTIN, 1);
	#define LED_OFF digitalWrite(LED_BUILTIN, 0);





	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  FUNZIONI E UTILITIES GLOBALI
	// ////////////////////////////////////////////////////////////////////////////////////////////
	//void playSingleNote(int pin, int freq,int noteDuration) {
	//	tone(pin, freq, noteDuration);
	//	noTone(pin);
	//}
	void setup_LED_RGB();



	void lampeggiaLed(int pin, int freq, uint16_t nvolte);

	//////////////////////////////////////////////////////////////////////////////////
	//  blinking LED       ///////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	static THD_WORKING_AREA(wa_led, 200); //size of working area
	//static THD_FUNCTION(thd_led, arg);



	
#endif
