///////////////////////////////////////////////////////////////////////
///	LED RGB
///////////////////////////////////////////////////////////////////////


#include "led.h"


// ////////////////////////////////////////////////////////////////////////////////////////////
//  FUNZIONI E UTILITIES GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
//void playSingleNote(int pin, int freq,int noteDuration) {
//	tone(pin, freq, noteDuration);
//	noTone(pin);
//}


void lampeggiaLed(int pin, int freq, uint16_t nvolte) {
 	int ms = 500 / freq;
	for (size_t i = 0; i < nvolte; i++)
	{
		digitalWrite(pin, 1);//digitalWriteFast(pin, 1); 
		chThdSleepMilliseconds(ms);
		digitalWrite(pin, 0);  //digitalWriteFast(pin, 0);      
		chThdSleepMilliseconds(ms);

	}

}
void setup_LED_RGB(){
	pinMode(PIN_LED_R,OUTPUT);
	pinMode(PIN_LED_G,OUTPUT);
	pinMode(PIN_LED_B,OUTPUT);
    pinMode(LED_BUILTIN,OUTPUT);
    	
	LED_RGB_OFF;

	LED_R_ON; delay(200);LED_R_OFF;
	LED_G_ON; delay(200);LED_G_OFF;		 
	LED_B_ON; delay(200);LED_B_OFF; 
	
	LED_RGB_OFF;
}



	
