
/* Functions for various sensor types */
#ifndef __SENSORS__
#define __SENSORS__
    #include "Arduino.h"

    // Bumpers
    //#define PIN_BUMP_B 52
    #define PIN_BUMP_R 49
    #define PIN_BUMP_L 51
    #define PIN_BUMP_F 53

    
    float microsecondsToCm(long microseconds);
    long Ping(int pin);
  
    void initSensors();
    int readBumpers();
#endif
