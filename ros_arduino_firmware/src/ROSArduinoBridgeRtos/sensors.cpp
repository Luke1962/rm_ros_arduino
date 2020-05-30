
/* Functions for various sensor types */

#include "sensors.h"
float microsecondsToCm(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per cm.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long Ping(int pin) {
  long duration, range;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(28);
  digitalWrite(pin, HIGH);
  delayMicroseconds(29);
  digitalWrite(pin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  // convert the time into meters
  range = microsecondsToCm(duration);

  return(range);
}

//ritorna intero > 0 se uno dei bumpers è attivato
int readBumpers(){
  int bump=0;
  //bumpers = digitalRead(PIN_BUMP_B);
  bump += !digitalRead(PIN_BUMP_F)<<1; // bit 1 =2
  bump += !digitalRead(PIN_BUMP_L)<<2; // bit 2 =4
  bump += !digitalRead(PIN_BUMP_R)<<3; // bit 3 =8
  return bump;
}

//inizializza le porte e i sensori
void initSensors(){

  /* bumpers in pullup mode in quanto la chiusura dello switch mette a massa*/
  pinMode(PIN_BUMP_R, INPUT_PULLUP);
  pinMode(PIN_BUMP_L, INPUT_PULLUP);
  pinMode(PIN_BUMP_F, INPUT_PULLUP);
}
