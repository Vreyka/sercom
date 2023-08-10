#include <Arduino.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <PinChangeInterrupt.h>  
#include <stdlib.h>
#include <string.h>
#include <secomlib.h>

sercom slave('s', 10, 9);
volatile char *receive_word;

void setup(){
  Serial.begin(9600);
  slave.receive();
  receive_word = slave.memory;
}
void loop(){

}