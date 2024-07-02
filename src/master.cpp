#include <Arduino.h>
#include <secomlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <PinChangeInterrupt.h>  
#include <stdlib.h>
#include <string.h>
// put function declarations here:

const uint8_t clk_pin = 10;
const uint8_t data_pin = 9;
#define bit_rate 400000

char mode = 'm';
char data[100]= "Hello World 2023" ;

sercom master(mode, clk_pin, data_pin);

void setup(){
  
  Serial.begin(9600);
  master.set_clk(bit_rate);
  master.send(data);
  master.setSendData();
  
  
}
void loop(){

}