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

char mode = 'm';
char data[100]= "Hello world" ;

sercom master(mode, clk_pin, data_pin);
// void x(){
//   Serial.println("FALLING");
// }
void setup(){
  
  Serial.begin(9600);
  master.set_clk(484848);
  master.send(data);
  master.setSendData();
  // attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(10), x, FALLING);
  
}
void loop(){
  // x = master.StateMachine;
  // Serial.println(x);
  // DDRB |= (1 << DDB1);
  // PORTB |= (1 << PORTB1);
  // delay(1000);
  // PORTB &= ~(1 << PORTB1);
  // delay(1000);

}