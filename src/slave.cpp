#include <Arduino.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <PinChangeInterrupt.h>  
#include <stdlib.h>
#include <string.h>
#include <secomlib.h>
#include <LiquidCrystal_I2C.h>

sercom slave('s', 10, 9);
volatile char *receive_word;
volatile String receive_word2;
LiquidCrystal_I2C lcd(0x27, 16, 2);
char d[100];
String d2;

void setup(){
  Serial.begin(9600);
  slave.receive();
  d2 = d;
  // receive_word2 = String( slave.memory);
  lcd.init();
  lcd.backlight();
}
void loop(){
  for(int i=0; i<100; i++){
    d[i] = slave.memory[i];
    
  }

  d2 = d;
  lcd.clear();                 // clear display
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("Arduino");        // print message at (0, 0)
  lcd.setCursor(2, 1);         // move cursor to   (2, 1)
  lcd.print(d2); // print message at (2, 1)
  delay(2000);
}