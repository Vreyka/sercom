#include <Arduino.h>
#include <secomlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <PinChangeInterrupt.h>  
#include <stdlib.h>
// put function declarations here:

const uint8_t clk_pin = 10;
const uint8_t data_pin = 9;

char mode = 'm';
char data[100]= "Hello world" ;

volatile uint8_t x = 0b01100000;
sercom master('m', 10, 9);



// sercom master('m', clk_pin, data_pin);
volatile uint8_t y,z;
volatile uint8_t flag = 0;
unsigned long start=0;
void del(char *d);
void count(){
    y = micros();
    Serial.println(y);
  
}
void clk(){
  TCCR1A=0; TCCR1B=0;
        // Đầu ra PB2 là OUTPUT (pin 10)
        DDRB |= (1 << PB2);        
        // chọn Fast PWM chế độ chọn TOP_value tự do  ICR1
        TCCR1A |= (1 << WGM11);
        TCCR1B |= (1 << WGM12)|(1 << WGM13);         
        // So sánh thường( none-inverting)
        TCCR1A |= (1 << COM1B1);
        // xung răng cưa tràn sau 15 P_clock
        // ICR1 = (uint8_t)((16000000/clk)-1);
        ICR1 = 15624;
        // Value=8 -> độ rộng 50 %
        OCR1B = ICR1/2;
        //chia 1
        TCCR1B |= (1 << CS10)|(1 << CS12);
}
void setup(){
  Serial.begin(9600);
  // master.set_clk(1000000);
  clk();
  // if(flag<2){
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(10), count, RISING);
  // }
  Serial.println(z-y);
  z = sizeof(data);
  Serial.println(z);
  start =micros();
  del(data);
  start=micros()-start;
  Serial.println(start);
  
  if (data[0]){
    Serial.println("not null");
    Serial.println(data);
  }
  else {
    Serial.println("null");
  }
  
}
void loop(){
  // Serial.println("Hello world");
  
}

void del(char *d){
  d[0] = '\0';
  d[1] = '\0';
  // d[2] = '\0';
  // d[3] = '\0';
  // d[4] = '\0';
  // d[5] = '\0';
  // d[6] = '\0';
  // d[7] = '\0';
  // d[8] = '\0';
  // d[9] = '\0';
  // d[10] = '\0';
  // d[11] = '\0';
  // d[12] = '\0';
  // d[13] = '\0';
  // d[14] = '\0';
  // d[15] = '\0';
  // d[16] = '\0';
  // d[17] = '\0';
  // d[18] = '\0';
  // d[19] = '\0';
  // d[20] = '\0';
  // d[21] = '\0';
  // d[22] = '\0';
  // d[23] = '\0';
  // d[24] = '\0';
  // d[25] = '\0';
  // d[26] = '\0';
  // d[27] = '\0';
  // d[28] = '\0';
  // d[29] = '\0';
  // d[30] = '\0';
  // d[31] = '\0';
  // d[32] = '\0';
  // d[33] = '\0';
  // d[34] = '\0';
  // d[35] = '\0';
  // d[36] = '\0';
  // d[37] = '\0';
  // d[38] = '\0';
  // d[39] = '\0';
  // d[40] = '\0';
  // d[41] = '\0';
  // d[42] = '\0';
  // d[43] = '\0';
  // d[44] = '\0';
  // d[45] = '\0';
  // d[46] = '\0';
  // d[47] = '\0';
  // d[48] = '\0';
  // d[49] = '\0';
  // d[50] = '\0';
  // d[51] = '\0';
  // d[52] = '\0';
  // d[53] = '\0';
  // d[54] = '\0';
  // d[55] = '\0';
  // d[56] = '\0';
  // d[57] = '\0';
  // d[58] = '\0';
  // d[59] = '\0';
  // d[60] = '\0';
  // d[61] = '\0';
  // d[62] = '\0';
  // d[63] = '\0';
  // d[64] = '\0';
  // d[65] = '\0';
  // d[66] = '\0';
  // d[67] = '\0';
  // d[68] = '\0';
  // d[69] = '\0';
  // d[70] = '\0';
  // d[71] = '\0';
  // d[72] = '\0';
  // d[73] = '\0';
  // d[74] = '\0';
  // d[75] = '\0';
  // d[76] = '\0';
  // d[77] = '\0';
  // d[78] = '\0';
  // d[79] = '\0';
  // d[80] = '\0';
  // d[81] = '\0';
  // d[82] = '\0';
  // d[83] = '\0';
  // d[84] = '\0';
  // d[85] = '\0';
  // d[86] = '\0';
  // d[87] = '\0';
  // d[88] = '\0';
  // d[89] = '\0';
  // d[90] = '\0';
  // d[91] = '\0';
  // d[92] = '\0';
  // d[93] = '\0';
  // d[94] = '\0';
  // d[95] = '\0';
  // d[96] = '\0';
  // d[97] = '\0';
  // d[98] = '\0';
  // d[99] = '\0';
  
}