#include <Arduino.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <ChaCha.h>
#include <stdlib.h>
#include "sercom.h"
#include <PinChangeInterrupt.h>  



sercom* sercom::instancePtr;

sercom::sercom(char type, uint8_t clk_port, uint8_t data_port) {
    
    if (type == 'm') {
        this->state = 1;
    } else if (type == 's') {
        this->state = 0;
    }
    else this->state = -1;

    if (this->state != -1) {    
        this->isInterrupt = true;
        this->CLP = clk_port;
        this->DTP = data_port; 

        this->portdata = digitalPinToPort(this->DTP);
        this->datapin  = digitalPinToBitMask(this->DTP);

        this->portclk = digitalPinToPort(this->CLP);
        this->clkpin  = digitalPinToBitMask(this->CLP);

        this->StateMachine = 0;

        if (this->state == 1) {
            pinMode(this->CLP, OUTPUT);
            // pinMode(this->DTP, OUTPUT);
            this->isSending = false;
            this->countStart = 0;
            this->MESS = 0;
            this->isReceiving = false;
        } 
        else if (this->state == 0) {
            pinMode(this->CLP, INPUT);
            // pinMode(this->DTP, INPUT);
            this->isReceiving = true;
            this->START = 0;
            this->isRecived = false;
            this->isStarted = false;
            this->countStart = 0;
            this->isSended = 0;
            this->isMemset = 0;
            this->dataId = 0;
            this->parity = 0;
        }
    } else {
        Serial.println("Error: Wrong type of sercom");
    }

}

void sercom::set_clk(long clk) {

    if (this->state == 1) {
        // RESET lại 2 thanh ghi
        TCCR1A=0; TCCR1B=0;
        // Đầu ra PB2 là OUTPUT (pin 10)
        DDRB |= (1 << PB2);        
        // chọn Fast PWM chế độ chọn TOP_value tự do  ICR1
        TCCR1A |= (1 << WGM11);
        TCCR1B |= (1 << WGM12)|(1 << WGM13);         
        // So sánh thường( none-inverting)
        TCCR1A |= (1 << COM1B1);
        // xung răng cưa tràn sau 15 P_clock
        ICR1 = (uint8_t)((16000000/clk)-1);
        // ICR1 = 15624;
        // Value=8 -> độ rộng 50 %
        OCR1B = ICR1/2;
        //chia 1
        TCCR1B |= (1 << CS10);

    }
    else {
        Serial.println("Error: Wrong type of sercom to set clock");
    }
}

void sercom::print() {
    Serial.print("CLK: ");
    Serial.println(this->CLP);
    Serial.print("DATA: ");
    Serial.println(this->DTP);
    Serial.print("State: ");
    Serial.println(this->state); 
}

//hàm xử lý ngắt cho việc truyền dữ liệu của master
void sercom::Mcall() {
    if(instancePtr->state==1){
        switch(instancePtr->StateMachine){
            case 0:
                if((instancePtr->isSending)&&(instancePtr->countStart <3)){
                    instancePtr->setStartToCLK();
                    
                    if(instancePtr->countStart == 2){
                        instancePtr->countStart = 0;
                        instancePtr->StateMachine = 1;
                    }
                    else{
                        instancePtr->countStart++;
                    }
                }            
                break;
            case 1:
                if((instancePtr->isSending)&&(instancePtr->countStart<=3)){
                    //wait data from slave to 3 posedge
                    if(instancePtr->countStart == 3){
                        instancePtr->countStart = 0;
                        instancePtr->isReceiving = 0;
                        //data is recived
                        if(instancePtr->isRecived == 1){
                            instancePtr->isRecived = 0;
                            instancePtr->StateMachine = 2;
                            instancePtr->setDataToCLK();
                        }
                        //resend
                        else{
                            instancePtr->setStartToCLK();
                            instancePtr->StateMachine = 0;
                        }
                    }
                    else{
                        instancePtr->wait();
                        instancePtr->countStart++;
                        instancePtr->isReceiving = 1;
                    }
                }
                else if(instancePtr->isSending==0){
                    instancePtr->StateMachine = 0;
                }
                break;
            case 2:
                if(instancePtr->isSending){

                    //handle cho việc dịch chuyển từng kí tự
                    if(instancePtr->nextChar==0){
                        instancePtr->setDataToCLK();
                    }
                    else{
                        if(instancePtr->countStart <= 3){
                            if(instancePtr->countStart == 3){

                                instancePtr->countStart = 0; 
                                instancePtr->isReceiving = 0; 

                                if((instancePtr->isRecived == 1)&&(instancePtr->isNoERROR == 1)){
                                    instancePtr->isRecived = 0;
                                    instancePtr->setDataToCLK();
                                }
                                else{
                                    instancePtr->ResetDataToCLK();
                                }
                            }
                            else{
                                instancePtr->wait();
                                instancePtr->isReceiving = 1;
                                instancePtr->countStart++;
                            }
                        }                
                    }

                }
                else{
                    //handle cho kí tự cuối cùng và kết thúc việc truyền dữ liệu
                    if(instancePtr->countStart <= 3){
                        
                        if(instancePtr->countStart == 3){
                            instancePtr->countStart = 0; 
                            instancePtr->isReceiving = 0;

                            if((instancePtr->isRecived == 1)&&(instancePtr->isNoERROR == 1)){
                                instancePtr->isRecived = 0;
                                instancePtr->StateMachine = 0;
                            }
                            else {
                                instancePtr->ResetDataToCLK();
                            }
                        }
                        else{
                            instancePtr->wait();
                            instancePtr->countStart++;
                            instancePtr->isReceiving = 1;
                        }
                    }
                }            
                break;    
            default:
                instancePtr->StateMachine = 0;
                break;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
        }

    }
    else{
        Serial.println("Error: Wrong type of sercom");
    }
    
}

//hàm xử lý ngắt cho việc nhận thông báo của slave
void sercom::MRcall(){
    if(instancePtr->state==1){
        if((instancePtr->StateMachine==1)||(instancePtr->StateMachine==2)){
            if(instancePtr->isReceiving){
                instancePtr->getMessFromSlave();
            }
            if (instancePtr->countStart==3){
                if((instancePtr->MESS==0b100)||(instancePtr->MESS==0b111)){
                    instancePtr->isRecived = 1;
                    instancePtr->isNoERROR = 1;
                }
                else if((instancePtr->MESS==0b001)||(instancePtr->MESS==0b010)){
                    instancePtr->isRecived = 1;
                    instancePtr->isNoERROR = 0;
                }

                else if (instancePtr->MESS==0b000){
                    instancePtr->isRecived = 0;
                    instancePtr->isNoERROR = 0;
                }
                else {
                    instancePtr->isRecived = 1;
                    instancePtr->MESS &= 0b100;
                    if(instancePtr->MESS){
                        instancePtr->isNoERROR = 1;
                    } 
                    else{
                        instancePtr->isNoERROR = 0;
                    }
                instancePtr->MESS = 0;
                }
            }
            
        }
    }
    else{
        Serial.println("Error: Wrong type of sercom");
    }
}

void sercom::wait(){
    DDRB |= (1 << DDB1);
}

void sercom::setSendData(){
    if (this->state == 1) {
        Serial.println("Sending");
        this->parity = 0;
        instancePtr = this;
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(this->CLP), Mcall, FALLING);
        attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(this->CLP), MRcall, RISING);
        sei();
    }
    else {
        Serial.println("Error: Wrong type of sercom");
    }
}


void sercom::setDataToCLK(){
    if (this->state == 1) {
        DDRB |= (1 << DDB1);
        this->nextChar = 0;

        if(this->charId==0){
            this->DataToSend = this->data[this->dataId];
        }
        if(this->charId <8){        
            volatile bool bitToSend = (this->DataToSend >> this->charId) & 0x01;

            if(bitToSend){
                this->portdata |= (1<<this->datapin);
                this->parity = !this->parity;
            }
            else{
                this->portdata &= ~(1<<this->datapin);
            }
            this->charId++;
        }
        else if (this->charId > 8){
            if(this->parity==0){
                this->portdata |= (1<<this->datapin);
            }
            else{
                this->portdata &= ~(1<<this->datapin);
            }
            if (this->data[this->dataId+1]!= '\0'){
                this->dataId++;
            }
            else{
                this->isSending = false;
            }
            this->nextChar = 1;
            this->parity = 0;
            this->charId=0;
        }
        else if (this->charId == 8){
            if (this->data[this->dataId+1]!= '\0'){
                this->portdata |= (1<<this->datapin);
                this->parity = !this->parity;
            }
            else {
                this->portdata &= ~(1<<this->datapin);
                
            }
            this->charId++;
        }

    }
    else {
        Serial.println("Error: Wrong type of sercom");
    }
    
}

void sercom::ResetDataToCLK(){
    if (this->state == 1) {
        if(this->isSending){
            this->dataId -= 1;
        }
        else{
            this->isSending = true;
        }
        this->setDataToCLK();
    }
    else {
        Serial.println("Error: Wrong type of sercom");
    }
}

void sercom::getMessFromSlave(){
    if(this->state==1){
    volatile uint8_t shift = this->countStart - 1;
    this->MESS |= (PINB & (1 << PINB1))<<shift;   
    }
    else {
        Serial.println("Error: Wrong type of sercom");
    } 
}

void sercom::send(char *data){
    if (this->state == 1) {
        if(!this->isSending){
            this->charId = 0;
            this->dataId = 0;
            this->data = data;
            this->isSending = true;
            this->isReceiving = false;
            this->nextChar = 0;
        }    
    }
    else {
        Serial.println("Error: Wrong type of sercom");
    }
}

void sercom::receiveStart(){
    //always wait for the start bit
    if (this->state == 0) {        
        
        if (this->countStart<3){

            DDRB |= (1 << DDB1);
            this->START |= (PINB & (1 << PINB1))<<this->countStart;
            this->countStart++;

            if (this->countStart==3){

                this->countStart = 0;

                if (this->START==0b111){
                    this->isStarted = true;
                    this->isReceiving = false;
                    this->StateMachine = 1;
                    this->START = 0;
                }
            }
        }

    }
    else {
        Serial.println("Error: Wrong type of sercom");
    }
}


void sercom::SSwait(){
    if (this->state == 0){
        this->countStart++;
        if (this->countStart==2){
            countStart = 0;
            if((this->isMemset)&&(this->isSended)){
                this->StateMachine  = 2;
            }
            else this->StateMachine = 0;            
        }
    }
    else {
        this->isInterrupt = false;
        Serial.println("Error: Wrong type of sercom");
    }
}

void sercom::receiveData(){
    if (this->state == 0) {
        DDRB |= (1 << DDB1);
        if (this->countStart<9){
            this->MESS |= (PINB & (1 << PINB1))<<this->countStart;
            if(PINB & (1 << PINB1)){
                this->parity = !this->parity;
            }
            if (this->countStart==7){
                this->memory[this->dataId] = this->MESS;
            }
        }
        this->countStart++;
    }
    else {
        Serial.println("Error: Wrong type of sercom");
    }
}

//for posedge 
void sercom::Scall(){
    if (instancePtr->state == 0) {
        switch (instancePtr->StateMachine){
            //  wait for start bit, if started -> state = 1
            case 0:
                instancePtr->receiveStart();
                break;
            //wait get data for 3 posedge to send mess for start
            //may be longer for prepare memory
            //chỉ khi memset = 1 thì state mới được phép -> 2
            //nếu đếm đủ posedge (count = 2) mà memset chưa = 1 thì state = 0
            case 1:
                instancePtr->SSwait();

                break;
            case 2:
                instancePtr->receiveData();
                //nếu count = 9 thì so sánh px = (!p == d[8])?
                //  NE = !px = 1 
                // state = 3
                break;
            //wait for 10 posedge to send mess for data
            case 3:
                //gửi NE, !p, ~(NE^!p)
                instancePtr->Swait();
                //done -> state = 2
                // nếu NE = 0 -> id = id
                // nếu NE = 1 -> id++
                break;
            default:
                instancePtr->StateMachine = 0;
                break;

        }
        
        
    }
    else {
        instancePtr->isInterrupt = false;
        Serial.println("Error: Wrong type of sercom");
    }
}

void sercom::datamemset(){
    this->memory[0] = '\0';
    this->memory[1] = '\0';
    this->memory[2] = '\0';
    this->memory[3] = '\0';
    this->memory[4] = '\0';
    this->memory[5] = '\0';
    this->memory[6] = '\0';
    this->memory[7] = '\0';
    this->memory[8] = '\0';
    this->memory[9] = '\0';
    this->memory[10] = '\0';
    this->memory[11] = '\0';
    this->memory[12] = '\0';
    this->memory[13] = '\0';
    this->memory[14] = '\0';
    this->memory[15] = '\0';
    this->memory[16] = '\0';
    this->memory[17] = '\0';
    this->memory[18] = '\0';
    this->memory[19] = '\0';
    this->memory[20] = '\0';
    this->memory[21] = '\0';
    this->memory[22] = '\0';
    this->memory[23] = '\0';
    this->memory[24] = '\0';
    this->memory[25] = '\0';
    this->memory[26] = '\0';
    this->memory[27] = '\0';
    this->memory[28] = '\0';
    this->memory[29] = '\0';
    this->memory[30] = '\0';
    this->memory[31] = '\0';
    this->memory[32] = '\0';
    this->memory[33] = '\0';
    this->memory[34] = '\0';
    this->memory[35] = '\0';
    this->memory[36] = '\0';
    this->memory[37] = '\0';
    this->memory[38] = '\0';
    this->memory[39] = '\0';
    this->memory[40] = '\0';
    this->memory[41] = '\0';
    this->memory[42] = '\0';
    this->memory[43] = '\0';
    this->memory[44] = '\0';
    this->memory[45] = '\0';
    this->memory[46] = '\0';
    this->memory[47] = '\0';
    this->memory[48] = '\0';
    this->memory[49] = '\0';
    this->memory[50] = '\0';
    this->memory[51] = '\0';
    this->memory[52] = '\0';
    this->memory[53] = '\0';
    this->memory[54] = '\0';
    this->memory[55] = '\0';
    this->memory[56] = '\0';
    this->memory[57] = '\0';
    this->memory[58] = '\0';
    this->memory[59] = '\0';
    this->memory[60] = '\0';
    this->memory[61] = '\0';
    this->memory[62] = '\0';
    this->memory[63] = '\0';
    this->memory[64] = '\0';
    this->memory[65] = '\0';
    this->memory[66] = '\0';
    this->memory[67] = '\0';
    this->memory[68] = '\0';
    this->memory[69] = '\0';
    this->memory[70] = '\0';
    this->memory[71] = '\0';
    this->memory[72] = '\0';
    this->memory[73] = '\0';
    this->memory[74] = '\0';
    this->memory[75] = '\0';
    this->memory[76] = '\0';
    this->memory[77] = '\0';
    this->memory[78] = '\0';
    this->memory[79] = '\0';
    this->memory[80] = '\0';
    this->memory[81] = '\0';
    this->memory[82] = '\0';
    this->memory[83] = '\0';
    this->memory[84] = '\0';
    this->memory[85] = '\0';
    this->memory[86] = '\0';
    this->memory[87] = '\0';
    this->memory[88] = '\0';
    this->memory[89] = '\0';
    this->memory[90] = '\0';
    this->memory[91] = '\0';
    this->memory[92] = '\0';
    this->memory[93] = '\0';
    this->memory[94] = '\0';
    this->memory[95] = '\0';
    this->memory[96] = '\0';
    this->memory[97] = '\0';
    this->memory[98] = '\0';
    this->memory[99] = '\0';
    this->isMemset = 1;
  
}