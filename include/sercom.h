#ifndef SERCOM_H
#define SERCOM_H

#include <Arduino.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <ChaCha.h>
#include <stdlib.h>
#include <PinChangeInterrupt.h>  



#define MAX_BUFFER_SIZE 128
#define MAX_KEY_SIZE 512
#define MAX_DATA_LENGTH 100


class sercom {
    private:
        ChaCha chacha;
        int8_t state;
        volatile uint8_t StateMachine;
    public:
        // int8_t state;
        uint8_t CLP;
        uint8_t DTP;
        volatile bool isSending, isReceiving;
        volatile char memory[100];
        char *data;
        volatile uint8_t DataToSend;
        volatile uint8_t dataId, charId;
        volatile bool parity;
        volatile uint8_t *portclk;
        volatile uint8_t *portdata;
        uint8_t clkpin;
        uint8_t datapin;
        static sercom* instancePtr;
        volatile bool isRecived, isNoERROR, isStarted, isSended;
        volatile bool isMemset, isInterrupt;
        
        volatile uint8_t countStart;
        volatile bool nextChar;

        volatile uint8_t MESS,START;
        
        char password[128] = "Loli is the precious thing in the world";
        int key[128];
        
        sercom(char type, uint8_t clk_port, uint8_t data_port);

        //master func
        void set_clk(long clk);
        void print();
        void clkInterrupt();
        void setSendData();
        void send(char *data);
        void setDataToCLK();
        void ResetDataToCLK();
        void setStartToCLK();

        void getMessFromSlave();

        void wait();

        static void Mcall();
        static void MRcall();

        //slave func
        static void Scall_p();
        static void Scall_n();

        void Swait();
        void SSwait();

        void receiveStart();
        void StartCallBack();
        void DataCallBack();
        void receiveData();
        void datamemset();
        void receive();


};

#endif