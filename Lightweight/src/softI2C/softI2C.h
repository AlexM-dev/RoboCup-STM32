#pragma once
#include "project_config.h" 
#include "pins_setup.h"
class softI2C
{
    public:  
        softI2C(Pin &sclPin,
                Pin &sdaPin);
        void softI2CInit();  
        void sclHigh();
        void sdaHigh();
        void sclLow();
        void sdaLow();
        bool generateStart();
        bool generateStop();
        void delay();
        bool sdaRead();
        bool sclRead();
        void writeBit(bool bit);
        bool send(uint8_t package);
        bool readAch();
        uint32_t read();
        void sendAch();
        void sendNach();
    private:
        Pin m_sclPin;
        Pin m_sdaPin;
};
