#include "softI2C.h"

softI2C::softI2C(Pin &sclPin,
                 Pin &sdaPin):
                 m_sclPin(sclPin),
                 m_sdaPin(sdaPin)
{
    sdaHigh();
    sclHigh();
}

bool softI2C::sclRead()
{
   bool a =(m_sclPin.getGPIOx()->IDR & m_sclPin.getPinNumber());
   return a;
}

bool softI2C::sdaRead()
{
   return (m_sdaPin.getGPIOx()->IDR & m_sdaPin.getPinNumber()); 
}

void softI2C::sdaHigh()
{
    m_sdaPin.setBit();
}    

void softI2C::sdaLow()
{
    m_sdaPin.resetBit();
}

void softI2C::sclHigh()
{
    m_sclPin.setBit();
}  

void softI2C::sclLow()
{
    m_sclPin.resetBit();
}

void softI2C::softI2CInit()
{
   m_sclPin.pinInit();
   m_sdaPin.pinInit(); 
}

void softI2C::delay()
{
    volatile uint8_t i = 50;
    while(i!=0)
    {
        i--;
    }
}
bool softI2C::generateStart()
{
    sdaHigh();
    sclHigh();
    delay();
    if (!sdaRead())return false;
    sdaLow();
    delay();
    if (sdaRead())return false;
    sclLow();//softI2C::
    delay();
    return true;
}

bool softI2C::readAch()
{
    sclLow();
    delay();
    sclHigh();
    delay();
    if(sdaRead())
    {
        sclLow();
        delay();
        return false;
    }

    sclLow();
    delay();
    return true;

}

void softI2C::sendAch()
{
    sclLow();
    delay();
    sdaLow();
    delay();
    sclHigh();
    delay();
    sclLow();
    delay();
}

void softI2C::sendNach()
{
    sclLow();
    delay();
//    sdaLow();
//    delay();
    sdaHigh();
    delay();
    sclHigh();
    delay();
    sclLow();
    delay();
//    sdaLow();
//    delay();
}
bool softI2C::generateStop()
{
//    if(arch()){
        sclLow();
        delay();
        sdaLow();
        delay();
        sclHigh();
        delay();
        sdaHigh();
        delay();
    //if(!sdaRead())return false;
    return true;
//    }
//    else
//    {
//        sclHigh();
//        delay();
//        sdaHigh();
//        delay();        
//        return false;
//    }
}

void softI2C::writeBit(bool bit)
{
    sclLow();
    delay();
    if(bit == 0)
    {
        sdaLow();
        delay();
//        sclHigh();
//        delay();
//        sclLow();
//        delay();
    }
    else
    {
        sdaHigh();
        delay();
    }
        sclHigh();
        delay();
//        sclLow();
//        delay();
//        sdaLow();
//        delay();
    
}
uint32_t softI2C::read()
{
    sdaHigh();
    //delay();
    
    uint8_t number =0;
    for(int8_t i = 7; i >= 0; i--) 
    {
        number<<=1;
        sclLow();
        delay();
        sclHigh();
        delay();
        

        if(sdaRead())
        {
            number|= 0x01;
        }

    }
    sclLow();
    //sendAch();
    return number;
}

bool softI2C::send(uint8_t package)
{
    for(uint8_t i = 0x80; i > 0x00; i >>= 1)
    {
        if(package & i)
        {
            writeBit(1);
        }
        else
        {
            writeBit(0);
        }
    }
    sclLow();
    delay();
    return 1;
//    if(readAch())
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }
}
