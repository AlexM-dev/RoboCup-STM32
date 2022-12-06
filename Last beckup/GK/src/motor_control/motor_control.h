#pragma once
#include "project_config.h"
#include "pins_setup.h"

class Motor 
{
    public:
        Motor(Pin &motorPinIn1,
              Pin &motorPinIn2);
        void motorInit();
        void go(int32_t speed);
    private:
        //Pin &m_motorPinEn; 
        Pin &m_motorPinIn1;
				Pin &m_motorPinIn2;
				//Pin &m_motorPinInV;
				uint32_t m_motor;
				volatile int32_t curspeed;
				uint32_t m_time;
				bool m_accel;
				uint32_t time_;
};

//Motor m1(GPIO_)
