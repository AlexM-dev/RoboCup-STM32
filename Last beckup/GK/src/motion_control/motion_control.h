#pragma once
#include <project_config.h>
#include "motor_control.h"
#include "mathematics.h"
#include "ballDetection.h"
class robotMotion
{
    public:
        robotMotion(Motor &m1,
                      Motor &m2,
                      Motor &m3,
                      Motor &m4);
        void robotInit();
        void move(int64_t m_speed1, int64_t m_speed2, int64_t m_speed3, int64_t m_speed4);
    private:
        Motor &m_m1; 
        Motor &m_m2;
        Motor &m_m3;
        Motor &m_m4;
        //mathematics m_math;
				//ballDetection m_ball;
    
};

