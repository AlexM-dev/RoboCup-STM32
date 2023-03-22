#include "motion_control.h"

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3 

robotMotion::robotMotion(Motor &m1,
                         Motor &m2,
                         Motor &m3,
                         Motor &m4):
                         m_m1(m1),
                         m_m2(m2),
                         m_m3(m3),
                         m_m4(m4)
{
	
}

void robotMotion::robotInit()
{
		m_m1.motorInit();
		m_m2.motorInit();
		m_m3.motorInit();
		m_m4.motorInit();
}

void robotMotion::move(int64_t m_speed1, int64_t m_speed2, int64_t m_speed3, int64_t m_speed4, uint32_t period)
{    
//    m_m1.go(m_speed1);      first robot
//    m_m3.go(m_speed2);
//    m_m2.go(m_speed3); 
//    m_m4.go(m_speed4);
		m_m1.go(m_speed4, period);
		m_m2.go(m_speed1, period);      
    m_m3.go(m_speed2, period);
    m_m4.go(-m_speed3, period);
}
