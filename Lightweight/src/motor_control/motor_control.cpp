#include "motor_control.h"
#include "time_service.h"

#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4

#define ACCELERATION 60

Motor::Motor(Pin &motorPinIn1,
             Pin &motorPinIn2):
             m_motorPinIn1(motorPinIn1),
             m_motorPinIn2(motorPinIn2)
{
}
void Motor::motorInit()
{
		curspeed = 0;
		m_accel = 0;
    m_motorPinIn1.pinInit();
		m_motorPinIn2.pinInit();
		m_motorPinIn1.pwmInit();
		m_motorPinIn2.pwmInit();
		time_ = 0;
		
}
void Motor::go(int32_t speed)
{	
	if(speed > 4094)speed = 4094;
	else if (speed < -4094) speed = -4094;
	
	if(speed > 100)
	{
		m_motorPinIn1.pwm(4094);
		m_motorPinIn2.pwm(4094 - speed);
	}
	else if (speed < -100)
	{
		m_motorPinIn1.pwm(4094 + speed);
		m_motorPinIn2.pwm(4094);
	}
	else
	{
		m_motorPinIn1.pwm(4094);
		m_motorPinIn2.pwm(4094);
	}
}

