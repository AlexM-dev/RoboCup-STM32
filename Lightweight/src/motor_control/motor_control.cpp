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
void Motor::go(int32_t speed, uint32_t period)
{	
	int64_t tempPeriod = int64_t(period) - 1;
	if(speed > tempPeriod)speed = tempPeriod;
	else if (speed < -tempPeriod) speed = -tempPeriod;
	
	if(speed > 0)
	{
		m_motorPinIn1.pwm(tempPeriod);
		m_motorPinIn2.pwm(tempPeriod - speed);
	}
	else if (speed < 0)
	{
		m_motorPinIn1.pwm(tempPeriod + speed);
		m_motorPinIn2.pwm(tempPeriod);
	}
	else
	{
		m_motorPinIn1.pwm(tempPeriod);
		m_motorPinIn2.pwm(tempPeriod);
	}
}

/*void Motor::go(int32_t speed, uint32_t period)
{	
	if(speed > period - 1)speed = period - 1;
	else if (speed < -(int64_t(period) - 1)) speed = -(period - 1);
	
	if(speed > 100)
	{
		m_motorPinIn1.pwm(period - 1);
		m_motorPinIn2.pwm(period - 1 - speed);
	}
	else if (speed < -100)
	{
		m_motorPinIn1.pwm(period - 1 + speed);
		m_motorPinIn2.pwm(period - 1);
	}
	else
	{
		m_motorPinIn1.pwm(period - 1);
		m_motorPinIn2.pwm(period - 1);
	}
}*/

