#include "mathematics.h"
#include <math.h>

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3 
#define DEGREE_TO_RADIAN 57.3

#define ACCELERATION 70

mathematics::mathematics()
{
    mAngle[0] = 135;
		mAngle[1] = 45;
		mAngle[2] = -45;
		mAngle[3] = -135;
		m_speed = 0;
		m_angle = 0;
}

void mathematics::setAngle(int32_t angleT) {
	m_angle = angleT;
}

void mathematics::setSpeed(int32_t speedT) {
	m_speed = speedT;
}

void mathematics::setVector(int32_t angleT, int32_t speedT) {
	m_angle = angleT;
	m_speed = speedT;
}

int32_t mathematics::getAngle( void ) {
	return m_angle;
}

int32_t mathematics::getSpeed( void ) {
	return m_speed;
}

void mathematics::addVector(int32_t angleT, int32_t speedT) {
	int x1 = getSpeed() * sin(getAngle() / DEGREE_TO_RADIAN), 
	x2 = speedT * sin(angleT / DEGREE_TO_RADIAN), 
	y1 = getSpeed() * cos(getAngle() / DEGREE_TO_RADIAN), 
	y2 = speedT * cos(angleT / DEGREE_TO_RADIAN),
	x = x1 + x2,
	y = y1 + y2;
	
	setVector(atan2(double(x), double(y)) * 57.3, sqrt(abs(double(x * x + y * y))));
}

void mathematics::calculateSpeed(int32_t angle, int32_t maxSpeed, int32_t &sp1, int32_t &sp2, int32_t &sp3, int32_t &sp4)
{
	if (maxSpeed > 4000)
		maxSpeed = 4000;
	angle = -(angle + 180);
	volatile int loas = time_service::getCurTime() - time_;
	volatile int32_t s = maxSpeed;
	volatile int32_t cs = curspeed;
	
	if(maxSpeed < 20 && maxSpeed > -20)
	{
		curspeed = 0;
	}
	else if(maxSpeed > curspeed)
	{
		if (curspeed < 0)
		{
			curspeed+=ACCELERATION*(time_service::getCurTime() - time_);
			if (maxSpeed < curspeed)curspeed = maxSpeed;
		}		
		else if (curspeed > 0)
		{
			curspeed+=ACCELERATION*(time_service::getCurTime() - time_);
			if (curspeed >4096)curspeed = 4096;
			if (maxSpeed < curspeed)curspeed = maxSpeed;
		}
		if (curspeed == 0)curspeed++;
	}
	else if(maxSpeed < curspeed)
	{
		if (curspeed < 0)
		{
			curspeed-=ACCELERATION*(time_service::getCurTime() - time_);
			if (curspeed <-4096)curspeed = -4096;
			if (maxSpeed > curspeed)curspeed = maxSpeed;
		}	
		else if (curspeed > 0)
		{
				curspeed-=ACCELERATION*(time_service::getCurTime() - time_);
				if (maxSpeed > curspeed)curspeed = maxSpeed;
		}
		if (curspeed == 0)curspeed--;
	}
	time_ = time_service::getCurTime();	
	
	sp1 = sin((angle + 45) / DEGREE_TO_RADIAN) * maxSpeed;
	sp2 = sin((angle + 135) / DEGREE_TO_RADIAN) * maxSpeed;
	sp3 = sin((angle - 135) / DEGREE_TO_RADIAN) * maxSpeed;
	sp4 = sin((angle - 45) / DEGREE_TO_RADIAN) * maxSpeed;
}

void mathematics::getVecFromTr(double x, double y, unsigned long t, int32_t &sp, int32_t &ang) {
	double newX, newY;
	double S = V * t;
	
	if (startY < offsetY - r) { //Robot on the line
		newY = startY + S;
		newX = startX;
		//a = 90;
	} else { //Robot on the cercle
		a -= (S * 180) / (pi * r);
		if (a <= 0) {
			newX = r * side + offsetX;
			newY = f(r) + offsetY;
		} else {
			newX = r * cos(a / 57.3) * side + offsetX;
			newY = f(r * cos(a / 57.3)) + offsetY;
		}
	}
	
	//FOR TEST//
	newX = 0;
	newY = 0;

	ang = atan2(newY - y, newX - x) * 57.3;
	sp = 100 * sqrt(pow(newY - y, 2) + pow(newX - x, 2));
	if (sp > 3000)
		sp = 3000;
	updateStartDot(newX, newY);
}

void mathematics::setStartDot(double x, double y) {
	a = 90;
	if (x < 0) {
		side = 1;
		offsetX = -70; 
		offsetY = 80;
	} else {
		side = -1;
		offsetX = 70;
		offsetY = 80;
	}
	
	if (y <  offsetY - r) {
		startY = y;
		startX = offsetX + r * side;
	} else {
		startY = y;
		startX = g(y) * side + offsetX;
	}
	
}

void mathematics::updateStartDot(double x, double y) {
	startX = x;
	startY = y;
}

double mathematics::f(double x) { //Cercle
	return -(r - sqrt(-(x * x - 2 * x * r)));
}

double mathematics::g(double x) { // f-1(x)
	return -(-r + sqrt(-2 * x * r - x * x));
}