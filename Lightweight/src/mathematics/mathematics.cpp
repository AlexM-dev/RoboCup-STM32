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
		V = 0.03; // cm/msec
		curI = 0;
		a = 90;
		r = 20; 
	
		///
		offsetX = -50; 
		offsetY = 60;
		startX = -50;
		startY = 0;
		side = 1;
		///
}

int32_t mathematics::getFormatedAngle(int32_t ang){
  while(ang > 180)
    ang-=360;
  while(ang < -180)
    ang+=360;
  return ang;
}

void mathematics::setAngle(int32_t angleT) {
	angleT = getFormatedAngle(angleT);
	//m_angle = abs(float(180 - abs(float(angleT)))) * angleT / abs(float(angleT));
	m_angle = angleT;
}

void mathematics::setSpeed(int32_t speedT) {
	m_speed = speedT;
}

void mathematics::setVector(int32_t angleT, int32_t speedT) {
	angleT = getFormatedAngle(angleT);
	//m_angle = abs(float(180 - abs(float(angleT)))) * angleT / abs(float(angleT));
	m_angle = angleT;
	m_speed = speedT;
}

void mathematics::setPeriod(uint32_t period) {
	m_period = period;
}

uint32_t mathematics::getPeriod( void ) {
	return m_period;
}


int32_t mathematics::getAngle( void ) {
	return m_angle;
}

int32_t mathematics::getSpeed( void ) {
	return m_speed;
}

void mathematics::addVector(int32_t angleT, int32_t speedT) {
	if (getSpeed() + speedT > getPeriod()) {
		setSpeed(getSpeed() * (float(getPeriod()) / (getSpeed() + speedT)));
		speedT = speedT * (float(getPeriod()) / (getSpeed() + speedT));
	}
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
	if (maxSpeed >= m_period)
		maxSpeed = m_period;
	/*volatile int loas = time_service::getCurTime() - time_;
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
			if (curspeed >= period) curspeed = period;
			if (maxSpeed < curspeed)curspeed = maxSpeed;
		}
		if (curspeed == 0)curspeed++;
	}
	else if(maxSpeed < curspeed)
	{
		if (curspeed < 0)
		{
			curspeed-=ACCELERATION*(time_service::getCurTime() - time_);
			if (curspeed <= -period) curspeed = -period;
			if (maxSpeed > curspeed) curspeed = maxSpeed;
		}	
		else if (curspeed > 0)
		{
				curspeed-=ACCELERATION*(time_service::getCurTime() - time_);
				if (maxSpeed > curspeed) curspeed = maxSpeed;
		}
		if (curspeed == 0) curspeed--;
	}
	time_ = time_service::getCurTime();	*/
	
	sp1 = sin((angle + 45) / DEGREE_TO_RADIAN) * maxSpeed;
	sp2 = sin((angle + 135) / DEGREE_TO_RADIAN) * maxSpeed;
	sp3 = sin((angle - 135) / DEGREE_TO_RADIAN) * maxSpeed;
	sp4 = sin((angle - 45) / DEGREE_TO_RADIAN) * maxSpeed;
}

void mathematics::getVecFromTr(double x, double y, unsigned long t, int32_t &sp, int32_t &ang) {
	double S = V * t;
	
	if (startY < offsetY - r) { //Robot on the line
		startY+=S;
		startX+=0;
		//a = 90;
	} else { //Robot on the cercle
		a -= (S * 180) / (pi * r);
		if (a <= 0) {
			startX = r * side + offsetX;
			startY = f(r) + offsetY;
		} else {
			startX = r * cos(a / 57.3) * side + offsetX;
			startY = f(r * cos(a / 57.3)) + offsetY;
		}
	}

	ang = 90 + atan2(y - startY, x - startX) * 57.3;
	sp = 100 * sqrt(pow(startY - y, 2) + pow(startX - x, 2));
	if (sp > getPeriod() * 0.6)
		sp = getPeriod() * 0.6;
	//updateStartDot(newX, newY);
}

void mathematics::setStartDot(double x, double y) {
	a = 90;
	if (x < 0) {
		side = 1;
		offsetX = -60; 
		offsetY = 60;
	} else {
		side = -1;
		offsetX = 60;
		offsetY = 60;
	}
	
	if (y < offsetY - r) {
		startY = y;
		startX = offsetX;
	} else {
		startY = y;
		startX = g(y) * side + offsetX;
	}
		/*offsetX = -60; 
		offsetY = 60;
		startX = -60;
		startY = y;
		side = 1;*/
	
}

bool mathematics::isArrived(double x, double y) {
	if (sqrt(pow(f(r) + offsetY - y, 2) + pow(r * side + offsetX - x, 2)) < 10) {
		return 1;
	} else {
		return 0;
	}
}

void mathematics::getVecToPoint(double x, double y, double tX, double tY, int32_t &sp, int32_t &ang) {
	ang = 90 + atan2(y - tY, x - tX) * 57.3;
	sp = 100 * sqrt(pow(tY - y, 2) + pow(tX - x, 2));
}

void mathematics::getAngleToPoint(double x, double y, double tX, double tY, int32_t &ang) {
	ang = 90 + atan2(y - tY, x - tX) * 57.3;
}

double mathematics::f(double x) { //Cercle
	return -(r - sqrt(-(x * x - 2 * x * r)));
}

double mathematics::g(double x) { // f-1(x)
	return -(-r + sqrt(-2 * x * r - x * x));
}

int mathematics::max(int a, int b) {
	return a > b ? a : b;
}

int mathematics::min(int a, int b) {
	return a > b ? b : a;
}
