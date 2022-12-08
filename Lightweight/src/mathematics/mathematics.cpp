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
		V = 0.1; // cm/msec
		S = 0;
		step = 0.1;
		curI = 0;
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
	S = S + V * t;
	if (startY > tr[0][1] + offsetY) { //Robot on the cercle
		int i = 0;
		for (;i + curI < TR_SIZE; ++i) {
			if (sqrt(pow(startX - (tr[i+curI][0] * side + offsetX), 2) + pow(startY - (tr[i+curI][1] + offsetY), 2)) > S) {
				newX = tr[i+curI][0] * side + offsetX;
				newY = tr[i+curI][1] + offsetY;
				break;
			}
		}
		curI += i;
		if(i != 0) {
			S = 0;
		}
	} else {	//Robot on the line
		newX = startX;
		newY = startY + S;
	}

	ang = atan2(newY - y, newX - x);
	sp = sqrt(pow(newY - y, 2) + pow(newX - x, 2));
	updateStartDot(newX, newY);
}

void mathematics::setStartDot(double x, double y) {
	if (x < 0) {
		side = 1;
		offsetX = -20 - 50; 
		offsetY = 80;
	} else {
		side = -1;
		offsetX = 20 + 50;
		offsetY = 80;
	}
		 
	if (y < tr[0][1] + offsetY) {
			startY = y;
		if (x < 0) {
			startX = -40;
		} else {
			startX = 40;
		}
	} else {
		int l = 0;
		int r = TR_SIZE;
		int mid;
	 
		while (l < r) {
				mid = (l + r) / 2;
		 
				if (tr[mid][1] > y) r = mid;
				else l = mid + 1;
		}
		 
		r--; 
		startX = tr[r][0] * side + offsetX;
		startY = tr[r][1] + offsetY;
	}
}

void mathematics::updateStartDot(double x, double y) {
	startX = x;
	startY = y;
}

void mathematics::setTr (void) {
	float temp[TR_SIZE][2] = {
													{ 0.0 , -20 } ,
													{ 0.0 , -19.9 } ,
													{ 0.001 , -19.8 } ,
													{ 0.002 , -19.7 } ,
													{ 0.004 , -19.6 } ,
													{ 0.006 , -19.5 } ,
													{ 0.009 , -19.4 } ,
													{ 0.012 , -19.3 } ,
													{ 0.016 , -19.2 } ,
													{ 0.02 , -19.1 } ,
													{ 0.025 , -19.0 } ,
													{ 0.03 , -18.9 } ,
													{ 0.036 , -18.8 } ,
													{ 0.042 , -18.7 } ,
													{ 0.049 , -18.6 } ,
													{ 0.056 , -18.5 } ,
													{ 0.064 , -18.4 } ,
													{ 0.072 , -18.3 } ,
													{ 0.081 , -18.2 } ,
													{ 0.09 , -18.1 } ,
													{ 0.1 , -18.0 } ,
													{ 0.111 , -17.9 } ,
													{ 0.121 , -17.8 } ,
													{ 0.133 , -17.7 } ,
													{ 0.145 , -17.6 } ,
													{ 0.157 , -17.5 } ,
													{ 0.17 , -17.4 } ,
													{ 0.183 , -17.3 } ,
													{ 0.197 , -17.2 } ,
													{ 0.211 , -17.1 } ,
													{ 0.226 , -17.0 } ,
													{ 0.242 , -16.9 } ,
													{ 0.258 , -16.8 } ,
													{ 0.274 , -16.7 } ,
													{ 0.291 , -16.6 } ,
													{ 0.309 , -16.5 } ,
													{ 0.327 , -16.4 } ,
													{ 0.345 , -16.3 } ,
													{ 0.364 , -16.2 } ,
													{ 0.384 , -16.1 } ,
													{ 0.404 , -16.0 } ,
													{ 0.425 , -15.9 } ,
													{ 0.446 , -15.8 } ,
													{ 0.468 , -15.7 } ,
													{ 0.49 , -15.6 } ,
													{ 0.513 , -15.5 } ,
													{ 0.536 , -15.4 } ,
													{ 0.56 , -15.3 } ,
													{ 0.585 , -15.2 } ,
													{ 0.61 , -15.1 } ,
													{ 0.635 , -15.0 } ,
													{ 0.661 , -14.9 } ,
													{ 0.688 , -14.8 } ,
													{ 0.715 , -14.7 } ,
													{ 0.743 , -14.6 } ,
													{ 0.771 , -14.5 } ,
													{ 0.8 , -14.4 } ,
													{ 0.829 , -14.3 } ,
													{ 0.859 , -14.2 } ,
													{ 0.89 , -14.1 } ,
													{ 0.921 , -14.0 } ,
													{ 0.953 , -13.9 } ,
													{ 0.985 , -13.8 } ,
													{ 1.018 , -13.7 } ,
													{ 1.052 , -13.6 } ,
													{ 1.086 , -13.5 } ,
													{ 1.12 , -13.4 } ,
													{ 1.156 , -13.3 } ,
													{ 1.191 , -13.2 } ,
													{ 1.228 , -13.1 } ,
													{ 1.265 , -13.0 } ,
													{ 1.303 , -12.9 } ,
													{ 1.341 , -12.8 } ,
													{ 1.38 , -12.7 } ,
													{ 1.419 , -12.6 } ,
													{ 1.46 , -12.5 } ,
													{ 1.5 , -12.4 } ,
													{ 1.542 , -12.3 } ,
													{ 1.584 , -12.2 } ,
													{ 1.626 , -12.1 } ,
													{ 1.67 , -12.0 } ,
													{ 1.714 , -11.9 } ,
													{ 1.758 , -11.8 } ,
													{ 1.804 , -11.7 } ,
													{ 1.85 , -11.6 } ,
													{ 1.896 , -11.5 } ,
													{ 1.943 , -11.4 } ,
													{ 1.991 , -11.3 } ,
													{ 2.04 , -11.2 } ,
													{ 2.089 , -11.1 } ,
													{ 2.139 , -11.0 } ,
													{ 2.19 , -10.9 } ,
													{ 2.242 , -10.8 } ,
													{ 2.294 , -10.7 } ,
													{ 2.347 , -10.6 } ,
													{ 2.4 , -10.5 } ,
													{ 2.455 , -10.4 } ,
													{ 2.51 , -10.3 } ,
													{ 2.566 , -10.2 } ,
													{ 2.622 , -10.1 } ,
													{ 2.679 , -10.0 } ,
													{ 2.738 , -9.9 } ,
													{ 2.797 , -9.8 } ,
													{ 2.856 , -9.7 } ,
													{ 2.917 , -9.6 } ,
													{ 2.978 , -9.5 } ,
													{ 3.04 , -9.4 } ,
													{ 3.103 , -9.3 } ,
													{ 3.167 , -9.2 } ,
													{ 3.231 , -9.1 } ,
													{ 3.297 , -9.0 } ,
													{ 3.363 , -8.9 } ,
													{ 3.43 , -8.8 } ,
													{ 3.498 , -8.7 } ,
													{ 3.567 , -8.6 } ,
													{ 3.637 , -8.5 } ,
													{ 3.708 , -8.4 } ,
													{ 3.779 , -8.3 } ,
													{ 3.852 , -8.2 } ,
													{ 3.925 , -8.1 } ,
													{ 4.0 , -8.0 } ,
													{ 4.075 , -7.9 } ,
													{ 4.152 , -7.8 } ,
													{ 4.229 , -7.7 } ,
													{ 4.308 , -7.6 } ,
													{ 4.388 , -7.5 } ,
													{ 4.468 , -7.4 } ,
													{ 4.55 , -7.3 } ,
													{ 4.633 , -7.2 } ,
													{ 4.716 , -7.1 } ,
													{ 4.801 , -7.0 } ,
													{ 4.887 , -6.9 } ,
													{ 4.975 , -6.8 } ,
													{ 5.063 , -6.7 } ,
													{ 5.153 , -6.6 } ,
													{ 5.244 , -6.5 } ,
													{ 5.336 , -6.4 } ,
													{ 5.429 , -6.3 } ,
													{ 5.524 , -6.2 } ,
													{ 5.62 , -6.1 } ,
													{ 5.717 , -6.0 } ,
													{ 5.816 , -5.9 } ,
													{ 5.916 , -5.8 } ,
													{ 6.016 , -5.702 } ,
													{ 6.116 , -5.604 } ,
													{ 6.216 , -5.509 } ,
													{ 6.316 , -5.414 } ,
													{ 6.416 , -5.321 } ,
													{ 6.516 , -5.229 } ,
													{ 6.616 , -5.138 } ,
													{ 6.716 , -5.049 } ,
													{ 6.816 , -4.961 } ,
													{ 6.916 , -4.874 } ,
													{ 7.016 , -4.788 } ,
													{ 7.116 , -4.703 } ,
													{ 7.216 , -4.619 } ,
													{ 7.316 , -4.537 } ,
													{ 7.416 , -4.455 } ,
													{ 7.516 , -4.375 } ,
													{ 7.616 , -4.295 } ,
													{ 7.716 , -4.217 } ,
													{ 7.816 , -4.14 } ,
													{ 7.916 , -4.063 } ,
													{ 8.016 , -3.988 } ,
													{ 8.116 , -3.914 } ,
													{ 8.216 , -3.84 } ,
													{ 8.316 , -3.768 } ,
													{ 8.416 , -3.696 } ,
													{ 8.516 , -3.626 } ,
													{ 8.616 , -3.556 } ,
													{ 8.716 , -3.487 } ,
													{ 8.816 , -3.419 } ,
													{ 8.916 , -3.352 } ,
													{ 9.016 , -3.286 } ,
													{ 9.116 , -3.221 } ,
													{ 9.216 , -3.156 } ,
													{ 9.316 , -3.093 } ,
													{ 9.416 , -3.03 } ,
													{ 9.516 , -2.968 } ,
													{ 9.616 , -2.907 } ,
													{ 9.716 , -2.847 } ,
													{ 9.816 , -2.787 } ,
													{ 9.916 , -2.728 } ,
													{ 10.016 , -2.67 } ,
													{ 10.116 , -2.613 } ,
													{ 10.216 , -2.557 } ,
													{ 10.316 , -2.501 } ,
													{ 10.416 , -2.446 } ,
													{ 10.516 , -2.392 } ,
													{ 10.616 , -2.338 } ,
													{ 10.716 , -2.285 } ,
													{ 10.816 , -2.233 } ,
													{ 10.916 , -2.182 } ,
													{ 11.016 , -2.131 } ,
													{ 11.116 , -2.081 } ,
													{ 11.216 , -2.032 } ,
													{ 11.316 , -1.984 } ,
													{ 11.416 , -1.936 } ,
													{ 11.516 , -1.889 } ,
													{ 11.616 , -1.842 } ,
													{ 11.716 , -1.796 } ,
													{ 11.816 , -1.751 } ,
													{ 11.916 , -1.707 } ,
													{ 12.016 , -1.663 } ,
													{ 12.116 , -1.62 } ,
													{ 12.216 , -1.577 } ,
													{ 12.316 , -1.535 } ,
													{ 12.416 , -1.494 } ,
													{ 12.516 , -1.453 } ,
													{ 12.616 , -1.413 } ,
													{ 12.716 , -1.374 } ,
													{ 12.816 , -1.335 } ,
													{ 12.916 , -1.297 } ,
													{ 13.016 , -1.259 } ,
													{ 13.116 , -1.222 } ,
													{ 13.216 , -1.186 } ,
													{ 13.316 , -1.15 } ,
													{ 13.416 , -1.115 } ,
													{ 13.516 , -1.08 } ,
													{ 13.616 , -1.046 } ,
													{ 13.716 , -1.013 } ,
													{ 13.816 , -0.98 } ,
													{ 13.916 , -0.948 } ,
													{ 14.016 , -0.916 } ,
													{ 14.116 , -0.885 } ,
													{ 14.216 , -0.855 } ,
													{ 14.316 , -0.825 } ,
													{ 14.416 , -0.795 } ,
													{ 14.516 , -0.767 } ,
													{ 14.616 , -0.738 } ,
													{ 14.716 , -0.711 } ,
													{ 14.816 , -0.684 } ,
													{ 14.916 , -0.657 } ,
													{ 15.016 , -0.631 } ,
													{ 15.116 , -0.606 } ,
													{ 15.216 , -0.581 } ,
													{ 15.316 , -0.556 } ,
													{ 15.416 , -0.532 } ,
													{ 15.516 , -0.509 } ,
													{ 15.616 , -0.486 } ,
													{ 15.716 , -0.464 } ,
													{ 15.816 , -0.443 } ,
													{ 15.916 , -0.421 } ,
													{ 16.016 , -0.401 } ,
													{ 16.116 , -0.381 } ,
													{ 16.216 , -0.361 } ,
													{ 16.316 , -0.342 } ,
													{ 16.416 , -0.324 } ,
													{ 16.516 , -0.306 } ,
													{ 16.616 , -0.288 } ,
													{ 16.716 , -0.271 } ,
													{ 16.816 , -0.255 } ,
													{ 16.916 , -0.239 } ,
													{ 17.016 , -0.224 } ,
													{ 17.116 , -0.209 } ,
													{ 17.216 , -0.195 } ,
													{ 17.316 , -0.181 } ,
													{ 17.416 , -0.168 } ,
													{ 17.516 , -0.155 } ,
													{ 17.616 , -0.143 } ,
													{ 17.716 , -0.131 } ,
													{ 17.816 , -0.12 } ,
													{ 17.916 , -0.109 } ,
													{ 18.016 , -0.099 } ,
													{ 18.116 , -0.089 } ,
													{ 18.216 , -0.08 } ,
													{ 18.316 , -0.071 } ,
													{ 18.416 , -0.063 } ,
													{ 18.516 , -0.055 } ,
													{ 18.616 , -0.048 } ,
													{ 18.716 , -0.041 } ,
													{ 18.816 , -0.035 } ,
													{ 18.916 , -0.029 } ,
													{ 19.016 , -0.024 } ,
													{ 19.116 , -0.02 } ,
													{ 19.216 , -0.015 } ,
													{ 19.316 , -0.012 } ,
													{ 19.416 , -0.009 } ,
													{ 19.516 , -0.006 } ,
													{ 19.616 , -0.004 } ,
													{ 19.716 , -0.002 } ,
													{ 19.816 , -0.001 } ,
													{ 20 , 0 }
													};
	for (int i = 0; i < TR_SIZE; i++) {
		tr[i][0] = temp[i][0];
		tr[i][1] = temp[i][1];
	}
}
