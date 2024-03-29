#pragma once
#include <project_config.h>
#include <IRlocator.h>
#include <math.h>
#include <time_service.h>
#define pi 3.14159
class mathematics//class for mathematics 
{
    public:
        mathematics();
				void calculateSpeed(int32_t angle, int32_t maxSpeed, int32_t &sp1, int32_t &sp2, int32_t &sp3, int32_t &sp4);//function for calculating speed
				void setAngle(int32_t angleT);
				void setSpeed(int32_t speedT);
				void setVector(int32_t angleT, int32_t speed);
				void addVector(int32_t angleT, int32_t speed);
				int32_t getAngle( void );
				int32_t getSpeed( void );
				//void setStartDotForTr1(double x, double y)
				void setPeriod( uint32_t period );
				uint32_t getPeriod( void );
				void getVecFromTr(double x, double y, unsigned long t, int32_t &sp, int32_t &ang);
				void getVecToPoint(double x, double y, double tX, double tY, int32_t &sp, int32_t &ang);
				void getAngleToPoint(double x, double y, double tX, double tY, int32_t &ang);
				void setTr( void );
				void setStartDot(double x, double y);
				bool isArrived(double x, double y);
				int max(int a, int b);
				int min(int a, int b);
    private:
				void updateStartDot(double x, double y);
        int32_t speed[4];
				int32_t mAngle[4];
				uint64_t time_;
				int32_t curspeed;
				int32_t m_speed;
				int32_t m_angle;
				int32_t m_period;
				double startX, startY;
				double offsetX, offsetY;
				int side, curI;
				double V;
				double r, a;
				double f(double x);
				double g(double y);
				int32_t getFormatedAngle(int32_t ang);
};


