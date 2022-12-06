#pragma once
#include <project_config.h>
#include <IRlocator.h>
#include <math.h>
#include <time_service.h>
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
    private:
        int32_t speed[4];
				int32_t mAngle[4];
				uint64_t time_;
				int32_t curspeed;
				int32_t m_speed;
				int32_t m_angle;
};


