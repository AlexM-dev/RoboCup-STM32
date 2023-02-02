#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include <usart1.h>
#include <math.h>
#include <time_service.h>
#include <mpu9250.h>

#define MAX_ROTATION_K 2000
#define MIN_ROTATION_K 200
#define ROT_K 50
#define ROT_D 40
#define ROT_I 0.0000

#define ACCELERATION 25

class gyro
{
	public:
		gyro(Pin &tx, Pin &rx);
		void read();
		float getDist();
		int getAngle();
		bool isCanSee();
	  void setRotationByGyro();
		void setRotation(int a);
		void setMaxSpeed(int a);
		int getTargetRobotAngle();
		int getDev();
		int getMaxSpeed();
		int getDevFromTarget();
	private:
		Pin m_tx, m_rx;
		int targetAngle, zeroAngle;
		unsigned long rTime;
		int rAngle;
		int oldErr;
		int rotationK;
		int maxSpeed;
		double I;
		int getFormatedAngle(int ang);
		uint64_t time_;
};


