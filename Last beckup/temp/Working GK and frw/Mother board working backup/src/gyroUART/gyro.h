#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include <usart1.h>

#define MAX_ROTATION_K 2000
#define MIN_ROTATION_K 200
#define ROT_K 30
#define ROT_D 40
#define ROT_I 0


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
		int getTargetRobotAngle();
	private:
		Pin m_tx, m_rx;
		int targetAngle, zeroAngle;
		int rAngle;
		int oldErr;
		int rotationK;
};


