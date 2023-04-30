#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include <usart2.h>
#include <math.h>

class Camera
{
	public:
		Camera(Pin &tx, Pin &rx);
		void readData();
		void setGyroAng(int angle);
		int getCamDist();
		int getCamAngle();
		int getAnotherCamDist();
		int getAnotherCamAngle();
		int getFormatedAngle(int ang);
		bool canSee();
		void changeSide( void );
		int getX();
		int getY(); //ONLY FOR GK. IN ANY CASE USE getCamAngle() and getAnotherCamAngle()
		int getGK();
	private:
		char m_temp;
		int gyroAng;
		char m_str;
		int8_t koef;
		bool gotSomething;
		int8_t sign;
		Pin m_tx, m_rx;
		int8_t m_line;
		volatile int data[6], dataOld[6];
		bool goal;
		int crc8(int* data, int len);
		int t0, t2;
		bool yCanSee, bCanSee;
		int getYGK();
		int getBGK();
		int getYAngle();
		int getBAngle();
		int getYDist();
		int getBDist();
};
