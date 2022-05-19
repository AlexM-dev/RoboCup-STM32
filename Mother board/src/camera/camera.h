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
		int getYAngle();
		int getBAngle();
		int getYDist();
		int getBDist();
		int getCamDist();
		int getCamAngle();
		int getAnotherCamDist();
		int getAnotherCamAngle();
		int getFormatedAngle(int ang);
		int getX();
		int getY(); //ONLY FOR GK. IN ANY CASE USE getCamAngle() and getAnotherCamAngle()
	private:
		char m_temp;
		char m_str;
		int8_t koef;
		bool gotSomething;
		int8_t sign;
		Pin m_tx, m_rx;
		int8_t m_line;
		int data[4];
		int goal;
		int crc8(int* data, int len);
};
