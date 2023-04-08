#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include <Usart6.h>
#include <stm32f407_sysFunc.h>
class lowerBoard
{
	public:
		lowerBoard(Pin &tx, Pin &rx);
		void read();
		void setGyroAng(int angle);
		int getDist();
		int getAngle();
	  int getLocalAngle();
		bool isCanSee();
	private:
		Pin m_tx, m_rx;
		int gyroAng;
		int m_dist;
		int m_angle; 
		bool m_canSee;
		int count;
		int getFormatedAngle(int ang);
		unsigned long erri; 
		int crc8(int* data, int len); 
};
