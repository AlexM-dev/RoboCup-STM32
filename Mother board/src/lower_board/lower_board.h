#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include <usart3.h>
#include <stm32f407_sysFunc.h>
class lowerBoard
{
	public:
		lowerBoard(Pin &tx, Pin &rx);
		void read();
		int getDist();
		int getAngle();
		bool isCanSee();
	private:
		Pin m_tx, m_rx;
		int m_dist;
		int m_angle; 
		bool m_canSee;
		unsigned long erri; 
		int crc8(int* data, int len); 
		int getFormatedAngle(int ang);
};
