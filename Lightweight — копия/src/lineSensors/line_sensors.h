#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_SPI.h"

class line_sensors
{
	public:
		line_sensors(unsigned int spi);
		void read();
		float getDist();
		float getAngle();
		bool isCanSee();
	private:
		float m_dist;
		float m_angle; 
		bool m_canSee;
		unsigned int m_spi;
		uint16_t m_ss;
};
