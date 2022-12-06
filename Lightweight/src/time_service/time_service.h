#pragma once
#include "project_config.h"
#include <stdint.h>

namespace time_service
{
	extern volatile uint32_t systemTime;
	inline uint32_t getCurTime(void)
	{ 
		return systemTime;
	}
	
	void init();
	
	void stopTime(void);
	
	void startTime(void);
	
	void delay_ms(uint32_t delta);
}
