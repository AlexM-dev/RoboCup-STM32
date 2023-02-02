#pragma once
#include <project_config.h>
#include <adc.h>
#include <dma.h>
#include <pins_setup.h>
class Ultrasonic
{
	public:
		Ultrasonic(Pin &echo, Adc &ultraAdc, Dma &ultraDma);
		uint32_t getDistance();
	private:
		Pin m_echo;
		Adc m_ultraAdc;
		Dma m_ultraDma;
};
