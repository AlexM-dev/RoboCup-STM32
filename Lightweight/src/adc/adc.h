#pragma once
#include "project_config.h"
#include "pins_setup.h"

class Adc
{
	public:
		Adc(ADC_TypeDef* ADCx,
				uint8_t numberOfChannels,
				uint8_t curChannel,
				uint8_t ADC_Channel_x,
				uint32_t RCC_APB2Periph_ADCx,
				Pin &sig);
		void adcInit();
		void startAdc();
		void setChannel();
		void sendMeChannel(uint8_t chan);
		ADC_TypeDef* getAdc();
		void adcDmaInit();
	private:
		ADC_TypeDef* m_ADCx;
		uint8_t m_numberOfChannels;
		uint8_t m_curChannel;
	  uint8_t m_ADC_Channel_x;
		uint32_t m_RCC_APB2Periph_ADCx;
		Pin m_sig;
		uint8_t channel[50];
		uint8_t cur;
};
