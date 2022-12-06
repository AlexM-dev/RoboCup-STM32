#pragma once
#include "project_config.h"
#include "adc.h"
class Dma
{
	public:
		Dma(uint32_t RCC_AHB1Periph_DMAx, Adc &adcx);
		void dmaInit(DMA_Stream_TypeDef* dmax_streamx, uint32_t dma_channelx, uint32_t bufferSize);
		void adcInitInDma();
		uint16_t dataReturn(uint8_t n);
	private:
		Adc m_adcx;
		uint16_t m_adcData[5];
		uint32_t m_RCC_AHB1Periph_DMAx;
	
};
