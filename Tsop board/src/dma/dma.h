#pragma once
#include "project_config.h"
#include "adc.h"
class Dma
{
	public:
		Dma(uint32_t RCC_AHB1Periph_DMAx, Adc &adcx, DMA_Channel_TypeDef* dma_channelx, uint32_t bufferSize);
		void dmaInit(/*DMA_Channel_TypeDef* dma_channelx, uint32_t bufferSize*/);
		void adcInitInDma();
		uint32_t dataReturn(uint8_t n);
	private:
		Adc m_adcx;
		DMA_Channel_TypeDef* m_dma_channelx;
		volatile uint16_t m_adcData[2];
		uint32_t m_bufferSize;
		volatile uint16_t ADCBuffer[2];
		uint32_t m_RCC_AHBPeriph_DMAx;
	
};
