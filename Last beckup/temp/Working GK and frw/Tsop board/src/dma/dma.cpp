#include "dma.h"

Dma::Dma(uint32_t RCC_AHBPeriph_DMAx,Adc &adcx, DMA_Channel_TypeDef* dma_channelx, uint32_t bufferSize):m_adcx(adcx)
{
	m_RCC_AHBPeriph_DMAx = RCC_AHBPeriph_DMAx;
	m_dma_channelx = dma_channelx;
	m_bufferSize = bufferSize;
}

	void Dma::adcInitInDma()
	{
		m_adcx.adcInit();

		m_adcx.setChannel();
    m_adcx.startAdc();
		m_adcx.adcDmaInit();
		ADCBuffer[0] = 0xAAAA;
		ADCBuffer[1] = 0xAAAA;
		m_adcData[0] = 0xAAAA;
		m_adcData[1] = 0xAAAA;
	}
void Dma::dmaInit(/*DMA_Channel_TypeDef* dma_channelx, uint32_t bufferSize*/)
{
	RCC_AHBPeriphClockCmd(m_RCC_AHBPeriph_DMAx , ENABLE );
	DMA_InitTypeDef DMA_InitStructure;
//	RCC_AHBPeriphClockCmd(m_RCC_AHBPeriph_DMAx, ENABLE);
//	DMA_InitTypeDef DMA_InitStructure;
//	DMA_DeInit(dma_channelx);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(m_adcx.getAdc()->DR);
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)m_adcData;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//	DMA_InitStructure.DMA_BufferSize = bufferSize;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(dma_channelx, &DMA_InitStructure);
//	DMA_Cmd(dma_channelx, ENABLE);
	DMA_InitStructure.DMA_BufferSize = m_bufferSize;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)m_adcData;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&m_adcx.getAdc()->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(m_dma_channelx, &DMA_InitStructure);
	DMA_Cmd(m_dma_channelx , ENABLE ) ;
}

uint32_t Dma::dataReturn(uint8_t n)
{
	return m_adcData[n];
}
