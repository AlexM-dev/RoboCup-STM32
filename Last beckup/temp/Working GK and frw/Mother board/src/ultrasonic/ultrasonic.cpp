#include "ultrasonic.h"

Ultrasonic::Ultrasonic(Pin &echo, Adc &ultraAdc, Dma &ultraDma):m_echo(echo),
																																m_ultraAdc(ultraAdc),
																																m_ultraDma(ultraDma)
{
	m_echo.pinInit();
	m_ultraDma.dmaInit(DMA2_Stream0, DMA_Channel_2, 1);
	m_ultraDma.adcInitInDma();
	
}

uint32_t Ultrasonic::getDistance()
{
	return m_ultraDma.dataReturn(0);
}
