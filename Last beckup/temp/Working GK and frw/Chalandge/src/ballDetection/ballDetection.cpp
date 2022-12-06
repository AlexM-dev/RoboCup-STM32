#include "ballDetection.h"
#define PHOTOVALUE 1800 //700
#define BALLDATCHSENSOR 1
ballDetection::ballDetection(Dma &dmaBall):m_dmaBall(dmaBall)
{
		m_dmaBall.dmaInit(DMA2_Stream4, DMA_Channel_0, BALLDATCHSENSOR);//function for initing dma
    m_dmaBall.adcInitInDma();//function for initing adc
}
bool ballDetection::isThereABall()
{
	if(m_dmaBall.dataReturn(0)>PHOTOVALUE)
	{
		return false;
	}
	else 
	{
		return true;
	}
}



