#include "adc.h"

Adc::Adc(ADC_TypeDef* ADCx,
				 uint8_t numberOfChannels,
				uint8_t curChannel,          
			 	 uint8_t ADC_Channel_x,
				 uint32_t RCC_APB2Periph_ADCx,
				 Pin &sig):m_sig(sig)
{
	m_numberOfChannels = numberOfChannels;
	m_ADCx = ADCx;
	m_curChannel = curChannel;
	m_ADC_Channel_x = ADC_Channel_x;
	m_RCC_APB2Periph_ADCx = RCC_APB2Periph_ADCx;
    //m_numberOfChannels
    cur = 1;
}

void Adc::adcInit()
{
	m_sig.pinInit();
	RCC_APB2PeriphClockCmd(m_RCC_APB2Periph_ADCx, ENABLE);
	ADC_CommonInitTypeDef cADC;
	cADC.ADC_Mode = ADC_Mode_Independent;
	cADC.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	cADC.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	cADC.ADC_Prescaler = ADC_Prescaler_Div2;
	
	ADC_CommonInit(&cADC);
}

void Adc::startAdc()
{
		ADC_InitTypeDef adc;
		adc.ADC_Resolution = ADC_Resolution_12b; 
		adc.ADC_ScanConvMode = ENABLE;
		adc.ADC_ContinuousConvMode = ENABLE;
		adc.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	  adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
		adc.ADC_DataAlign = ADC_DataAlign_Right;
		adc.ADC_NbrOfConversion = m_numberOfChannels;
		
		ADC_Init(m_ADCx, &adc);
		ADC_Cmd(m_ADCx, ENABLE);
}
void Adc::sendMeChannel(uint8_t chan)
{
    channel[cur] = chan;
    cur++;
}
void Adc::setChannel()
{
    for(uint8_t i = 1; i<=m_numberOfChannels; i++)
    {
	//ADC_EOCOnEachRegularChannelCmd(m_ADCx, ENABLE);
			ADC_RegularChannelConfig(m_ADCx, channel[i], i, ADC_SampleTime_56Cycles);
//	ADC_RegularChannelConfig(ADC3, , 2, ADC_SampleTime_56Cycles);
    }
}

ADC_TypeDef* Adc::getAdc()
{
	return m_ADCx;
}

void Adc::adcDmaInit()
{
	ADC_DMARequestAfterLastTransferCmd(m_ADCx, ENABLE);
	ADC_EOCOnEachRegularChannelCmd(m_ADCx, ENABLE);
	ADC_DMACmd(m_ADCx, ENABLE);
	ADC_SoftwareStartConv(m_ADCx);
}
