#include "adc.h"

Adc::Adc(ADC_TypeDef* ADCx,
				 uint8_t numberOfChannels,
				 uint8_t curChannel,          
				 uint32_t RCC_APB2Periph_ADCx,
				 Pin &sig):m_sig(sig)
{
	m_numberOfChannels = numberOfChannels;
	m_ADCx = ADCx;
	m_curChannel = curChannel;
	//m_ADC_Channel_x = ADC_Channel_x;
	m_RCC_APB2Periph_ADCx = RCC_APB2Periph_ADCx;
    //m_numberOfChannels
    cur = 1;
	channel[1] = ADC_Channel_0;
	channel[2] = ADC_Channel_3;
}

void Adc::adcInit()
{
	//m_sig.pinInit();
	//RCC_APB2PeriphClockCmd(m_RCC_APB2Periph_ADCx, ENABLE);
//	ADC_CommonInitTypeDef cADC;
//	cADC.ADC_Mode = ADC_Mode_Independent;
//	cADC.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
//	cADC.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
//	cADC.ADC_Prescaler = ADC_Prescaler_Div2;
//	
//	ADC_CommonInit(&cADC);
}

void Adc::startAdc()
{
		RCC_APB2PeriphClockCmd(m_RCC_APB2Periph_ADCx, ENABLE);
		ADC_InitTypeDef ADC_InitStructure;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_NbrOfChannel = m_numberOfChannels;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_Init(m_ADCx, &ADC_InitStructure);
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);
//    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_7Cycles5);
//    ADC_Cmd(ADC1 , ENABLE ) ;
//    ADC_DMACmd(ADC1 , ENABLE ) ;
//    ADC_ResetCalibration(ADC1);
//		ADC_InitTypeDef adc;	
//		adc.ADC_Mode = ADC_Mode_Independent;
//    adc.ADC_ScanConvMode = ENABLE;
//    adc.ADC_ContinuousConvMode = ENABLE;  // we work in continuous sampling mode
//    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//    adc.ADC_DataAlign = ADC_DataAlign_Right;
//    adc.ADC_NbrOfChannel = m_numberOfChannels;

//		ADC_Init(m_ADCx, &adc);
//		ADC_Cmd(m_ADCx, ENABLE);
//		ADC_SoftwareStartConvCmd(m_ADCx, ENABLE);
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
//	ADC_RegularChannelConfig(m_ADCx, channel[1], 1, ADC_SampleTime_7Cycles5);
//	ADC_RegularChannelConfig(m_ADCx, channel[2], 2, ADC_SampleTime_7Cycles5);
//	ADC_RegularChannelConfig(ADC3, , 2, ADC_SampleTime_56Cycles);
			ADC_RegularChannelConfig(m_ADCx, channel[i], i, ADC_SampleTime_7Cycles5);
			//ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_7Cycles5);
		}
    ADC_Cmd(m_ADCx , ENABLE ) ;
    ADC_DMACmd(m_ADCx , ENABLE ) ;
		ADC_SoftwareStartConvCmd(m_ADCx, ENABLE);
//    }
}

ADC_TypeDef* Adc::getAdc()
{
	return m_ADCx;
}

void Adc::adcDmaInit()
{
	//ADC_DMARequestAfterLastTransferCmd(m_ADCx, ENABLE);
	//ADC_EOCOnEachRegularChannelCmd(m_ADCx, ENABLE);
	//ADC_DMACmd(m_ADCx, ENABLE);
	//ADC_SoftwareStartConv(m_ADCx);
}
