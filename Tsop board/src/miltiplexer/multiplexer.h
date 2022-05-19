#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include <dma.h>
class Multiplexer
{
	public:
		Multiplexer(Pin &stp1,
					Pin &stp2,
					Pin &stp3,
					Pin &stp4,
					Dma &dmaL,
					int32_t buffSize,
					DMA_Channel_TypeDef* channelNum);
		bool getLine();
		int32_t getPh1Value(uint8_t num);
		int32_t getPh2Value(uint8_t num);
		void setState1(uint8_t number);
		void setState2(uint8_t number);
	private:		
		Pin m_stp1;
		Pin m_stp2;
		Pin m_stp3;
		Pin m_stp4;
		//Pin m_LightValue;
		Dma m_dmaL;
		int8_t multState[16][4];
	//protected:
};
