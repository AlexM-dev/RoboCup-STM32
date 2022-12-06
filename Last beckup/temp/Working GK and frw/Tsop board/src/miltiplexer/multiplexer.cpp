#include "multiplexer.h"

Multiplexer::Multiplexer(Pin &stp1,
												 Pin &stp2,
												 Pin &stp3,
												 Pin &stp4,
												 Dma &dmaL,
												 int32_t buffSize,
												 DMA_Channel_TypeDef* channelNum):
												 m_stp1(stp1),
												 m_stp2(stp2),
												 m_stp3(stp3),
												 m_stp4(stp4),
												 m_dmaL(dmaL)
												 
{
	m_stp1.pinInit();
	m_stp2.pinInit();
	m_stp3.pinInit();
	m_stp4.pinInit();
	//m_dmaL.adcInitInDma();
	m_dmaL.dmaInit(/*channelNum, buffSize*/);

}
void Multiplexer::setState1(uint8_t number)
{
	bool	multState[16][4] = {{0,0,0,0},
														{0,0,0,1},
														{0,0,1,0},
														{0,0,1,1},
														{0,1,0,0},
														{0,1,0,1},
														{0,1,1,0},
														{0,1,1,1},
														{1,0,0,0},
														{1,0,0,1},
														{1,0,1,0},
														{1,0,1,1},
														{1,1,0,0},
														{1,1,0,1},
														{1,1,1,0},
														{1,1,1,1}};
	if(multState[number][0] == 0)m_stp1.resetBit();
	else m_stp1.setBit();
	if(multState[number][1] == 0)m_stp2.resetBit();
	else m_stp2.setBit();
	if(multState[number][2] == 0)m_stp3.resetBit();
	else m_stp3.setBit();			
	if(multState[number][3] == 0)m_stp4.resetBit();
	else m_stp4.setBit();										
}
void Multiplexer::setState2(uint8_t number)
{
	bool	multState[16][4] = {{0,0,0,0},
														{0,0,0,1},
														{0,0,1,0},
														{0,0,1,1},
														{0,1,0,0},
														{0,1,0,1},
														{0,1,1,0},
														{0,1,1,1},
														{1,0,0,0},
														{1,0,0,1},
														{1,0,1,0},
														{1,0,1,1},
														{1,1,0,0},
														{1,1,0,1},
														{1,1,1,0},
														{1,1,1,1}};
	if(multState[number][0] == 0)m_stp1.resetBit();
	else m_stp1.setBit();
	if(multState[number][1] == 0)m_stp2.resetBit();
	else m_stp2.setBit();
	if(multState[number][2] == 0)m_stp3.resetBit();
	else m_stp3.setBit();	
	if(multState[number][3] == 0)m_stp4.resetBit();
	else m_stp4.setBit();									
}

bool Multiplexer::getLine()
{
	int ifTwo = 0;
	for(int i = 0; i < 16; i++)
	{
		int value = 0;
		setState2(i);
		value = m_dmaL.dataReturn(1);
		if(value > 3000)
		{
			ifTwo++;
		}
		value = 0;
		setState1(i);
		value = m_dmaL.dataReturn(0);
		if(value > 3000)
		{
			ifTwo++;
		}
	}
	if(ifTwo > 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

int32_t Multiplexer::getPh1Value(uint8_t num)
{
	int32_t value;
	setState1(num);
	value = m_dmaL.dataReturn(0);
	return value;
}
int32_t Multiplexer::getPh2Value(uint8_t num)
{
	int32_t value;
	setState2(num);
	value = m_dmaL.dataReturn(1);
	return value;
}
