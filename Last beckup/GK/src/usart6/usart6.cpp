#include "usart6.h"

extern "C"
{
	void USART6_IRQHandler(void)
	{
			volatile uint8_t data = USART6->SR;

			if(data & USART_SR_RXNE){
					Usart6::rx[Usart6::m_rxCnt] = USART6->DR; 
					Usart6::m_rxCnt++;
					if (Usart6::m_rxCnt == 8)
					{
							Usart6::m_rxCnt = 0;
					}
			}
			if(data & USART_SR_TC)
			{
					USART_ClearITPendingBit(USART6, USART_IT_TC);
					if (Usart6::m_txCnt != 0)
					{
							(USART6->DR) = Usart6::tx[Usart6::m_sendCnt];
							//Uart::m_txCnt;
							Usart6::m_sendCnt++;
							if(Usart6::m_sendCnt == 8)Usart6::m_sendCnt = 0;
					}
					else{
							Usart6::fl = 1;
					}
	//time_service::systemTime++;
			}
			if(USART6->SR & USART_SR_ORE)
			{
					uint8_t a = USART6->DR;
					(void)a;
			}
	}
}
namespace Usart6{
//Uart::Uart(Pin &txPin,
//           Pin &rxPin):
//           m_txPin(txPin),
//      	     m_rxPin(rxPin)
//{
//               
//}
    volatile uint8_t tx[8];
    volatile uint8_t rx[8];
    volatile uint16_t m_rxCnt;
    volatile uint16_t m_txCnt; 
    volatile bool fl;
    volatile uint16_t m_readCnt;
    volatile uint16_t m_sendCnt;
    void usartInit()
    {
        fl = 1;
        m_txCnt = 0;
        m_rxCnt = 0;
        m_readCnt = 0;
        m_sendCnt = 0;
       // m_txPin.pinInit();
       // m_rxPin.pinInit();
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
        USART_InitTypeDef uart;
        uart.USART_BaudRate = 19200;
        uart.USART_WordLength = USART_WordLength_8b;
        uart.USART_StopBits = USART_StopBits_1;
        uart.USART_Parity = USART_Parity_No;
        uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART6, &uart);
        
        USART_ITConfig(USART6, USART_IT_TC, ENABLE);
        USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
        
        USART_Cmd(USART6, ENABLE);
        NVIC_SetPriority(USART6_IRQn, 0);
        NVIC_EnableIRQ(USART6_IRQn);
    }

    uint16_t read()
    {
      uint16_t a;  
      ENTER_CRITICAL_SECTION();  
      a = rx[m_readCnt];
      m_readCnt++;
      if (m_readCnt == 8)
      {
          m_readCnt = 0;
      }
      
      EXIT_CRITICAL_SECTION();  
      return a;      
    }

    uint16_t available()
    {
        uint16_t b;
        ENTER_CRITICAL_SECTION();
        b = m_rxCnt - m_readCnt;
        EXIT_CRITICAL_SECTION();
           // EXIT_CRITICAL_SECTION();
        return b;

    }

    void write(uint8_t byte)
    {
       ENTER_CRITICAL_SECTION();
       if (fl == 0){ 
       tx[m_txCnt] = byte;
       m_txCnt++;
       if (m_txCnt == 8)m_txCnt = 0;
       }
       else{
           fl = 0;
           (USART6->DR) = byte;
           //m_txCnt--;
       }
       EXIT_CRITICAL_SECTION();
    }

}
