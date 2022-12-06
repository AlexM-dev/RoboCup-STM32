#include "usart3.h"

extern "C"
{
	void USART3_IRQHandler(void)
	{
			volatile uint8_t data = USART3->SR;

			if(data & USART_SR_RXNE){
					Usart3::rx[Usart3::m_rxCnt] = USART3->DR; 
					Usart3::m_rxCnt++;
					if (Usart3::m_rxCnt == 16)
					{
							Usart3::m_rxCnt = 0;
					}
			}
			if(data & USART_SR_TC)
			{
					USART_ClearITPendingBit(USART3, USART_IT_TC);
					if (Usart3::m_txCnt != 0)
					{
							(USART3->DR) = Usart3::tx[Usart3::m_sendCnt];
							//Uart::m_txCnt;
							Usart3::m_sendCnt++;
							if(Usart3::m_sendCnt == 16)Usart3::m_sendCnt = 0;
					}
					else{
							Usart3::fl = 1;
					}
	//time_service::systemTime++;
			}
			if(USART3->SR & USART_SR_ORE)
			{
					uint8_t a = USART3->DR;
					(void)a;
			}
	}
}
namespace Usart3{
//Uart::Uart(Pin &txPin,
//           Pin &rxPin):
//           m_txPin(txPin),
//      	     m_rxPin(rxPin)
//{
//               
//}
    volatile uint8_t tx[16];
    volatile uint8_t rx[16];
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
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        USART_InitTypeDef uart;
        uart.USART_BaudRate = 460800;
        uart.USART_WordLength = USART_WordLength_8b;
        uart.USART_StopBits = USART_StopBits_1;
        uart.USART_Parity = USART_Parity_No;
        uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART3, &uart);
        
        USART_ITConfig(USART3, USART_IT_TC, ENABLE);
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        
        USART_Cmd(USART3, ENABLE);
        NVIC_SetPriority(USART3_IRQn, 0);
        NVIC_EnableIRQ(USART3_IRQn);
    }

    uint16_t read()
    {
      uint16_t a;  
      ENTER_CRITICAL_SECTION();  
      a = rx[m_readCnt];
      m_readCnt++;
      if (m_readCnt == 16)
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
       if (m_txCnt == 16)m_txCnt = 0;
       }
       else{
           fl = 0;
           (USART3->DR) = byte;
           //m_txCnt--;
       }
       EXIT_CRITICAL_SECTION();
    }

}
