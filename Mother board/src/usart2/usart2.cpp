#include "usart2.h"

extern "C"
{
	void USART2_IRQHandler(void)
	{
        volatile uint8_t data = USART2->SR;

        if(data & USART_SR_RXNE){
            Usart2::rx[Usart2::m_rxCnt] = USART2->DR; 
            Usart2::m_rxCnt++;
            if (Usart2::m_rxCnt == 32)
            {
                Usart2::m_rxCnt = 0;
            }
        }
        if(data & USART_SR_TC)
        {
            USART_ClearITPendingBit(USART2, USART_IT_TC);
            if (Usart2::m_txCnt != 0)
            {
                (USART2->DR) = Usart2::tx[Usart2::m_sendCnt];
                //Usart2::m_txCnt;
                Usart2::m_sendCnt++;
                if(Usart2::m_sendCnt == 32)Usart2::m_sendCnt = 0;
            }
            else{
                Usart2::fl = 1;
            }
		//time_service::systemTime++;
        }
        if(USART2->SR & USART_SR_ORE)
        {
            uint8_t a = USART2->DR;
            (void)a;
        }
    }
}
namespace Usart2{
//Usart2::Usart2(Pin &txPin,
//           Pin &rxPin):
//           m_txPin(txPin),
//           m_rxPin(rxPin)
//{
//               
//}
    volatile uint8_t tx[32];
    volatile uint8_t rx[32];
    volatile uint16_t m_rxCnt;
    volatile uint16_t m_txCnt; 
    volatile bool fl;
    volatile uint16_t m_readCnt;
    volatile uint16_t m_sendCnt;
    void uartInit()
    {
        fl = 1;
        m_txCnt = 0;
        m_rxCnt = 0;
        m_readCnt = 0;
        m_sendCnt = 0;
       // m_txPin.pinInit();
       // m_rxPin.pinInit();
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        USART_InitTypeDef uart;
        uart.USART_BaudRate = 115200;
        uart.USART_WordLength = USART_WordLength_8b;
        uart.USART_StopBits = USART_StopBits_1;
        uart.USART_Parity = USART_Parity_No;
        uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART2, &uart);
        
        USART_ITConfig(USART2, USART_IT_TC, ENABLE);
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        
        USART_Cmd(USART2, ENABLE);
        NVIC_SetPriority(USART2_IRQn, 0);
        NVIC_EnableIRQ(USART2_IRQn);
    }

    uint16_t read()
    {
      uint16_t a;  
      ENTER_CRITICAL_SECTION();  
      a = rx[m_readCnt];
      m_readCnt++;
      if (m_readCnt == 32)
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
       if (m_txCnt == 32)m_txCnt = 0;
       }
       else{
           fl = 0;
           (USART2->DR) = byte;
           //m_txCnt--;
       }
       EXIT_CRITICAL_SECTION();
    }

}
