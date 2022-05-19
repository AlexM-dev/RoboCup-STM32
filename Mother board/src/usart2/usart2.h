#pragma once
#include "project_config.h"
#include "pins_setup.h"
namespace Usart2
{
//        Uart(Pin &txPin,
//             Pin &rxPin);
//        bool fl;
	void uartInit();
	void write(uint8_t byte);
	uint16_t read();
	uint16_t available();
//        Pin m_txPin;
//        Pin m_rxPin;
	extern volatile uint8_t tx[32];
	extern volatile uint8_t rx[32];
	extern volatile uint16_t m_rxCnt;
	extern volatile uint16_t m_txCnt; 
	extern volatile bool fl;
	extern volatile uint16_t m_readCnt;
	extern volatile uint16_t m_sendCnt;
}
