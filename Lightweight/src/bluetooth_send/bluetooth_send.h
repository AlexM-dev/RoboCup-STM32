#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include <usart3.h>
class BluetoothSend
{
	public:
		BluetoothSend(Pin &tx, Pin &rx);
		void send(uint8_t data);
		int8_t read();
	private:
		Pin m_tx, m_rx;
};
