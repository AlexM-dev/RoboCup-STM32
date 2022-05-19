#pragma once
#include <project_config.h>
#include <pins_setup.h>
#include <usart3.h>
#include <math.h>
//#include <vector>
class BluetoothRobot
{
	public:
		BluetoothRobot(Pin &tx, Pin &rx);
		void send(int8_t data);
		void sendLetter(uint8_t data);
		uint8_t read();
	private:
		Pin m_tx, m_rx;
};
