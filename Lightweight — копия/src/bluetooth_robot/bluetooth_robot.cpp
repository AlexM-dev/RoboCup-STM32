#include "bluetooth_robot.h"

BluetoothRobot::BluetoothRobot(Pin &tx, Pin &rx):m_tx(tx),m_rx(rx)
{
	m_tx.pinInit();
	m_rx.pinInit();
	Usart3::usartInit();
}
void BluetoothRobot::send(int8_t data)
{
	if(data < 0)
	{
		data *= -1;
		Usart3::write('-');
	}
	uint8_t tmpData = data;
	uint8_t digitCount = 0; 
	while(tmpData > 0)
	{
		digitCount++;
		tmpData/=10;
	}
	for(int8_t iter = digitCount - 1; iter >= 0; iter--)
	{
		Usart3::write(uint8_t(data/pow(10.0, iter))%10 + 48);
	}
	Usart3::write(32);
}

void BluetoothRobot::sendLetter(uint8_t data)
{
	Usart3::write(data);
	//Usart3::write(32);
}

uint8_t BluetoothRobot::read()
{
	return Usart3::read();
}
