#include "bluetooth_send.h"

BluetoothSend::BluetoothSend(Pin &tx, Pin &rx):m_tx(tx),m_rx(rx)
{
	m_tx.pinInit();
	m_rx.pinInit();
	Usart3::usartInit();
}
void BluetoothSend::send(uint8_t data)
{
	Usart3::write(data);
}

int8_t BluetoothSend::read()
{
	if(Usart3::available())
	{
		return Usart3::read() - '0';
	}
	else
	{
		return -1;
	}
}
