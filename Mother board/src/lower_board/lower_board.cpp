#include "lower_board.h"

lowerBoard::lowerBoard(Pin &tx, Pin &rx):m_tx(tx),m_rx(rx)
{
	m_tx.pinInit();
	m_rx.pinInit();
	Usart3::usartInit();
	erri = 0;
	m_canSee = false;
	m_angle = 0;
	m_dist = 0;
	count = 0;
}

int lowerBoard::crc8(int* data, int len)
{
    int crc = 0xFF, i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (char)((crc << 1) ^ 0x31);
            else crc <<= 1;
        }
    }
    return crc;
}

void lowerBoard::read()
{
	int dt[3];

  if(Usart3::available() > 3){
    if(Usart3::read() == 0xff){
      for(int i = 0; i < 3; ++i)
        dt[i] = Usart3::read();
			
			if(crc8(dt, 2) == dt[2])
			{
				if(dt[1] == 0){
					count++;
					m_canSee = false;
					m_dist = 0;
				}
				else {
					count = 0;
					if (dt[0] <= 180 && dt[0] > 0) {
						m_angle = dt[0] * 2;
						m_dist = dt[1];
						m_canSee = true;
					}						
					/*if(m_angle > 180)
						m_angle = m_angle - 360;
					if(m_angle < -180)
						m_angle = m_angle + 360;*/
				}
			}
    }
	} 
	
	/*if(millis() - erri > 1000)
		m_canSee = false;
	else 
		m_canSee = true;*/
}

int lowerBoard::getAngle()
{
	//int retAng = m_angle + 90;
	//return m_angle;
	if(m_angle < 180)
		return m_angle;
	else
		return m_angle - 360;
}
int lowerBoard::getDist()
{
	return m_dist;
}
bool lowerBoard::isCanSee()
{
	return m_canSee;
}
