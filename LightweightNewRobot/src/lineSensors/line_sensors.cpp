#include "line_sensors.h"

line_sensors::line_sensors(unsigned int spi):m_spi(spi)
{
	initSPI(m_spi, MASTER, 8, 2);
}

void line_sensors::read()
{
  if(SPIAvailable(m_spi)){
    m_angle = writeSPI(_SPI1, 0);
		m_canSee = true;
	} else {
		m_canSee = false;
	}
}

bool line_sensors::isCanSee()
{
	return m_canSee;
}
