#ifndef IMU_LIB
#define IMU_LIB

#include "stm32f407_sysFunc.h"
#include "stm32f407_pin.h"
#include "stm32f407_pinList.h"
#include "stm32f407_wrappers.h"
#include "stm32f407_SPI.h"
#include "mpu9250.h"


class IMU
{
	public:
		void init(unsigned int spi, uint16_t ss);
		void turnOn();
		void turnOff();
		void setZeroAngle();
		void calibrate(uint32_t t);
		unsigned int update();
		void updateAnglesFromFIFO();
		double getAngle();
		double imuFloatValue, angleChange;
		long long int imuFloatTime;
		double getXa();
		double getYa();
		double getZa();
		double getXg();
		double getYg();
		double getZg();
		mpu9250 mpuSensor;
	
	private:
		
		unsigned int spi;
		uint16_t en, ss;
		volatile double angle, zeroAngle;
		bool working;
		double correctAngle(double angle);
};


void IMU::init(unsigned int spi, uint16_t ss)
{
	this->spi = spi;
	this->ss = ss;
	
	turnOn();
	delay(100);
	mpuSensor.initIMU(spi, ss);
	delay(100);
	
	setZeroAngle(); // IMU calibrated angle estimating
	
	imuFloatTime = millis();
	imuFloatValue = 0;
}


void IMU::turnOn()
{
	working = 1;
}


void IMU::turnOff()
{
	working = 0;
}


void IMU::setZeroAngle()
{
	update();
	zeroAngle = angle;
}


unsigned int IMU::update()
{	
	updateAnglesFromFIFO();
	double neangle = mpuSensor.yaw;
	angle = neangle;
	return 0;
}


void IMU::updateAnglesFromFIFO()
{
	mpuSensor.updateAnglesFromFIFO();
}


void IMU::calibrate(uint32_t t)
{
	mpuSensor.calibrate(t);
}


double IMU::getAngle()
{
	double a = angle - zeroAngle;
	
	return -correctAngle(a);
}

double IMU::correctAngle(double angle)
{
	while(angle > 180)
		angle-=360;
	while(angle < -180)
		angle+=360;
	return angle;
}

double IMU::getXa()
{
	return mpuSensor.getXa();
}

double IMU::getYa()
{
	return mpuSensor.getYa();
}

double IMU::getZa()
{
	return mpuSensor.getZa();
}

double IMU::getXg()
{
	return mpuSensor.getXg();
}

double IMU::getYg()
{
	return mpuSensor.getYg();
}

double IMU::getZg()
{
	return mpuSensor.getZg();
}


#endif
