#include "gyro.h"

gyro::gyro(Pin &tx, Pin &rx):m_tx(tx),m_rx(rx)
{
	m_tx.pinInit();
	m_rx.pinInit();
	Usart1::uartInit();
}
void gyro::read()
{
  if(Usart1::available() > 0){
    rAngle = Usart1::read() * 2;

    int err = targetAngle - rAngle;
    if (err > 180) err-=360;
    else if (err < -180) err+=360;

    rotationK = (err * ROT_K) + ((err - oldErr) * ROT_D); //+ I * ROT_I;
    if(err < 3 && err > -3)
      rotationK = 0;
		else {
			  if (rotationK > 0 && rotationK < MIN_ROTATION_K)
					rotationK = MIN_ROTATION_K;
				if (rotationK < 0 && rotationK > -MIN_ROTATION_K)
					rotationK = -MIN_ROTATION_K;
		}

    oldErr = err;
	}
	
	if (rotationK > MAX_ROTATION_K)
    rotationK = MAX_ROTATION_K;
  if (rotationK < -MAX_ROTATION_K)
    rotationK = -MAX_ROTATION_K;
}

//Setters
void gyro::setRotationByGyro() {
  zeroAngle = rAngle;
  targetAngle = zeroAngle;
}

void gyro::setRotation(int a) {
  targetAngle = a + zeroAngle;
}

int gyro::getTargetRobotAngle() {
	return targetAngle - zeroAngle;
}

int gyro::getDev() {
	return rAngle - zeroAngle;
}

int gyro::getAngle() {
  return rotationK;
}
