#include "gyro.h"

gyro::gyro(Pin &tx, Pin &rx):m_tx(tx),m_rx(rx)
{
	m_tx.pinInit();
	m_rx.pinInit();
	Uart4::uartInit();
	I = 0;
	maxSpeed = 3500;
}
void gyro::read()
{
  if(Uart4::available() > 0){
    rAngle = Uart4::read() * 2;

    int err = targetAngle - rAngle;
    if (err > 180) err-=360;
    else if (err < -180) err+=360;
		I += err * (millis() - rTime) / 1000;

    rotationK = (err * ROT_K) + ((err - oldErr) * ROT_D) + I * ROT_I;
		if(I > 1000)
			I = 0;
    if(err < 3 && err > -3)
      rotationK = 0;
		else {
			  if (rotationK > 0 && rotationK < MIN_ROTATION_K)
					rotationK = MIN_ROTATION_K;
				if (rotationK < 0 && rotationK > -MIN_ROTATION_K)
					rotationK = -MIN_ROTATION_K;
		}

    oldErr = err;
		rTime = micros();
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

void gyro::setMaxSpeed(int a) {
  maxSpeed = a;
}

void gyro::setRotation(int a) {	
  targetAngle = getFormatedAngle(getFormatedAngle(a) + zeroAngle);
}

int gyro::getTargetRobotAngle() {
	return getFormatedAngle(targetAngle - zeroAngle);
}

int gyro::getDevFromTarget() {
	return getFormatedAngle(rAngle - targetAngle);
}

int gyro::getDev() {
	return getFormatedAngle(rAngle - zeroAngle);
}

int gyro::getAngle() {
	if (abs(float(rotationK)) > maxSpeed)
		return maxSpeed * rotationK / abs(float(rotationK));
	return rotationK;
}

int gyro::getMaxSpeed() {
  return maxSpeed;
}

int gyro::getFormatedAngle(int ang){
  while(ang > 180)
    ang-=360;
  while(ang < -180)
    ang+=360;
  return ang;
}
