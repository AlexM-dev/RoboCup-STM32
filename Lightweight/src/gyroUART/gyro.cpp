#include "gyro.h"

gyro::gyro(Pin &tx, Pin &rx):m_tx(tx),m_rx(rx)
{
	m_tx.pinInit();
	m_rx.pinInit();
	Uart4::uartInit();
	I = 0;
	maxSpeed = 256;
	rTime = micros();
}

void gyro::read()
{
  if(Uart4::available() > 0){
    rAngle = Uart4::read() * 2;

    int err = targetAngle - rAngle;
    if (err > 180) err-=360;
    else if (err < -180) err+=360;
		I += err * (micros() - rTime) / 1000;

    rotationK = (err * ROT_K) + ((err - oldErr) * ROT_D) + I * ROT_I;
    if(err < 2 && err > -2)
      rotationK = 0, I = 0;
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
	
	if (rotationK > maxSpeed)
    rotationK = maxSpeed;
  if (rotationK < -maxSpeed)
    rotationK = -maxSpeed;
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

float gyro::getRotationKForRotateToBall(float k, float d, float i)
{
	int err = targetAngle - rAngle;
	if (err > 180) err-=360;
	else if (err < -180) err+=360;

	float rotK = (err * k) + ((err - oldErr) * d) + I * i;
	if(err < 2 && err > -2)
		rotK = 0;
	else {
			if (rotK > 0 && rotK < MIN_ROTATION_K)
				rotK = MIN_ROTATION_K;
			if (rotK < 0 && rotK > -MIN_ROTATION_K)
				rotK = -MIN_ROTATION_K;
	}
	
	if (rotK > MAX_ROTATION_K)
    rotK = MAX_ROTATION_K;
  if (rotK < -MAX_ROTATION_K)
    rotK = -MAX_ROTATION_K;
	
	if (rotK > maxSpeed)
    rotK = maxSpeed;
  if (rotK < -maxSpeed)
    rotK = -maxSpeed;
	return rotK;
}

int gyro::getDevFromTarget() {
	 return getFormatedAngle(targetAngle - rAngle);
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
