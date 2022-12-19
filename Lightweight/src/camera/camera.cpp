#include "camera.h"

Camera::Camera(Pin &tx, Pin &rx):m_tx(tx),m_rx(rx)
{
	m_tx.pinInit();
	m_rx.pinInit();
	Usart2::uartInit();
	sign = 1;
	koef = 1;
	gotSomething = false;
	goal = 1;
	gyroAng = 0;
}


void Camera::readData()
{	
	if(Usart2::available() >= 13){
    char v = Usart2::read();
    if(v == '*'){
      for(int i = 0; i < 4; ++i){
        int a = Usart2::read() - '0';
        int b = Usart2::read() - '0';
        int c = Usart2::read() - '0';

        if(a < 0 ||  b < 0 || c < 0 || a > 9 || b > 9 || c > 9)
          continue;
        
        data[i] = a * 100;
        data[i] += b * 10;
        data[i] += c * 1;
				if (i == 0) {
					if(data[i] == dataOld[i]) {
						t0++;
					} else {
						t0=0;
					}
				}
				if (i == 2) {
					if(data[i] == dataOld[i]) {
						t2++;
					} else {
						t2=0;
					}
				}
				dataOld[i] = a * 100;
        dataOld[i] += b * 10;
        dataOld[i] += c * 1;
      }
			if (t0 > 10)
				yCanSee = false;
			else 
				yCanSee = true;
			if (t2 > 10) 
				bCanSee = false;
			else 
				bCanSee = true;
    }
  }
}

int Camera::getFormatedAngle(int ang){
  while(ang > 180)
    ang-=360;
  while(ang < -180)
    ang+=360;
  return ang;
}

int Camera::getYAngle(){
  return getFormatedAngle(data[0]);
}

int Camera::getBAngle(){
  return getFormatedAngle(data[2]);
}

int Camera::getYDist(){
  return data[1];
}

int Camera::getBDist(){
  return data[3];
}

int Camera::getCamDist(){
  return goal == 0? getBDist() : getYDist();
}

int Camera::getCamAngle(){
  return getFormatedAngle((goal == 0? getBAngle() : getYAngle()) + gyroAng);
}

int Camera::getAnotherCamDist(){
  return goal == 0? getYDist() : getBDist();  
}

int Camera::getAnotherCamAngle(){
  return getFormatedAngle((goal == 0? getYAngle() : getBAngle()) + gyroAng);
}

bool Camera::canSee(){
  return (goal == 0? bCanSee : yCanSee);
}

void Camera::setGyroAng(int a){
  gyroAng = getFormatedAngle(a);
}

