#include <project_config.h>
#include <pins_setup.h>
#include <MOTOR_control.h>
#include <motion_control.h>
#include <mathematics.h>
#include <time_service.h>
#include <adc.h>
#include <dma.h>
#include <uart.h>
#include <stm32f407_SPI.h>
#include <mpu9250.h>
#include <softI2C.h>
#include <strategy.h>
#include <camera.h>
#include <lower_board.h>
#include <IRlocator.h>
#include <bluetooth_send.h>
#include <bluetooth_robot.h>
#include <ssd1306_fonts.h>
#include <IMU.h>
#include <line_sensors.h>
#include <gyro.h>

#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4

#define GOALKEEPER 0
#define ATTACKER 1
#define SNIPER 3
#define PENALTY 4
#define CODE GOALKEEPER


float getAngleMultipiller(float angle){
	angle = abs(float(angle));
	if (angle < 15) 
		return 2.3;
	if (angle < 45) 
		return 2.1;
	if (angle < 90) 
		return 1.9;
	if (angle < 135) 
		return 1.7;
	return 1.5;
	/*if (angle < 15) 
		return 1.2;
	if (angle < 45) 
		return 1.8;
	if (angle < 90) 
		return 1.7;
	if (angle < 135) 
		return 1.6;
	return 1.6;*/
}

float getAngleMultipillerGK(float angle){
	angle = abs(float(angle));
	if (angle < 15) 
		return 1.3;
	if (angle < 45) 
		return 1.5;
	if (angle < 90) 
		return 1.3;
	if (angle < 135) 
		return 1.7;
	return 1.5;
	/*if (angle < 15) 
		return 1.2;
	if (angle < 45) 
		return 1.8;
	if (angle < 90) 
		return 1.7;
	if (angle < 135) 
		return 1.6;
	return 1.6;*/
}

float getAngleP(float angle){
	if (angle > 0) {
		return 90;
	} else {
		return -90;
	}
}

int FAng (int ang) {
	while(ang > 180)
    ang-=360;
  while(ang < -180)
    ang+=360;
  return ang;
}

int getGKSpeed (int ang, int maxSpeed) {
	int sp = abs(abs(float(ang)) - 180) * abs(abs(float(ang)) - 180) * 5;
	if (sp > maxSpeed)
			sp = maxSpeed;
  return sp;
}	

int main()
{
	RCC_APB2PeriphClockCmd(RCC_APB2LPENR_SYSCFGLPEN, ENABLE);
	time_service::init();
	time_service::startTime();
  time_service::delay_ms(500);
	
	static Pin m1In1(GPIO_Pin_6,
									GPIOE,
									GPIO_Mode_AF,
									TIM9,
									CHANNEL2,
									RCC_APB2Periph_TIM9,
									GPIO_PinSource6,
									GPIO_AF_TIM9,
									4096,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_UP);
	static Pin m1In2(GPIO_Pin_5,
									GPIOE,
									GPIO_Mode_AF,
									TIM9,
									CHANNEL1,
									RCC_APB2Periph_TIM9,
									GPIO_PinSource5,
									GPIO_AF_TIM9,
									4096,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_UP);
	static 	Motor m1(m1In1, m1In2);
	
  static Pin m2In1(GPIO_Pin_7,
									GPIOB,
									GPIO_Mode_AF,
									TIM4,
									CHANNEL2,
									RCC_APB1Periph_TIM4,
									GPIO_PinSource7,
									GPIO_AF_TIM4,
									4096,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_NOPULL);
	static Pin m2In2(GPIO_Pin_8,
	  							 GPIOB,
									 GPIO_Mode_AF,
									 TIM4,
									 CHANNEL3,
									 RCC_APB1Periph_TIM4,
									 GPIO_PinSource8,
									 GPIO_AF_TIM4,
									 4096,
									 1,
									 GPIO_OType_PP,
									 GPIO_PuPd_NOPULL);
	static Motor m2(m2In1, m2In2);
	
	static Pin m3In1(GPIO_Pin_14,
									GPIOE,
									GPIO_Mode_AF,
									TIM1,
									CHANNEL4,
									RCC_APB2Periph_TIM1,
									GPIO_PinSource14,
									GPIO_AF_TIM1,
									4096,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_NOPULL);
	static Pin m3In2(GPIO_Pin_10,
									GPIOB,
									GPIO_Mode_AF,
									TIM2,
									CHANNEL3,
									RCC_APB1Periph_TIM2,
									GPIO_PinSource10,
									GPIO_AF_TIM2,
									4096,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_NOPULL);
	static Motor m3(m3In1, m3In2);
	
	static Pin m4In2(GPIO_Pin_8,
									GPIOC,
									GPIO_Mode_AF,
									TIM3,
									CHANNEL3,
									RCC_APB1Periph_TIM3,
									GPIO_PinSource8,
									GPIO_AF_TIM3,
									4096,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_NOPULL);
	static Pin m4In1(GPIO_Pin_15,
									GPIOD,
									GPIO_Mode_AF,
									TIM4,
									CHANNEL4,
									RCC_APB1Periph_TIM4,
									GPIO_PinSource15,
									GPIO_AF_TIM4,
									4096,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_NOPULL);
	static Motor m4(m4In1, m4In2);

	static Pin txTsop(GPIO_Pin_10,
						 GPIOC,
						 GPIO_Mode_AF,
						 TIM3,
						 CHANNEL2,
						 RCC_APB1Periph_TIM3,
						 GPIO_PinSource10,
						 GPIO_AF_USART3,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_NOPULL);
	static Pin rxTsop(GPIO_Pin_11,
						 GPIOC,
						 GPIO_Mode_AF,
						 TIM3,
						 CHANNEL2,
						 RCC_APB1Periph_TIM3,
						 GPIO_PinSource11,
						 GPIO_AF_USART3,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_NOPULL);
	lowerBoard tsop(txTsop, rxTsop);
 
	static Pin txCam(GPIO_Pin_2,
						 GPIOA,
						 GPIO_Mode_AF,
						 TIM3,
						 CHANNEL2,
						 RCC_APB1Periph_TIM3,
						 GPIO_PinSource2,
						 GPIO_AF_USART2,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_NOPULL);
 static Pin rxCam(GPIO_Pin_3,
						 GPIOA,
						 GPIO_Mode_AF,
						 TIM3,
						 CHANNEL2,
						 RCC_APB1Periph_TIM3,
						 GPIO_PinSource3,
						 GPIO_AF_USART2,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_NOPULL);
 	static Camera cam(txCam, rxCam);
 
 static Pin txGyro(GPIO_Pin_9,
						 GPIOA,
						 GPIO_Mode_AF,
						 TIM1,
						 CHANNEL2,
						 RCC_APB2Periph_TIM1,
						 GPIO_PinSource9,
						 GPIO_AF_USART1,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_NOPULL);
 static Pin rxGyro(GPIO_Pin_10,
						 GPIOA,
						 GPIO_Mode_AF,
						 TIM1,
						 CHANNEL3,
						 RCC_APB2Periph_TIM1,
						 GPIO_PinSource10,
						 GPIO_AF_USART1,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_NOPULL); 
  gyro gyro(txGyro, rxGyro);
 
	line_sensors line(_SPI1);
 
	static Pin switchPin(GPIO_Pin_0,
				 GPIOC,
					GPIO_Mode_IN,
				 TIM3,
				 CHANNEL2,
				 RCC_APB1Periph_TIM3,
				 GPIO_PinSource0,
				 GPIO_AF_USART1,
				 4096,
				 1,
				 GPIO_OType_PP,
				 GPIO_PuPd_DOWN);
	switchPin.pinInit();
	
	 
	static Pin button(GPIO_Pin_2,
				 GPIOC,
					GPIO_Mode_IN,
				 TIM3,
				 CHANNEL2,
				 RCC_APB1Periph_TIM3,
				 GPIO_PinSource2,
				 GPIO_AF_USART1,
				 4096,
				 1,
				 GPIO_OType_PP,
				 GPIO_PuPd_DOWN);
	button.pinInit();
	
	static Pin ballSensor(GPIO_Pin_1,
			 GPIOC,
			 GPIO_Mode_AN,
			 TIM3,
			 CHANNEL2,
			 RCC_APB1Periph_TIM3,
			 GPIO_PinSource1,
			 GPIO_AF_USART1,
			 4096,
			 1,
			 GPIO_OType_PP,
			 GPIO_PuPd_NOPULL);
	ballSensor.pinInit();
	Adc mpAdc(ADC1, 1, 0, ADC_Channel_11, RCC_APB2Periph_ADC1, ballSensor);
	mpAdc.sendMeChannel(ADC_Channel_11);
	mpAdc.adcInit();
	mpAdc.startAdc();
	mpAdc.setChannel();
	ADC_SoftwareStartConv(ADC1);
 
	robotMotion robot(m1, m2, m3, m4);
	robot.robotInit();
	mathematics math;
	
	//IMU imu;
	//imu.init(_SPI1, PA1);
	
	int32_t sp1 = 0;
	int32_t sp2 = 0;
	int32_t sp3 = 0;
	int32_t sp4 = 0;
	
	int k = 0;
	int gkGo = 0;
	unsigned long timer = 0;
	volatile bool f = false;
//	int mf[1024];
//	int c = 0;
	
	while(1)
	{
		if(button.readPin()){
			gyro.read();
			gyro.setRotationByGyro();
		}
		if(switchPin.readPin())
		{
			tsop.read();
			cam.readData();
			gyro.read();
			if(CODE == ATTACKER) {
				int speed = 3000; 
//				if(tsop.isCanSee()) {
//						if(tsop.getDist() < 70)
//							math.calculateSpeed(tsop.getAngle() * 1.2, 2500, sp1, sp2, sp3, sp4);
//						else {
//							if(abs(float(tsop.getAngle())) < 60)
//							{
//								if(abs(float(tsop.getAngle())) < 20) {
//									math.setSpeed(1500);
//									if(tsop.getDist() > 70)
//									{
//										math.setAngle(cam.getCamAngle());
//										math.setSpeed(3500);
//									}
//									else
//										math.setAngle(tsop.getAngle());
//								} else {
//										math.setVector(tsop.getAngle() * getAngleMultipiller(tsop.getAngle()), 1500);
//								}
//							} else {
//								math.setVector(tsop.getAngle() * getAngleMultipiller(tsop.getAngle()), 1500);
//							}
//						}
//				} else {
//					math.setVector(0, 0);
//				}
				volatile int tAng = tsop.getAngle() - cam.getCamAngle();
				//Main logic
//				if(tsop.isCanSee()) {
//					if (tsop.getDist() > 40) {
//						if (abs(float(tsop.getAngle())) < 20 && tsop.getDist() > 50) {
//							math.setVector(cam.getCamAngle(), speed);
//						} else {
//							if (abs(float(tsop.getAngle())) < 90) {
//								math.setVector(tsop.getAngle() * getAngleMultipiller(tsop.getAngle()), 2500);
//							} else {
//								math.setVector(tsop.getAngle() * getAngleMultipiller(tsop.getAngle()), speed);
//							}
//						}
//					} else {
//							math.setVector(tsop.getAngle(), speed);
//					}
//				} else {
//					math.setVector(tsop.getAngle(), 0);
//				}
				//
				math.setVector(0, 0); 
				/*if(tsop.isCanSee()) {
					if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 70 || abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) < 20) {
						if (tsop.getDist() > 40) {							
							if (abs(float(tsop.getAngle())) < 60) {
								math.setVector(tsop.getAngle() * getAngleMultipiller1(tsop.getAngle()), 1500);
							} else {
								math.setVector(tsop.getAngle() * getAngleMultipiller1(tsop.getAngle()), speed);
							}
						} else {
								math.setVector(tsop.getAngle(), speed);
						}
						if (abs(float(tsop.getAngle())) < 20 && tsop.getDist() > 60) {
								math.setVector(cam.getCamAngle(), speed);
						}
					} else {
						if (tsop.getDist() > 40) {
							if (abs(float(tAng)) > 15) {
								if (abs(float(tAng)) < 60) {
									math.setVector(tAng * getAngleMultipiller(tAng), 1500);
								} else {
									math.setVector(tAng * getAngleMultipiller(tAng), speed);
								}
							} else {
								math.setVector(cam.getCamAngle(), speed);
							}
						} else {
							math.setVector(tsop.getAngle(), speed);
						}
					}
				}	else {
						math.setVector(0,0);
				}*/
				
				//STABLE PART(obezd and go on goal)
				if (tsop.isCanSee()) {
					if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 50 || abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) < 20) {
						if (tsop.getDist() > 40) {
							if (abs(float(tsop.getAngle())) < 45) {
								math.setVector(tsop.getAngle() + (tsop.getAngle() > 0 ? 60 : -60), speed);
							} else {
								math.setVector(tsop.getAngle() + (tsop.getAngle() > 0 ? 60 : -60), speed);
							}
							if (abs(float(tsop.getAngle())) < 25) {
								if (tsop.getDist() > 60) {
									math.setVector(cam.getCamAngle(), speed);
								} else {
									math.setVector(tsop.getAngle()  * 1.2, speed);
								}
							}
						} else {
							if (tsop.getDist() < 30) {
									math.setVector(tsop.getAngle(), speed);
							} else {
									math.setVector(tsop.getAngle(), speed);
							}
						}
					} else {
						if (tsop.getDist() > 40) {
							if (abs(float(tAng)) > 15) {
								if (abs(float(tAng)) < 60) {
									math.setVector(tsop.getAngle() + (tAng > 0 ? 60 : -60), speed);
								} else {
									math.setVector(tsop.getAngle() + (tAng > 0 ? 60 : -60), speed);
								}
							} else {
								math.setVector(cam.getCamAngle(), speed);
							}
						} else {
							if (tsop.getDist() < 30) {
									math.setVector(tsop.getAngle(), speed);
							} else {
									math.setVector(tsop.getAngle(), speed);
							}
						}
					}
				} else {
						math.setVector(0,0);
				} 

				//math.setVector(0,0);
				//Line detecting
				//if(abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 40) //&& (abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 70 || abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 10))	
				//{
				//	math.setVector(0, speed);
				//}
				if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 10)	
				{
					math.setVector(180, speed);
				}
				if(cam.getCamDist() < 40)	
				{
					math.setVector(180, speed);
				}
				if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 70) 
				{
					if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) > 50)// || cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) > 50)	
					{
						math.setVector(90, speed);
					}
					if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) < -50)// || cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) < -50)	
					{
						math.setVector(-90, speed);
					}
				}	else {
					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) < -50)	
					{
						math.setVector(-90, speed);
					}
					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) > 50)	
					{
						math.setVector(90, speed);
					}
				}
				if(abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 35 && cam.getAnotherCamAngle() != 0)// || cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) > 50)	
				{
					math.setVector(0, speed);
				}
				//
				volatile int a = math.getAngle();
				
				//Move
				k = gyro.getAngle();
				math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
				robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
				//
			} 
			else if(CODE == GOALKEEPER) {
				int speedGK = 2500; 				
				math.setVector(0, 0);
				volatile int tAng = FAng(tsop.getAngle() - cam.getCamAngle()); 
				if(tsop.isCanSee()) {
					//Catch the ball
					if (abs(float(tAng)) < 170) {
						if (abs(float(cam.getCamAngle())) > 150) {
							if (tAng < 0) {
								math.setVector(90, getGKSpeed(tAng, speedGK));
							} else {
								math.setVector(-90, getGKSpeed(tAng, speedGK));
							}
						} else {
							if (tAng < 0) {
								math.setVector(cam.getCamAngle() - 90, getGKSpeed(tAng, speedGK));
							} else {
								math.setVector(cam.getCamAngle() + 90, getGKSpeed(tAng, speedGK));
							}
							if (abs(float(cam.getCamAngle())) < 140) {
								if(cam.getCamAngle() < 0 && tAng < 0) {
									math.setVector(0, 0);
								}
								if(cam.getCamAngle() > 0 && tAng > 0) {
									math.setVector(0, 0);
								}
							}
						}
					} else {
						//Go on ball
						math.setVector(0, 0);
					}
				} else {
					//Return on center			
					if (abs(float(cam.getCamAngle())) < 170) {
						if (cam.getCamAngle() > 0) {
							math.setVector(cam.getCamAngle() - 90, 2000);
						} else {
							math.setVector(cam.getCamAngle() + 90, 2000);
						}
					} else {
						math.setVector(0, 0);
					}
				}
				
				if(tsop.isCanSee()) {
					if ((abs(float(tAng)) > 160 || abs(float(tsop.getAngle())) < 60) && tsop.getDist() > 20) { 
						if (gkGo == 0) { 
							gkGo = 1;
							timer = millis();
						}
						if (gkGo == 1 && millis() - timer > 1500) { 
							gkGo = 2;
							timer = millis();
						}
						if (gkGo == 2 && millis() - timer < 1500) { 
							if (tsop.getDist() > 40) {
								if (abs(float(tsop.getAngle())) < 45) {
									math.setVector(tsop.getAngle() + (tsop.getAngle() > 0 ? 60 : -60), 2000);
								} else {
									math.setVector(tsop.getAngle() + (tsop.getAngle() > 0 ? 60 : -60), speedGK);
								}
								if (abs(float(tsop.getAngle())) < 15) {
									if (tsop.getDist() > 60) {
										math.setVector(cam.getAnotherCamAngle(), speedGK);
									} else {
										math.setVector(tsop.getAngle()  * 1.1, speedGK);
									}
								}
							} else {
								if (tsop.getDist() < 30) {
										math.setVector(tsop.getAngle(), speedGK);
								} else {
										math.setVector(tsop.getAngle(), speedGK);
								}
							}
						}
						if (gkGo == 2 && millis() - timer >= 1500) { 
							gkGo = 0;
						}
					}
				}
				
				if (gkGo != 2) {
					if (abs(float(cam.getCamAngle())) > 150) {
						if (abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 50 || cam.getCamAngle() == 0) {
							if (cam.getCamAngle() == 0) {
								math.setVector(180, 2000);
							} else {
								math.setVector(cam.getCamAngle(), 2000);
							}
						}
						if (abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 45) {
								math.addVector(180, 2000);
						}
						if (abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 35) {
								math.addVector(0, 2000);
						}
					} else {
						if (cam.getCamDist() > 45) {
							math.addVector(cam.getCamAngle(), 2000);	
						}
						if (cam.getCamDist() < 35) {
							math.addVector(cam.getCamAngle() + 180, 2000);
						}
					}
				} else {
					if (abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) > 50 || tsop.getDist() < 20) {
						gkGo = 0;
					}
					/*if((abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 60 && cam.getAnotherCamAngle() != 0) || (abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 90))	
					{
						gkGo = 0;
					}*/
				}
				if (abs(float(tsop.getAngle())) > 170) {
							math.setVector(0, 0);
				}
				math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
				k = gyro.getAngle();
				robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
			}
		}
		else
		{
			robot.move(0, 0, 0, 0);
		}
	} 
}
