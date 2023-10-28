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
#include <math.h>
#include <cmath>

#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4

#define GOALKEEPER 0
#define ATTACKER 1
#define SNIPER 3
#define PENALTY 4
#define DEBUG 5
#define NEW_ATTAKER 6
#define CODE ATTACKER

#define Ec 2.7182818284
#define BALL_SENSOR_THRESHOLD 450 

#define BALL_IS_NEAR 25
#define BALL_IS_REALLY_NEAR 70

double convertDist(double dist) {
			double maxDist = BALL_IS_REALLY_NEAR;
			double v = (dist - maxDist) / maxDist + 1;
			if (v > 1) 
				v = 1;
			if (v < 0) 
				v = 0;
      return v;
}

float AngleOffset(float angle, float dist) {
	if (abs(angle) < 25) {
		return 0;
	}
	if (dist > BALL_IS_NEAR && abs(angle) < 90) {
		if (angle < 0) {
			return -45;
		} else {
			return 45;
		}
	}
	double angK = 0.05 * pow(double(Ec), double(0.15 * abs(angle)));
	if (angK > 90)
		angK = 90;
	dist = convertDist(dist);
	double distK = 0.02 * pow(double(Ec), double(4.5 * dist));
	/*if (abs(angle) < 50) {
		distK = 1;
	}*/
	//double distK = 1;
	if (distK > 1)
		distK = 1;
	if (angle > 0) {
		return angK * distK;
	} else {
		return -angK * distK;
	}
}

int FAng (int ang) {
	while(ang > 180)
    ang-=360;
  while(ang < -180)
    ang+=360;
  return ang;
}

int getGKSpeed (float ang, float dist, int maxSpeed) {
	if (abs(float(ang)) < 10) {
		return 0;
	}
	dist = convertDist(dist);
	if (dist > 1)
		dist = 1;
	int sp = abs(abs(float(ang)) - 180) * abs(abs(float(ang)) - 180) / 4;
	if (sp > maxSpeed)
			sp = maxSpeed;
	if (sp < 60)
			sp = 60;
  return sp;
}

int getSpeedChalandge (int ang, int maxSpeed) {
	if (abs(float(ang)) < 1) {
		return 0;
	}
	int sp = abs(abs(float(ang)) - 180) * abs(abs(float(ang)) - 180) * 100;
	if (sp > maxSpeed)
			sp = maxSpeed;
	if (sp < 130)
			sp = 130;
  return sp;
}

int getSpeedChalandge2 (int dist, int maxSpeed) {
	if (abs(float(dist)) < 1) {
		return 0;
	}
	int sp = abs(float(dist)) * abs(float(dist)) * 20;
	if (sp > maxSpeed)
			sp = maxSpeed;
	if (sp < 70)
			sp = 70;
  return sp;
}

int getFRWSpeed (int ang, int maxSpeed, int minSpeed) {
	int sp = abs(float(ang)) * abs(float(ang)) / 64;
	if (sp > maxSpeed)
			sp = maxSpeed;
	if (sp < minSpeed)
			sp = minSpeed;
  return sp;
}

int getGKSpeedSlow (int dist, int maxSpeed) {
	int sp = abs(float(dist)) * abs(float(dist)) * 2;
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
	volatile uint32_t bebra = RCC->PLLCFGR;
	/*static Pin dribbler(GPIO_Pin_7,
							GPIOE,
							GPIO_Mode_AF,
							TIM9,
							CHANNEL2,
							RCC_APB2Periph_TIM9,
							GPIO_PinSource6,
							GPIO_AF_TIM9,
							65535,
							1,
							GPIO_OType_PP,
							GPIO_PuPd_NOPULL);
	dribbler.pinInit();
	dribbler.pwmInit();
	dribbler.pwm(32125); */

	uint32_t period = 256;
	static Pin m1In1(GPIO_Pin_6,
									GPIOE,
									GPIO_Mode_AF,
									TIM9,
									CHANNEL2,
									RCC_APB2Periph_TIM9,
									GPIO_PinSource6,
									GPIO_AF_TIM9,
									period,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_DOWN);
	static Pin m1In2(GPIO_Pin_5,
									GPIOE,
									GPIO_Mode_AF,
									TIM9,
									CHANNEL1,
									RCC_APB2Periph_TIM9,
									GPIO_PinSource5,
									GPIO_AF_TIM9,
									period,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_DOWN);
	static 	Motor m1(m1In1, m1In2);
	
  static Pin m2In1(GPIO_Pin_14,
									GPIOD,
									GPIO_Mode_AF,
									TIM4,
									CHANNEL3,
									RCC_APB1Periph_TIM4,
									GPIO_PinSource14,
									GPIO_AF_TIM4,
									period,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_DOWN);
	static Pin m2In2(GPIO_Pin_15,
	  							 GPIOD,
									 GPIO_Mode_AF,
									 TIM4,
									 CHANNEL4,
									 RCC_APB1Periph_TIM4,
									 GPIO_PinSource15,
									 GPIO_AF_TIM4,
									 period,
									 1,
									 GPIO_OType_PP,
									 GPIO_PuPd_DOWN);
	static Motor m2(m2In2, m2In1);
	
	static Pin m3In1(GPIO_Pin_10,
									GPIOB,
									GPIO_Mode_AF,
									TIM2,
									CHANNEL3,
									RCC_APB1Periph_TIM2,
									GPIO_PinSource10,
									GPIO_AF_TIM2,
									period,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_DOWN);
	static Pin m3In2(GPIO_Pin_11,
									GPIOB,
									GPIO_Mode_AF,
									TIM2,
									CHANNEL4,
									RCC_APB1Periph_TIM2,
									GPIO_PinSource11,
									GPIO_AF_TIM2,
									period,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_DOWN);
	static Motor m3(m3In1, m3In2);
	
	static Pin m4In2(GPIO_Pin_12,
									GPIOD,
									GPIO_Mode_AF,
									TIM4,
									CHANNEL1,
									RCC_APB1Periph_TIM4,
									GPIO_PinSource12,
									GPIO_AF_TIM4,
									period,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_DOWN);
	static Pin m4In1(GPIO_Pin_13,
									GPIOD,
									GPIO_Mode_AF,
									TIM4,
									CHANNEL2,
									RCC_APB1Periph_TIM4,
									GPIO_PinSource13,
									GPIO_AF_TIM4,
									period,
									1,
									GPIO_OType_PP,
									GPIO_PuPd_DOWN);
	static Motor m4(m4In1, m4In2);
	
	static Pin dribbler(GPIO_Pin_9,
							GPIOC,
							GPIO_Mode_AF,
							TIM3,
							CHANNEL4,
							RCC_APB1Periph_TIM3,
							GPIO_PinSource9,
							GPIO_AF_TIM3,
							2000,
							159,
							GPIO_OType_PP,
							GPIO_PuPd_DOWN);
	dribbler.pinInit();
	dribbler.pwmInit();
	
	static Pin changeSideButton(GPIO_Pin_2,
			 GPIOD,
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
	changeSideButton.pinInit();
	
	static Pin startButton(GPIO_Pin_4,
		 GPIOD,
		 GPIO_Mode_IN,
		 TIM3,
		 CHANNEL2,
		 RCC_APB1Periph_TIM3,
		 GPIO_PinSource4,
		 GPIO_AF_USART1,
		 4096,
		 1,
		 GPIO_OType_PP,
		 GPIO_PuPd_DOWN);
	startButton.pinInit();
	
	static Pin imuReset(GPIO_Pin_5,
	 GPIOD,
	 GPIO_Mode_IN,
	 TIM3,
	 CHANNEL2,
	 RCC_APB1Periph_TIM3,
	 GPIO_PinSource5,
	 GPIO_AF_USART1,
	 4096,
	 1,
	 GPIO_OType_PP,
	 GPIO_PuPd_DOWN);
	imuReset.pinInit();
	
	static Pin lightTransistor(GPIO_Pin_5,
			 GPIOA,
			 GPIO_Mode_OUT,
			 TIM8,
			 CHANNEL1,
			 RCC_APB2Periph_TIM8,
			 GPIO_PinSource5,
			 GPIO_AF_TIM8,
			 4096,
			 1,
			 GPIO_OType_PP,
			 GPIO_PuPd_DOWN);
	lightTransistor.pinInit();

	static Pin txTsop(GPIO_Pin_6,
						 GPIOC,
						 GPIO_Mode_AF,
						 TIM3,
						 CHANNEL1,
						 RCC_APB1Periph_TIM3,
						 GPIO_PinSource6,
						 GPIO_AF_USART6,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_DOWN);
	static Pin rxTsop(GPIO_Pin_7,
						 GPIOC,
						 GPIO_Mode_AF,
						 TIM3,
						 CHANNEL2,
						 RCC_APB1Periph_TIM3,
						 GPIO_PinSource7,
						 GPIO_AF_USART6,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_DOWN);
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
						 GPIO_PuPd_DOWN);
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
						 GPIO_PuPd_DOWN);
 	static Camera cam(txCam, rxCam);
 
 static Pin txGyro(GPIO_Pin_0,
						 GPIOA,
						 GPIO_Mode_AF,
						 TIM5,
						 CHANNEL1,
						 RCC_APB1Periph_TIM5,
						 GPIO_PinSource0,
						 GPIO_AF_UART4,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_DOWN);
 static Pin rxGyro(GPIO_Pin_1,
						 GPIOA,
						 GPIO_Mode_AF,
						 TIM5,
						 CHANNEL2,
						 RCC_APB1Periph_TIM5,
						 GPIO_PinSource1,
						 GPIO_AF_UART4,
						 4096,
						 1,
						 GPIO_OType_PP,
						 GPIO_PuPd_DOWN); 
  gyro gyro(txGyro, rxGyro);
 
 	static Pin lightSensor(GPIO_Pin_2,
			 GPIOC,
			 GPIO_Mode_OUT,
			 TIM8,
			 CHANNEL1,
			 RCC_APB2Periph_TIM8,
			 GPIO_PinSource2,
			 GPIO_AF_TIM8,
			 4096,
			 1,
			 GPIO_OType_PP,
			 GPIO_PuPd_DOWN);
	lightSensor.pinInit();
	lightSensor.setBit();
 
	/*line_sensors line(_SPI1);
 
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
	ADC_SoftwareStartConv(ADC1);*/
	static Pin ballSensor(GPIO_Pin_3,
			 GPIOC,
			 GPIO_Mode_AN,
			 TIM3,
			 CHANNEL2,
			 RCC_APB1Periph_TIM3,
			 GPIO_PinSource3,
			 GPIO_AF_USART1,
			 4096,
			 1,
			 GPIO_OType_PP,
			 GPIO_PuPd_DOWN);
	ballSensor.pinInit();
	Adc mpAdc(ADC1, 1, 0, ADC_Channel_13, RCC_APB2Periph_ADC1, ballSensor);
	mpAdc.sendMeChannel(ADC_Channel_13);
	mpAdc.adcInit();
	mpAdc.startAdc();
	mpAdc.setChannel();
	ADC_SoftwareStartConv(ADC1);
	//mpAdc.getAdc
	int ballSensorRes = 0;	
	unsigned long ballSensorTimer = 0;
	volatile bool ballCathed = false, ballCathedFlag = false;
	
	lightTransistor.setBit();
	robotMotion robot(m1, m2, m3, m4);
	robot.robotInit();
	mathematics math;
	
	bool startMovingFlag = 0;
	long startMovingTimer = millis();
	
	
	//IMU imu;
	//imu.init(_SPI1, PA1);
	
	int32_t sp1 = 0;
	int32_t sp2 = 0; 
	int32_t sp3 = 0;
	int32_t sp4 = 0;
	
	bool pressFlag[4] = {0};
	
	int k = 0;
	int gkGo = 0;
	unsigned long timer = 0, kickTimer = 0, goBackTimer, penTimer;
	bool penFlag = 0;
	volatile bool f = false, updateF = false;
	int state = 0;
	int goalSide = 0;
	int32_t sp = 0, ang = 0;
	math.setPeriod(period);
	int strategyState = CODE;
	int kickState = 0;
	int kickSector = 0;
	
	volatile int testAng = 0; 
	int pwm = 120;
	
	int dribblerOn = 0;
	int temptAng = 0;
	
	bool mKick = 0;
	
	bool runFlag = 0;
	bool gyroFlag = 0;
	bool goBackFlag = 0;
	delay(500);
	dribbler.pwm(120);
	delay(500);
	dribbler.pwm(250);
	delay(500);
	dribbler.pwm(120);
	while(1)
	{
		ballSensorRes = (int)ADC1->DR;
		if (ballCathed && ballSensorRes > BALL_SENSOR_THRESHOLD) { 
			if (!ballCathedFlag) {
				ballCathedFlag = 1;
				ballSensorTimer = millis();
			} else {
				if (millis() - ballSensorTimer > 1000) {
					ballCathedFlag = 0; 
					ballCathed = !ballCathed;
				}
			}
		} else if (!ballCathed && ballSensorRes < BALL_SENSOR_THRESHOLD) {
			if (!ballCathedFlag) {
				ballCathedFlag = 1;
				ballSensorTimer = millis();
			} else {
				if (millis() - ballSensorTimer > 40) {
					ballCathedFlag = 0; 
					ballCathed = !ballCathed;
				}
			}
		} else {
			ballCathedFlag = 0;
		}

		gyro.read();
		cam.readData();
		tsop.read();
		if(startButton.readPin() && pressFlag[0] == 0){
			pressFlag[0] = 1;
		}
		if(!startButton.readPin() && pressFlag[0] == 1){
			pressFlag[0] = 0;
			runFlag = !runFlag;
			if (runFlag && abs(float(tsop.getAngle())) < 45 && tsop.getDist() > BALL_IS_NEAR) {
				startMovingFlag = 1;
				startMovingTimer = millis();
			}
		}
		
		if(changeSideButton.readPin() && pressFlag[1] == 0) {
				pressFlag[1] = 1;
		}
		if(!changeSideButton.readPin() && pressFlag[1] == 1) {			
			pressFlag[1] = 0;
			/*dribblerOn++;
			if(dribblerOn >= 3) {
				dribblerOn = 0;
			}*/
			cam.changeSide();
		}
		
		if(imuReset.readPin()){
			gyro.read();
			gyro.setRotationByGyro();
			gyro.setRotation(0);
		}
		if (runFlag) {	
			if(CODE == ATTACKER) {
				int speedFastFast = 256 * 0.9;
				int speedFast = 256 * 0.8;
				int speedSlow = 256 * 0.4;
				
				int speedBack = 256 * 0.9;
				gyroFlag = 0;				
				math.setVector(0, 0);
				//tsop.setGyroAng(0);
				gyro.setMaxSpeed(256);
				dribblerOn = 0;
				cam.setGyroAng(gyro.getDev());
				if (startMovingFlag == 0) {
					if (kickState == 0) {
						kickSector = 0;
						if (cam.getAnotherCamDist() != 0) {
							gyro.setRotation(180 + cam.getAnotherCamAngle());
						} else { 
							gyro.setRotation(180);
						}
						 
						if (tsop.isCanSee() || ballCathed) {
							if (ballCathed) {
								if (goBackFlag == 0) {
									goBackFlag = 1;
									goBackTimer = millis();
								} else {
									gyro.setMaxSpeed(50);
									math.setVector(180, (millis() - goBackTimer) / 6 > speedBack ? speedBack : (millis() - goBackTimer) / 6);
								}								
								dribblerOn = 2;
								if (cam.getAnotherCamDist() != 0 && abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 80) {
									if (millis() - goBackTimer < 200) {
										mKick = 1;
									} else {
										mKick = 0;
									}
									kickTimer = millis();
									kickState = 1;
								}
							} else {
								goBackFlag = 0;
								if (abs(float(tsop.getAngle())) < 90) {
										dribblerOn = 2;
								}
								if (tsop.getDist() > BALL_IS_NEAR) {
									if (abs(float(tsop.getAngle())) < 90) {
										math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedSlow);
									} else {
										math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedFast);
									}
									//getFRWSpeed(tsop.getAngle(), speedFast, speedSlow));
								} else {
									math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedFastFast);
								}
							}
						} else {
							math.setVector(0, 0);
						}
					} else {
						if (cam.getAnotherCamDist() != 0 && abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) > 80) {
							kickState = 0;
							continue;
						}
						dribblerOn = 2;
						if (cam.getAnotherCamDist() != 0) {
							gyro.setRotation(180 + cam.getAnotherCamAngle());
						} else {
							gyro.setRotation(180);
						}
						
						if (millis() - kickTimer > 1000) {
							kickState = 0;
							dribblerOn = 0;
							kickSector = 0;
							ballCathed = 0;
						} else {
							if (kickState == 1) {
								int dY = cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3);
								int dX = cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3);
								if (abs(float(cam.getAnotherCamAngle())) > 30) {
										if (dY < 0) {
											kickSector = 1;
										} else {
											kickSector = 6;
										}
								} else {
									if (dY < -30) {
										kickSector = 2;
									} else if (dY < 0) {
										kickSector = 3;
									} else if (dY < 30) {
										kickSector = 4;
									} else {
										kickSector = 5;
									}
								}
								kickState = 2;
								
								if (mKick == 1) {
									goBackTimer = millis();
								}
							} else if (kickState == 2) {
								if (millis() - goBackTimer < 350) {
									gyro.setRotation(180 + cam.getAnotherCamAngle());
								} else {
									dribblerOn = 2;
									switch(kickSector) {
									case 1:
										gyro.setRotation(-25 + cam.getAnotherCamAngle());
										break; 
									case 2:
										gyro.setRotation(25 + cam.getAnotherCamAngle());
										break;
									case 3:
										gyro.setRotation(25 + cam.getAnotherCamAngle());
										break;
									case 4:
										gyro.setRotation(-25 + cam.getAnotherCamAngle());
										break;
									case 5:
										gyro.setRotation(-25 + cam.getAnotherCamAngle());
										break;
									case 6:
										gyro.setRotation(25 + cam.getAnotherCamAngle());
										break;
									}
								}
							}
						}
					}
				} else {
					gyro.setRotation(0);
					if (tsop.getDist() > BALL_IS_NEAR) {
						if (abs(float(tsop.getAngle())) < 90) {
							math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedSlow);
						} else {
							math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedFast);
						}
					} else {
						math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedFastFast);
					}
					dribblerOn = 2;
					if (millis() - startMovingTimer > 5000 || ballCathed) {
						startMovingFlag = 0;
					}
				}
				
				int autX = 55;
					
				if (cam.getCamDist() != 0) {
					if(abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) > 50 || abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 40)
					{
						if (math.getSpeed() > 150) {
							math.setSpeed(150);
						}
					}
					
					if(abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) < 40) {
						if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 40)  
						{
							math.setVector(0 - gyro.getTargetRobotAngle(), speedFast);
						}
					}
					
					if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 25)  
					{
						math.setVector(0 - gyro.getTargetRobotAngle(), speedFast);
					}
		
					if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) > autX)
					{
						math.addVector(90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
					}
					if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) < -autX)
					{
						math.addVector(-90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
					}
				}
				
				if (cam.getAnotherCamDist() != 0) {
					if(abs(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3)) > 50 || abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 40)
					{
						if (math.getSpeed() > 150) {
							math.setSpeed(150);
						}
					}
					
					if(abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 20)  
					{
						math.setVector(180 - gyro.getTargetRobotAngle(), speedFast);
					}
					if(cam.getAnotherCamDist() < 40)  
					{
						if(abs(float(cam.getAnotherCamAngle())) > 45) {
							math.addVector((180 + cam.getAnotherCamAngle()) - gyro.getTargetRobotAngle(), math.getSpeed() + speedSlow);
						} else {
							math.addVector(180 - gyro.getTargetRobotAngle(), math.getSpeed() + speedSlow);
						}
					}
					
					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) < -autX)  
					{
						math.addVector(-90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedSlow);
					}
					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) > autX)  
					{
						math.addVector(90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedSlow);
					}
				}
				//math.setVector(0, 0);
				//gyro.setRotation(180);
				k = gyro.getAngle();
				//kickSector = 2;
				//dribblerOn = 2;
			} else if (CODE == NEW_ATTAKER) {
				int speedFastFast = 256 * 0.9;
				int speedFast = 256 * 0.8;
				int speedSlow = 256 * 0.6;
				
				int speedBack = 256 * 0.9;
				gyroFlag = 0;				
				math.setVector(0, 0);
				tsop.setGyroAng(gyro.getDev());
				gyro.setMaxSpeed(256);
				dribblerOn = 0;
				cam.setGyroAng(gyro.getDev());
				if (startMovingFlag == 0) {
					if (kickState == 0) {
						kickSector = 0;
						mKick = 0;

						if (tsop.isCanSee() || ballCathed) {
							if (ballCathed) {
								if (cam.getAnotherCamDist() != 0) {
									gyro.setRotation(180 + cam.getAnotherCamAngle());
								} else { 
									gyro.setRotation(180);
								}
								if (abs(float(gyro.getDevFromTarget())) > 30 && goBackFlag == 0) {
									mKick = 1;
									math.setSpeed(0);
									dribblerOn = 2;
								} else {
									gyro.setMaxSpeed(50);
									if (goBackFlag == 0) {
										goBackFlag = 1;
										goBackTimer = millis();
									} else {
										//if (cam.getCamDist() != 0) {
										//	math.setVector(180 + cam.getAnotherCamAngle(), (millis() - goBackTimer) / 6 > speedBack ? speedBack : (millis() - goBackTimer) / 6);
										//} else {
											math.setVector(180, (millis() - goBackTimer) / 6 > speedBack ? speedBack : (millis() - goBackTimer) / 6);
										//}
									}								
									dribblerOn = 2;
									if (cam.getAnotherCamDist() != 0 && abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 70) {
										if (millis() - goBackTimer < 500) {
											mKick = 1;
										} else {
											mKick = 0;
										}
										kickTimer = millis();
										kickState = 1;
									}
								}
							} else {
								/*if (cam.getCamDist() != 0 && abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 60 && abs(float(tsop.getLocalAngle())) > 90) {
									gyro.setRotation(180);
								} else {
									gyro.setRotation(0);
								}*/
								gyro.setRotation(0);
								goBackFlag = 0;
								if (abs(float(tsop.getAngle())) < 90) {
										dribblerOn = 2;
								}
								if (tsop.getDist() > BALL_IS_NEAR) {
									if (abs(float(tsop.getAngle())) < 45) {
										math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedSlow);
									} else {
										math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedFast);
									}
								} else {
									math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedFastFast);
								}
							}
						} else {
							math.setVector(0, 0);
						}
					} else {
						if (cam.getAnotherCamDist() != 0 && abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) > 80) {
							kickState = 0;
						}
						dribblerOn = 2;
						if (cam.getAnotherCamDist() != 0) {
							gyro.setRotation(180 + cam.getAnotherCamAngle());
						} else {
							gyro.setRotation(180);
						}
						
						if (millis() - kickTimer > 3000) {
							kickState = 0;
							dribblerOn = 0;
							kickSector = 0;
							ballCathed = 0;
						} else {
							if (kickState == 1) {
								int dY = cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3);
								int dX = cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3);
								if (abs(float(cam.getAnotherCamAngle())) > 30) {
										if (dY < 0) {
											kickSector = 1;
										} else {
											kickSector = 6;
										}
								} else {
									if (dY < -30) {
										kickSector = 2;
									} else if (dY < 0) {
										kickSector = 3;
									} else if (dY < 30) {
										kickSector = 4;
									} else {
										kickSector = 5;
									}
								}
								kickState = 2;
								
								if (mKick == 1) {
									goBackTimer = millis();
								} else {
									goBackTimer = 0;
								}
							} else if (kickState == 2) {
								if (mKick == 1) {
									gyro.setRotation(180 + cam.getAnotherCamAngle());
									if (millis() - goBackTimer > 1000 && abs(float(gyro.getDevFromTarget())) < 15) {
										mKick = 0;
									}
								} else {
									mKick = 0;
									dribblerOn = 2;
									switch(kickSector) {
									case 1:
										gyro.setRotation(-25 + cam.getAnotherCamAngle());
										break; 
									case 2:
										gyro.setRotation(25 + cam.getAnotherCamAngle());
										break;
									case 3:
										gyro.setRotation(25 + cam.getAnotherCamAngle());
										break;
									case 4:
										gyro.setRotation(-25 + cam.getAnotherCamAngle());
										break;
									case 5:
										gyro.setRotation(-25 + cam.getAnotherCamAngle());
										break;
									case 6:
										gyro.setRotation(25 + cam.getAnotherCamAngle());
										break;
									}
								}
							}
						}
					}
				} else {
					gyro.setRotation(0);
					if (tsop.getDist() > BALL_IS_NEAR) {
						if (abs(float(tsop.getAngle())) < 90) {
							math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedSlow);
						} else {
							math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedFast);
						}
					} else {
						math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speedFastFast);
					}
					dribblerOn = 2;
					if (millis() - startMovingTimer > 5000 || ballCathed) {
						startMovingFlag = 0;
					}
				}
				
				
				int autX = 60;
				
				if (cam.getCamDist() != 0) {
					/*if(abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) > 50 || abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 40)
					{
						if (math.getSpeed() > 150) {
							math.setSpeed(150);
						}
					}*/
					
					if(abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) < 40) {
						if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 40)  
						{
							math.setVector(0 - gyro.getTargetRobotAngle(), speedFast);
						}
					}
					
					if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 25)  
					{
						math.setVector(0 - gyro.getTargetRobotAngle(), speedFast);
					}
		
					if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) > autX)
					{
						math.addVector(90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
					}
					if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) < -autX)
					{
						math.addVector(-90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
					}
				}
				
				if (cam.getAnotherCamDist() != 0) {
					/*if(abs(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3)) > 50 || abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 40)
					{
						if (math.getSpeed() > 150) {
							math.setSpeed(150);
						}
					}*/
					
					if(abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 20)  
					{
						math.setVector(180 - gyro.getTargetRobotAngle(), speedFast);
					}
					if(cam.getAnotherCamDist() < 45)  
					{
						if(abs(float(cam.getAnotherCamAngle())) > 45) {
							math.addVector((180 + cam.getAnotherCamAngle()) - gyro.getTargetRobotAngle(), math.getSpeed() + speedSlow);
						} else {
							math.addVector(180 - gyro.getTargetRobotAngle(), math.getSpeed() + speedSlow);
						}
					}
					
					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) < -autX)  
					{
						math.addVector(-90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedSlow);
					}
					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) > autX)  
					{
						math.addVector(90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedSlow);
					}
				}
				k = gyro.getAngle();
			}
			else if (CODE == GOALKEEPER) {
				int speedGK = 230; 			
				int speedGKSlow = 230; 							
				math.setVector(0, 0);
				int angAng = 130;
				int tAng = 0;
				if (abs(float(tsop.getAngle())) > 90) {
					tAng = FAng(-tsop.getAngle()); 
				} else {
					tAng = FAng(tsop.getAngle() - cam.getCamAngle()); 
				}
				
				int yCoord = abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3));

				math.setVector(0, 0);
				gyro.setMaxSpeed(256);
				dribblerOn = 0;
				cam.setGyroAng(gyro.getDev());
				
				
				if(tsop.isCanSee()){
					if (abs(float(tAng)) < 170 || abs(float(tAng)) > 10) {
						if (abs(float(cam.getCamAngle())) > angAng) {
							if (tAng < 0) {
								math.setVector(90, getGKSpeed(tAng, tsop.getDist(), speedGK));
							} else {
								math.setVector(-90, getGKSpeed(tAng, tsop.getDist(), speedGK));
							}
						} else {
							if (tAng < 0) {
								if (cam.getCamAngle() < 0) {
									math.setVector(cam.getCamAngle() - 90, getGKSpeed(tAng, tsop.getDist(), math.min(speedGK, speedGK * yCoord / 30)));
								} else {
									math.setVector(cam.getCamAngle() - 90, getGKSpeed(tAng, tsop.getDist(), speedGK));
								}
							} else {
								if (cam.getCamAngle() > 0) {
									math.setVector(cam.getCamAngle() + 90, getGKSpeed(tAng, tsop.getDist(), math.min(speedGK, speedGK * yCoord / 30)));
								} else {
									math.setVector(cam.getCamAngle() + 90, getGKSpeed(tAng, tsop.getDist(), speedGK));
								}
							}
							if (yCoord < 20) {//(abs(float(cam.getCamAngle())) < 130) {
								if (cam.getCamAngle() < 0 && tAng < 0) {
									math.setVector(0, 0);
								}
								if (cam.getCamAngle() > 0 && tAng > 0) {
									math.setVector(0, 0);
								}
							}
						}
					} else {
						math.setVector(0, 0);
					}
				} else {
					if (abs(float(cam.getCamAngle())) < 170) {
						if (cam.getCamAngle() > 0) {
							math.setVector(cam.getCamAngle() - 90, speedGKSlow);
						} else {
							math.setVector(cam.getCamAngle() + 90, speedGKSlow);
						}
					} else {
						math.setVector(0, 0);
					}
				}
				
				//STOP FLAGS
				/*if (yCoord < 30) { //(abs(float(cam.getCamAngle())) < 130) {
					if((abs(double(cam.getCamAngle())) > 90 && tsop.getDist() > 50) && tAng < 0) {
						math.setVector(0, 0);
					}
					if((abs(double(cam.getCamAngle())) > 90 && tsop.getDist() > 50) && tAng > 0) {
						math.setVector(0, 0);
					}
				}*/
				
				if(tsop.isCanSee()) {
					if (abs(float(tAng)) > 160 && tsop.getDist() > 70 && abs(float(tsop.getAngle())) < 90) { 
						if (gkGo == 0) { 
							gkGo = 1;
							timer = millis();
						}
						if (gkGo == 1 && millis() - timer > 7000) { 
							gkGo = 2;
							timer = millis();
						}
						if (gkGo == 2 && millis() - timer < 7600) { 
							math.setVector(tsop.getAngle(), 200);
						}
						if (gkGo == 2 && millis() - timer >= 7600) { 
							gkGo = 0;
						}
					}
				}

				if (gkGo != 2) {
					if (abs(float(cam.getCamAngle())) > angAng) {	
						if (yCoord > 25) {
							math.addVector(180, getGKSpeedSlow(yCoord - 25, speedGK));
						} else {
							math.addVector(0, getGKSpeedSlow(yCoord - 25, speedGK));
						}
					} else {
						if (cam.getCamDist() > 35) {
							math.addVector(cam.getCamAngle(), getGKSpeedSlow(cam.getCamDist() - 40, speedGK));	
						} else {
							math.addVector(cam.getCamAngle() + 180, getGKSpeedSlow(cam.getCamDist() - 40, speedGK));
						}
					}
				} else {
					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) < -50)  
					{
						math.addVector(-90 - gyro.getTargetRobotAngle(), math.getSpeed() + 100);
					}
					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) > 50)  
					{
						math.addVector(90 - gyro.getTargetRobotAngle(), math.getSpeed() + 100);
					}
					if (abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) > 40 || tsop.getDist() < 30 || abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 70) {
						gkGo = 0;
					}
				}
				
				if (yCoord < 15) {
					math.setVector(0, speedGK);
				} 
				
				if (yCoord > 40) {
					math.addVector(180, speedGK);
				}
				
				testAng = tAng;
				//math.setVector(0, 0);
				k = gyro.getAngle();
			} else if (CODE == DEBUG) {
				/*dribblerOn = 0;
				if (tsop.isCanSee()) {
					if (tsop.getDist() > BALL_IS_REALLY_NEAR) {
						dribblerOn = 2;
					} else if (tsop.getDist() > BALL_IS_NEAR) {
						dribblerOn = 1;
					}
				}*/
			} else if (CODE == SNIPER) {
				int speed = 256 * 0.4;
				gyroFlag = 0;				
				math.setVector(0, 0);
				gyro.setMaxSpeed(256);
				dribblerOn = 0;
				cam.setGyroAng(gyro.getDev());
				gyro.setRotation(180);
				
				if (kickState == 0) {					 
					if (tsop.isCanSee() || ballCathed) {
						if (ballCathed) {
							float dX = cam.getCamDist() * sin(cam.getCamAngle() / 57.3);
							float dY = abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3));
							gyro.setMaxSpeed(50);
							math.setVector(cam.getAnotherCamAngle() > 0 ? -90 : 90, getSpeedChalandge(cam.getAnotherCamAngle(), speed));
							if (dY > 70) {
								math.addVector(0, getSpeedChalandge(abs(70. - dY), speed));
							} else {
								math.addVector(180, getSpeedChalandge(abs(70. - dY), speed));
							}
							dribblerOn = 2;
							
							if (abs(70 - dY) < 1 && abs(float(cam.getAnotherCamAngle())) < 1) {
								kickTimer = millis();
								kickState = 1;
								pwm = 260;
							}
							
							if (math.getSpeed() > speed) {
								math.setSpeed(speed);
							}
						} else {
							dribblerOn = 2;
							math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speed);
						}
					} else {
						math.setVector(0, 0);
					}
				} else {
					if (millis() - kickTimer < 1000) {
						//dribblerOn = 3;
						//pwm = 260 - (millis() - kickTimer) / 5;
						math.setVector(0, 0);
						dribblerOn = 2;
					} else if (millis() - kickTimer < 1100) {
						math.setVector(0, 0);
						dribblerOn = 0;
					} else if (millis() - kickTimer < 2000) {
						math.setVector(90, 250);
					} else {
						math.setVector(0, 0);
					}
				}
				k = gyro.getAngle();
			} else if (CODE == PENALTY) {
				if (millis() - penTimer < 6000) {
					math.setVector(0, 0);
					k = 0;
				} else {
					int speed = 256 * 0.4;
					gyroFlag = 0;				
					math.setVector(0, 0);
					gyro.setMaxSpeed(256);
					dribblerOn = 0;
					cam.setGyroAng(gyro.getDev());
					gyro.setRotation(0);
					if (kickState == 0) {					 
						if (tsop.isCanSee() || ballCathed) {
							if (ballCathed) {
									gyro.setRotation(180);
									gyro.setMaxSpeed(70);
									dribblerOn = 2;
									if (abs(float(gyro.getDevFromTarget())) < 5) {
										kickState = 1;
										kickTimer = millis();
									}
							} else {
								goBackFlag = 0;
								dribblerOn = 2;
								math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), speed);
							}
						} else {
							math.setVector(0, 0);
						}
					} else {
							gyro.setRotation(180);
							gyro.setMaxSpeed(70);
							dribblerOn = 2;
							if (millis() - kickTimer > 1000) {
								kickState = 0;
								dribblerOn = 0;
								kickSector = 0;
								ballCathed = 0;
								math.setVector(0, 0);
							} else {
								if (kickState == 1) {
									int dY = cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3);
									int dX = cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3);
									if (cam.getAnotherCamAngle() > 0) {
										kickSector = 1;
									} else {
										kickSector = 2;
									}
									kickState = 2;
									
									if (mKick == 1) {
										goBackTimer = millis();
									}
								} else if (kickState == 2) {
									switch(kickSector) {
									case 1:
										gyro.setRotation(-90 + cam.getAnotherCamAngle());
										break; 
									case 2:
										gyro.setRotation(90 + cam.getAnotherCamAngle());
										break;
									case 3:
										gyro.setRotation(25 + cam.getAnotherCamAngle());
										break;
									case 4:
										gyro.setRotation(-25 + cam.getAnotherCamAngle());
										break;
									case 5:
										gyro.setRotation(-25 + cam.getAnotherCamAngle());
										break;
									case 6:
										gyro.setRotation(25 + cam.getAnotherCamAngle());
										break;
									}
								}
							}
					}
					k = gyro.getAngle();
				}
			}
		} else {
			//dribbler.pwm(90);
			math.setVector(0, 0);
			timer = 0;
			f = false;
			k = 0;
			testAng = tsop.getDist();
			kickSector = 0;
			kickState = 0;
			penTimer = millis();
		}		
		if (dribblerOn == 0) {
			dribbler.pwm(120);
		} else if (dribblerOn == 1) {
			dribbler.pwm(260);
		} else if (dribblerOn == 2) {
			dribbler.pwm(260);
		} else if (dribblerOn == 3) {
			dribbler.pwm(pwm);
		}
		math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
		
		if (kickSector == 0) {
			robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k, math.getPeriod());
		} else {
			if (kickSector == 1 || kickSector == 6) {
				gyro.setMaxSpeed(160);
				k = gyro.getRotationKForRotateToBall(5, 3, 0.1);
			} else {
				gyro.setMaxSpeed(180);
				k = gyro.getRotationKForRotateToBall(3, 1.6, 0.05);
			}
			robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k, math.getPeriod());
		}
	}
}