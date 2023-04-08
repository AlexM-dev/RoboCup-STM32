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

#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4

#define GOALKEEPER 0
#define ATTACKER 1
#define SNIPER 3
#define PENALTY 4
#define DEBUG 5
#define CODE ATTACKER

#define Ec 2.7182818284
#define BALL_SENSOR_THRESHOLD 1700

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

double convertDist(double dist) {
			double maxDist = 230;
			double v = (dist-maxDist)/maxDist + 1;
			if (v > 1) 
				v = 1;
			if (v < 0) 
				v = 0;
      return v;
}

float AngleOffset(float angle, float dist){
	double angK = 0.04 * pow(double(Ec), double(0.15 * abs(angle)));
	if (angK > 90)
		angK = 90;
	dist = convertDist(dist);
	double distK = 0.02 * pow(double(Ec), double(4.5 * dist));
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

int getGKSpeed (int ang, int maxSpeed) {
	int sp = abs(abs(float(ang)) - 180) * abs(abs(float(ang)) - 180) / 8;
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
									GPIO_PuPd_NOPULL);
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
									GPIO_PuPd_NOPULL);
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
									GPIO_PuPd_NOPULL);
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
									 GPIO_PuPd_NOPULL);
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
									GPIO_PuPd_NOPULL);
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
									GPIO_PuPd_NOPULL);
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
									GPIO_PuPd_NOPULL);
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
									GPIO_PuPd_NOPULL);
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
							GPIO_PuPd_NOPULL);
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
						 GPIO_PuPd_NOPULL);
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
						 GPIO_PuPd_NOPULL);
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
						 GPIO_PuPd_NOPULL); 
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
			 GPIO_PuPd_NOPULL);
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
			 GPIO_PuPd_NOPULL);
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
	
	
	//IMU imu;
	//imu.init(_SPI1, PA1);
	
	int32_t sp1 = 0;
	int32_t sp2 = 0; 
	int32_t sp3 = 0;
	int32_t sp4 = 0;
	
	bool pressFlag[4] = {0};
	
	int k = 0;
	int gkGo = 0;
	unsigned long timer = 0;
	volatile bool f = false, updateF = false;
	int state = 0;
	int goalSide = 0;
	int32_t sp = 0, ang = 0;
	math.setPeriod(period);
	
	volatile int testAng = 0; 
	
	bool runFlag = 0;
	bool gyroFlag = 0;
	/*delay(1000);
	dribbler.pwm(90);
	delay(1000);*/
	/*dribbler.pwm(200);
	delay(1000);
	dribbler.pwm(90);*/
	while(1)
	{
		ballSensorRes = (int)ADC1->DR;
		if ((ballCathed && ballSensorRes > BALL_SENSOR_THRESHOLD) || (!ballCathed && ballSensorRes < BALL_SENSOR_THRESHOLD)) { 
			if (!ballCathedFlag) {
				ballCathedFlag = 1;
				ballSensorTimer = millis();
			} else {
				if (millis() - ballSensorTimer > 100) {
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
		}
		
		if(changeSideButton.readPin()){
			if (pressFlag[1] == 0) {
				pressFlag[1] = 1;
				dribbler.pwm(200);
			} else {
				pressFlag[1] = 0;
				dribbler.pwm(90);
			}
		}
		
		if(imuReset.readPin()){
			gyro.read();
			gyro.setRotationByGyro();
			gyro.setRotation(0);
		}
		if (runFlag) {	
			//testAng = tsop.getDist();			
			if(CODE == ATTACKER) {
				int speedFast = 256 * 0.5;
				int speedSlow = 256 * 0.3;	
				gyroFlag = 0;				
				math.setVector(0, 0);
				//tsop.setGyroAng(0);
				gyro.setMaxSpeed(256);
				
				if (cam.getAnotherCamDist() != 0 && 1 == 0) {
					cam.setGyroAng(gyro.getDev());
					gyro.setRotation(cam.getAnotherCamAngle());
					//gyro.s
				} else {
					cam.setGyroAng(0);
					gyro.setRotation(0);
				}
				 
				if (tsop.isCanSee()) {
					if (ballCathed) {
						math.setVector(0, speedFast);
					} else {
						math.setVector(tsop.getAngle() * 1.3, speedFast); //AngleOffset(tsop.getAngle(), tsop.getDist())
					}
				} else {
					math.setVector(0, 0);
				}
				math.setVector(0, 0);
				
				/*if (tsop.isCanSee()) {
					gyroFlag = 1;
					gyro.setRotation(tsop.getAngle());
					tsop.setGyroAng(gyro.getDev());
					math.setVector(tsop.getLocalAngle(), speedFast);
				} else {
					gyroFlag = 0;
					gyro.setRotation(0);
					tsop.setGyroAng(0);
					math.setVector(0, 0);
				}*/
				/*if (f == 0) {
					timer = millis();
					f = 1;
				} else {
					if (millis() - timer > 3000) {
						gyro.setRotation(90);
						math.setVector(0, 0);
					} else if (millis() - timer > 2750) {
						gyro.setRotation(90);
						math.setVector(90, speedFast);
					} else {
						math.setVector(90, speedFast);
					}
				}*/
				
				/*if (tsop.isCanSee()) {
					//testAng = tsop.getAngle();
					math.setAngle(tsop.getAngle() * 1.5);
					
					if (abs(float(tsop.getAngle())) < 15) {
						math.setAngle(tsop.getAngle());
						math.setSpeed(speedFast);
					} else {
						mat.setSpeed(speedSlow);
					}
				}*/
				/*if (gyroFlag == 0) {
					k = gyro.getAngle();
				} else {
					if (abs(float(gyro.getDevFromTarget())) < 90) {
						k = gyro.getRotationKForRotateToBall(0.75);
					} else {
						k = gyro.getRotationKForRotateToBall(1);
					}
				}*/
				k = gyro.getAngle();
			} else if (CODE == GOALKEEPER) {
				int speedGK = 150; 			
				int speedGKSlow = 100; 							
				math.setVector(0, 0);
				volatile int tAng = FAng(tsop.getAngle() - cam.getCamAngle()); 
				
				if(tsop.isCanSee()) {
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
							if (abs(float(cam.getCamAngle())) < 130) {
								if(cam.getCamAngle() < 0 && tAng < 0) {
									math.setVector(0, 0);
								}
								if(cam.getCamAngle() > 0 && tAng > 0) {
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
									math.setVector(tsop.getAngle() + (tsop.getAngle() > 0 ? 60 : -60), speedGKSlow);
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
								math.setVector(180, speedGKSlow);
							} else {
								math.setVector(cam.getCamAngle(), speedGKSlow);
							}
						}
						if (abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 25) {
								math.addVector(180, speedGKSlow);
						}
						if (abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 15) {
								math.addVector(0, speedGKSlow);
						}
					} else {
						if (cam.getCamDist() > 45) {
							math.addVector(cam.getCamAngle(), speedGKSlow);	
						}
						if (cam.getCamDist() < 35) {
							math.addVector(cam.getCamAngle() + 180, speedGKSlow);
						}
					}
				} else {
					if (abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) > 50 || tsop.getDist() < 20) {
						gkGo = 0;
					}
				}
				if (abs(float(tsop.getAngle())) > 170) {
							math.setVector(0, 0);
				}
				k = gyro.getAngle();
			}
		} else {
			//dribbler.pwm(90);
			math.setVector(0, 0);
			timer = 0;
			f = false;
			k = 0;
		}		
		math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
		robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k, math.getPeriod());
	}
//	int mf[1024]z===
//	int c = 0;
		/*while(1)
	{
		robot.move(3000, 3000, 3000, 3000);
		if(0 == 1 && button.readPin()){
			gyro.read();
			gyro.setRotationByGyro();
		}
		tsop.read();
		cam.readData();
		gyro.read();
		if(0==1 && switchPin.readPin())
		{
			if(CODE == ATTACKER) {
				int speedFast = 2500;
				int speedSlow = 3000;			
				volatile int tAng = tsop.getAngle() - cam.getCamAngle();
				math.setVector(0, 0);
				cam.setGyroAng(0);
				gyro.setRotation(0);
				tsop.setGyroAng(0);
				gyro.setMaxSpeed(4096);
				
				if (cam.getCamDist() != 0) {
					cam.setGyroAng(gyro.getDev());
				}
				
				if (tsop.isCanSee()) {
					if(abs(float(tsop.getAngle())) > 20 || 1 == 1) {
						math.setAngle(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()));
						if ((abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) < 30 && cam.getCamDist() != 0) || (abs(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3)) < 30 && cam.getAnotherCamDist() != 0)) {
							if (abs(float(tsop.getAngle())) < 30) {
								math.setSpeed(speedFast);
							} else {
								math.setSpeed(speedFast);
							}
						} else {
							if (abs(float(tsop.getAngle())) < 30) {
								math.setSpeed(speedFast);
							} else {
								math.setSpeed(speedFast);
							}
						}
					} else {
						if (abs(cam.getCamDist() * cos(gyro.getTargetRobotAngle() / 57.3)) < 60) {
							math.setVector(0, speedSlow);
						} else {
							math.setVector(0, speedFast);
						}
					}
				} else {
					math.setVector(0,0);
				}
				
//				if (tsop.isCanSee()) {
//					tsop.setGyroAng(gyro.getDev());
//				}
// 				
//				if(true) { //Ball cathed
//					double x = cam.getCamDist() * sin(cam.getCamAngle() / 57.3);
//					double y = 85 - cam.getCamDist() * cos(cam.getCamAngle() / 57.3);
//					if (state == 0) {
//						math.setStartDot(x, y);
//						state = 1;
//						timer = millis();
//					} else {
//						//math.setAngle(atan2(y, x) * 57.3);
//						if (millis() - timer > 0) {
//							math.getVecFromTr(x, y, millis() - timer, sp, ang);
//							timer = millis();
//						}
//						math.setSpeed(sp);
//						math.setAngle(ang - gyro.getDev());
//						if (math.isArrived(x, y)) {
//							state = 2;
//						}
//					}
//				} else {
//					state = 0;
//					if (tsop.isCanSee()) {
//						if(abs(float(tsop.getDist())) > 50) {
//							if(abs(double(gyro.getTargetRobotAngle())) < 15) {
//								math.setVector(0, 2500);
//							} else {
//								if(gyro.getTargetRobotAngle() > 0) {
//									math.setVector(90, 2000);
//								} else {
//									math.setVector(-90, 2000);
//								}
//							}						
//						} else {
//							math.setVector(0, 2500);
//						}
//					} else {
//						math.setVector(0, 0);
//					}
//				}
				
				//Line detecting
//				if(cam.getCamDist() != 0) 
//				{
//					if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) > 42)
//					{
//						math.addVector(90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
//					}
//					if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) < -42)
//					{
//						math.addVector(-90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
//					}
//					if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 10)	
//					{
//						math.setVector(180 - gyro.getTargetRobotAngle(), speedFast);
//					}
//					if(cam.getCamDist() < 40)	
//					{
//						math.addVector(180 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
//					}
//				}	
//				if(cam.getAnotherCamDist() != 0) 				
//				{
//					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) < -42)	
//					{
//						math.addVector(-90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
//					}
//					if(cam.getAnotherCamDist() * sin(cam.getAnotherCamAngle() / 57.3) > 42)	
//					{
//						math.addVector(90 - gyro.getTargetRobotAngle(), math.getSpeed() + speedFast);
//					}
//					if(abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 55)
//					{
//						math.setVector(0 - gyro.getTargetRobotAngle(), speedFast);
//					}
//				}
					
				k = gyro.getAngle();
				if (abs(float(gyro.getDevFromTarget())) > 30 && state != 3)
					math.setSpeed(0);
				
				math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
				volatile int a = math.getAngle();
				robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
				//
			} 
			else if(CODE == GOALKEEPER) {
				int speedGK = 3000; 				
				math.setVector(0, 0);
				volatile int tAng = FAng(tsop.getAngle() - cam.getCamAngle()); 
				
				if(tsop.isCanSee()) {
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
						math.setVector(0, 0);
					}
				} else {
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
				}
				if (abs(float(tsop.getAngle())) > 170) {
							math.setVector(0, 0);
				}
				math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
				k = gyro.getAngle();
				robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
			} else if(CODE == DEBUG) {
				math.setVector(0, 0);
				gyro.setRotation(0);
				math.setVector(tsop.getAngle() + AngleOffset(tsop.getAngle(), tsop.getDist()), 3500);
				volatile int a = math.getAngle();
				if (!tsop.isCanSee())
					math.setVector(0,0);
				math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
				k = gyro.getAngle();
				robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
			}
		}
		else
		{
			robot.move(0, 0, 0, 0);
			state = 0;
		}
	} */
}
