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
#define CODE ATTACKER


float getAngleMultipiller(float angle){
	if (angle < 15) 
		return 1.3;
	if (angle < 45) 
		return 1.9;
	if (angle < 90) 
		return 1.7;
	if (angle < 135) 
		return 1.5;
	return 1.3;
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
	bool viezd = false, viezdAct = false;
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
		volatile int a = tsop.getAngle();
		if(switchPin.readPin())
		{
			tsop.read();
			cam.readData();
			gyro.read();
			if(CODE == ATTACKER) {
				/*if(tsop.isCanSee()) {
						if(tsop.getDist() < 70)
							math.calculateSpeed(tsop.getAngle() * 1.2, 2500, sp1, sp2, sp3, sp4);
						else {
							if(abs(float(tsop.getAngle())) < 60)
							{
								if(abs(float(tsop.getAngle())) < 20) {
									math.setSpeed(1500);
									if(tsop.getDist() > 70)
									{
										math.setAngle(cam.getCamAngle());
										math.setSpeed(3500);
									}
									else
										math.setAngle(tsop.getAngle());
								} else {
										math.setVector(tsop.getAngle() * getAngleMultipiller(tsop.getAngle()), 1500);
								}
							} else {
								math.setVector(tsop.getAngle() * getAngleMultipiller(tsop.getAngle()), 1500);
							}
						}
				} else {
					math.setVector(0, 0);
				}*/
				
//				math.setAngle(cam.getCamAngle());
//				volatile int a = cam.getYAngle(), b = cam.getBAngle();
//				math.setSpeed(1000);
//				k = 0;	
				if(tsop.isCanSee()) {
					if (tsop.getDist() > 50) {
							math.setVector(tsop.getAngle() * 1.5, 3500);
					} else {
							math.setVector(tsop.getAngle(), 3500);
					}
				} else {
					math.setVector(tsop.getAngle(), 0);
				}
								if(abs(cam.getAnotherCamDist() * cos(cam.getAnotherCamAngle() / 57.3)) < 40)	
				{
					math.calculateSpeed(0, 2000, sp1, sp2, sp3, sp4);
				}
				
				if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 25)	
				{
					math.setVector(180, 2000);
				}
				if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) > 50)	
				{
					math.setVector(90, 2000);
				}
				if(cam.getCamDist() * sin(cam.getCamAngle() / 57.3) < -50)	
				{
					math.setVector(-90, 2000);
				}
				k = gyro.getAngle();
				math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
				robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
			} 
			else if(CODE == GOALKEEPER) {
				/*if(tsop.isCanSee()) {
					if(tsop.getAngle() <= 15 && tsop.getDist() >= 50 && !viezd && !viezdAct)
					{
						timer = millis();
						viezd = true;
					} else {
						if(tsop.getAngle() > 15 || tsop.getDist() < 50)
						{
							viezd = false;
							viezdAct = false;
						}
					}
					if(tsop.getAngle() * tsop.getAngle() < 1000)
						math.calculateSpeed(tsop.getAngle() > 0 ? 90 : -90, 1000, sp1, sp2, sp3, sp4);
					else {
						if(tsop.getAngle() * tsop.getAngle() > 4000)
							math.calculateSpeed(tsop.getAngle() > 0 ? 90 : -90, 4000, sp1, sp2, sp3, sp4);
						else 
							math.calculateSpeed(tsop.getAngle() > 0 ? 90 : -90, tsop.getAngle() * tsop.getAngle(), sp1, sp2, sp3, sp4);
					}
					if(abs(float(tsop.getAngle())) < 10)
					{
						math.calculateSpeed(0, 0, sp1, sp2, sp3, sp4);
					}
					
					if(millis() - timer > 3000 && viezd && !viezdAct) {
						//math.calculateSpeed(0, 3500, sp1, sp2, sp3, sp4);
						viezdAct = true;
						viezd = false;
						timer = millis();
					}
					
					if(millis() - timer <= 1000 && viezdAct) {
						math.calculateSpeed(0, 3500, sp1, sp2, sp3, sp4);
					}
					if(millis() - timer > 1000 && viezdAct) {
						viezdAct = false;
					}				
				} else {
					math.calculateSpeed(cam.getCamAngle() > 0 ? 90 : -90, 1500, sp1, sp2, sp3, sp4);
					if(abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) < 5)
					{
						math.calculateSpeed(0, 0, sp1, sp2, sp3, sp4);
					}
				}*/

				if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) < 30)	
				{
					math.calculateSpeed(0, 2500, sp1, sp2, sp3, sp4);
				}
		
			
				if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) > 35 && !viezdAct)	
				{
					math.calculateSpeed(180, 2500, sp1, sp2, sp3, sp4);
				}
				//}
				
				if(abs(cam.getCamDist() * sin(cam.getCamAngle() / 57.3)) > 20)	
				{
					math.calculateSpeed(cam.getCamAngle() > 0 ? 90 : -90, 2500, sp1, sp2, sp3, sp4);
				}
				k = gyro.getAngle();
				robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
			}
			else if(CODE == PENALTY) {
				tsop.read();
				cam.readData();
				gyro.read();
				
				if(abs(cam.getCamDist() * cos(cam.getCamAngle() / 57.3)) <  165)
				{
					math.calculateSpeed(-180, 1000, sp1, sp2, sp3, sp4);
				} else {
					if(abs(float(tsop.getAngle())) < 15) {
						math.calculateSpeed(cam.getCamAngle(), 3500, sp1, sp2, sp3, sp4);
					} else {
						math.calculateSpeed(getAngleMultipiller(tsop.getAngle()), 1000, sp1, sp2, sp3, sp4);
					}
				}
				k = gyro.getAngle();
				robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
			}
			else if(CODE == SNIPER) {
				
			}
			math.calculateSpeed(math.getAngle(), math.getSpeed(), sp1, sp2, sp3, sp4);
			robot.move(sp1 + k, sp2 + k, sp3 + k, sp4 + k);
		}
		else
		{
			robot.move(0, 0, 0, 0);
		}
	}
}
