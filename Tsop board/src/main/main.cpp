#include "project_config.h"	
#include "pins_setup.h"
#include "uart.h"
#include <adc.h>
#include <dma.h>
#include <multiplexer.h>
#include <math.h>
#include <time_service.h>
#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4
#define SORT_SIZE 15
#define PI 3.141592653589793

void delay(unsigned int nCount);
double toRadians(double degs);
double getAngle(double x, double y);
double getX(int tsop_results[]);
double getY(int tsop_results[]);
double getDist(double x, double y);
void qsortRecursive(int *mas, int size);
uint8_t crc8(uint8_t* data, int len);
//GPIO_InitTypeDef GPIO_InitStruct;

int realDistance(int x) {
		int goalCoords[18][2] = {{0, 100},
              {10, 80},
              {20, 74},
              {30, 62},
							{40, 54},
              {50, 47},
              {60, 38},
              {70, 26},
              {80, 10},
              {90, 0}};
		int xOne = 0, xTwo = 0, yOne = 0, yTwo = 0, fSign = 1, xAbs, masLen = 18;
    if (x < 0) {
        xAbs = x*-1;
        fSign = -1;
		} else {
        xAbs = x;
		}
    for (int i = 0; i < masLen; i++) {
			if (goalCoords[i][1] <= xAbs && xAbs < goalCoords[i + 1][1]) {
					xOne = goalCoords[i][1];
					xTwo = goalCoords[i + 1][1];
					yOne = goalCoords[i][0];
					yTwo = goalCoords[i + 1][0];
					return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign;
			}
		}
      
    if (goalCoords[masLen - 1][1] <= xAbs) {
        xOne = goalCoords[masLen - 2][1];
        xTwo = goalCoords[masLen - 1][1];
        yOne = goalCoords[masLen - 2][0];
        yTwo = goalCoords[masLen - 1][0];
		}
    return (((xAbs-xOne)*(yTwo-yOne))/(xTwo-xOne) + yOne)*fSign;
}

int main (void)
{ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	time_service::init();
	time_service::startTime();
	Pin tx(GPIO_Pin_9,
				 GPIOA,
				 GPIO_Mode_AF_PP,
				 TIM1,
				 CHANNEL2,
				 RCC_APB2Periph_TIM1,
				 GPIO_PinSource9,
         4096,
				 1);	
	tx.pinInit();
  Pin rx(GPIO_Pin_10,
				 GPIOA,
				 GPIO_Mode_AF_PP,
				 TIM1,
				 CHANNEL3,
				 RCC_APB2Periph_TIM1,
				 GPIO_PinSource10,
         4096,
         1);	
	rx.pinInit();
	Uart::uartInit();
	/*Pin sclk(GPIO_Pin_10,
		GPIOB,
		GPIO_Mode_AF_PP,
		TIM2,
		CHANNEL4,
		RCC_APB1Periph_TIM2,
		GPIO_PinSource10,
		4096,
		1);	
	sclk.pinInit();
	Pin mosi(GPIO_Pin_10,
		GPIOB,
		GPIO_Mode_AF_PP,
		TIM2,
		CHANNEL4,
		RCC_APB1Periph_TIM2,
		GPIO_PinSource10,
		4096,
		1);	
	mosi.pinInit();
	Pin miso(GPIO_Pin_10,
		GPIOB,
		GPIO_Mode_AF_PP,
		TIM2,
		CHANNEL4,
		RCC_APB1Periph_TIM2,
		GPIO_PinSource10,
		4096,
		1);	
	miso.pinInit();
	Pin ss(GPIO_Pin_10,
		GPIOB,
		GPIO_Mode_AF_PP,
		TIM2,
		CHANNEL4,
		RCC_APB1Periph_TIM2,
		GPIO_PinSource10,
		4096,
		1);	
	ss.pinInit*/
	
	static Pin state1(GPIO_Pin_12,
		GPIOB,
		GPIO_Mode_Out_PP,
		TIM1,
		CHANNEL1,
		RCC_APB2Periph_TIM1,
		GPIO_PinSource10,
		4096,
		1);				 
	static Pin state2(GPIO_Pin_13,
		GPIOB,
		GPIO_Mode_Out_PP,
		TIM1,
		CHANNEL1,
		RCC_APB2Periph_TIM1,
		GPIO_PinSource10,
		4096,
		1);
	static Pin state3(GPIO_Pin_14,
		GPIOB,
		GPIO_Mode_Out_PP,
		TIM2,
		CHANNEL4,
		RCC_APB2Periph_TIM1,
		GPIO_PinSource10,
		4096,
		1);
	static Pin state4(GPIO_Pin_15,
		GPIOB,
		GPIO_Mode_Out_PP,
		TIM2,
		CHANNEL4,
		RCC_APB2Periph_TIM1,
		GPIO_PinSource10,
		4096,
		1);
	static Pin lightValue(GPIO_Pin_1,
		GPIOA,
		GPIO_Mode_AIN,
		TIM2,
		CHANNEL2,
		RCC_APB1Periph_TIM2,
		GPIO_PinSource1,
		4096,
		1);
	lightValue.pinInit();
	static Pin lightValue_2(GPIO_Pin_3,
		GPIOA,
		GPIO_Mode_AIN,
		TIM2,
		CHANNEL4,
		RCC_APB1Periph_TIM2,
		GPIO_PinSource3,
		4096,
		1);
	lightValue_2.pinInit();
	
	Adc mpAdc(ADC1, 2, 0, RCC_APB2Periph_ADC1, lightValue);
	mpAdc.startAdc();
	Dma mpDma(RCC_AHBPeriph_DMA1, mpAdc, DMA1_Channel1, 2);
	mpDma.adcInitInDma();
	
	/*Pin ssArray[1];
	ssArray[0] = ss;
	spi spi(0, sclk, mosi, miso, 1, ssArray);
	spi.spiInit();*/

	Multiplexer mp(state4, state3, state2, state1, mpDma, 2, DMA1_Channel1);
  int data[32];
	int binData[32];
	int nonF1[SORT_SIZE], nonF2[SORT_SIZE];
	volatile int test1, test2;
	volatile double dist, angle;
	double x, y, binX, binY;
	int i;
	volatile uint8_t a;
	
	while (1)
	{
		for(i = 0; i < 16; i++)
		{
 			for(int j = 0; j < SORT_SIZE; j++)
			{
			  nonF1[j] = mp.getPh1Value(i);
				nonF2[j] = mp.getPh2Value(i);
			}
			qsortRecursive(nonF1, SORT_SIZE);
			qsortRecursive(nonF2, SORT_SIZE);

			data[i] = nonF1[SORT_SIZE / 2];
			data[16 + i] = nonF2[SORT_SIZE / 2];
			
			if (data[i] > 2500) {
				binData[i] = 0;
			} else { 
				binData[i] = 1;
			}
			
			if (data[16 + i] > 2500) {
				binData[16 + i] = 0;
			} else {
				binData[16 + i] = 1;
			}
		}
		//data[20] = 4096;
		//binData[20] = 0;
		//20 here = 5 real
		x = getX(data);
		y = getY(data);
		
		binX = getX(binData);
		binY = getY(binData);
	
	
		dist = getDist(binX, binY);
		angle = getAngle(x, y) - 135 + 180;
		while (angle < 0) 
			angle += 360;
		while (angle >= 360) 
			angle -= 360;
	
		angle = uint8_t(angle / 2);
		dist = uint8_t(dist * 10);
		if (dist > 255) dist = 255;
		if (angle > 255) angle = 255;
		Uart::write(0xff);
		Uart::write(angle);
		Uart::write(dist); 
		uint8_t dt[2];
		dt[0] = angle;
		dt[1] = dist;
		Uart::write(crc8(dt, 2)); 
	}
}

uint8_t crc8(uint8_t* data, int len)
{
    int crc = 0xFF, i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (char)((crc << 1) ^ 0x31);
            else crc <<= 1;
        }
    }
    return crc;
}

double toRadians(double degs) {
  return degs * PI / 180;
}

double getAngle(double x, double y) {
	double angle;
	angle = atan2(x, y) * 57.2928;
  if (angle < 0)
    angle = 360 + angle;

  return angle;
}

double getX(int *tsop_results) {
  double x1 = 0;
  for (int i = 0; i < 32; ++i)
    x1 += sin(toRadians(double(11.25 * i))) * double(tsop_results[i]);
  return x1;
}

double getY(int *tsop_results) {
  double y1 = 0;
  for (int i = 0; i < 32; ++i)
    y1 += cos(toRadians(double(11.25 * i))) * double(tsop_results[i]);
  return y1;
}

double getDist(double x, double y) {
  return sqrt(abs(x * x + y * y));
}

void qsortRecursive(int *mas, int size) {
    int i = 0;
    int j = size - 1;

    int mid = mas[size / 2];

    do {
        while(mas[i] < mid) {
            i++;
        }
        while(mas[j] > mid) {
            j--;
        }

        if (i <= j) {
            int tmp = mas[i];
            mas[i] = mas[j];
            mas[j] = tmp;

            i++;
            j--;
        }
    } while (i <= j);


    if(j > 0) {
        qsortRecursive(mas, j + 1);
    }
    if (i < size) {
        qsortRecursive(&mas[i], size - i);
    }
}
