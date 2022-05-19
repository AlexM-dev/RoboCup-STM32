#include "project_config.h"	
#include "pins_setup.h"
#include "stm32f103_SPI.h"
#include <adc.h>
#include <dma.h>
#include <multiplexer.h>
#include <math.h>
#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4
#define SORT_SIZE 20
#define PI 3.141592653589793

void delay(unsigned int nCount);
double toRadians(double degs);
double getAngle(double x, double y);
double getX(int tsop_results[]);
double getY(int tsop_results[]);
double getDist(double x, double y);
void qsortRecursive(int *mas, int size);
//GPIO_InitTypeDef GPIO_InitStruct;
 
int main (void)
{
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
		GPIOA,
		GPIO_Mode_Out_PP,
		TIM2,
		CHANNEL4,
		RCC_APB2Periph_TIM1,
		GPIO_PinSource10,
		4096,
		1);				 
	static Pin state2(GPIO_Pin_11,
		GPIOA,
		GPIO_Mode_Out_PP,
		TIM2,
		CHANNEL4,
		RCC_APB2Periph_TIM1,
		GPIO_PinSource10,
		4096,
		1);
	static Pin state3(GPIO_Pin_8,
		GPIOA,
		GPIO_Mode_Out_PP,
		TIM2,
		CHANNEL4,
		RCC_APB2Periph_TIM1,
		GPIO_PinSource10,
		4096,
		1);
	static Pin lightValue(GPIO_Pin_0,
		GPIOA,
		GPIO_Mode_AIN,
		TIM2,
		CHANNEL4,
		RCC_APB2Periph_TIM1,
		GPIO_PinSource10,
		4096,
		1);
	lightValue.pinInit();
	static Pin lightValue_2(GPIO_Pin_3,
		GPIOA,
		GPIO_Mode_AIN,
		TIM3,
		CHANNEL3,
		RCC_APB1Periph_TIM2,
		GPIO_PinSource10,
		4096,
		1);
	lightValue_2.pinInit();
	
	static Pin enable(GPIO_Pin_8,
	GPIOB,
	GPIO_Mode_Out_PP,
	TIM4,
	CHANNEL3,
	RCC_APB1Periph_TIM4,
	GPIO_PinSource8,
	4096,
	1);
	
	enable.pinInit();
	Adc mpAdc(ADC1, 2, 0, RCC_APB2Periph_ADC1, lightValue);
	mpAdc.startAdc();
	Dma mpDma(RCC_AHBPeriph_DMA1, mpAdc, DMA1_Channel1, 2);
	mpDma.adcInitInDma();
	
	/*Pin ssArray[1];
	ssArray[0] = ss;
	spi spi(0, sclk, mosi, miso, 1, ssArray);
	spi.spiInit();*/

	Multiplexer mp(state1, state2, state3, mpDma, 2, DMA1_Channel1);
	int data[16];
	int nonF1[SORT_SIZE], nonF2[SORT_SIZE];
	volatile double x, y, dist, angle;
	int i;
	initSPI(_SPI1, SLAVE, 8, 2);

	while (1)
	{
		for(i = 0; i < 8; i++)
		{
			//data[i] = mp.getPh1Value(i);
			//data[8 + i] = mp.getPh2Value(i);
			
			for(int j = 0; j < SORT_SIZE; j++)
			{
			  nonF1[j] = mp.getPh1Value(i);
				nonF2[j] = mp.getPh2Value(i);
			}
			qsortRecursive(nonF1, SORT_SIZE);
			qsortRecursive(nonF2, SORT_SIZE);

			data[i] = nonF1[SORT_SIZE / 2];
			data[8 + i] = nonF2[SORT_SIZE / 2];
			if (data[i] > 1100)
				data[i] = 0;
			else
				data[i] = 1;
			if (data[8 + i] > 1100)
				data[8 + i] = 0;
			else
				data[8 + i] = 1;
		}
	
		x = getX(data);
		y = getY(data);
	
	
		//dist = getDist(x, y);
		angle = getAngle(x, y);
		writeTxBufSPI(_SPI1, 0xff);	
		writeTxBufSPI(_SPI1, angle / 2);	
		
		enable.pwm(3000);
		
		//Uart::write(dist);
	}
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
  for (int i = 0; i < 16; ++i)
    x1 += sin(toRadians(double(22.5 * i))) * double(tsop_results[i]);
  return x1;
}

double getY(int *tsop_results) {
  double y1 = 0;
  for (int i = 0; i < 16; ++i)
    y1 += cos(toRadians(double(22.5 * i))) * double(tsop_results[i]);
  return y1;
}

double getDist(double x, double y) {
  return sqrt(abs(x * x + y * y));
}

void qsortRecursive(int *mas, int size) {
    //????????? ? ?????? ? ? ????? ???????
    int i = 0;
    int j = size - 1;

    //??????????? ??????? ???????
    int mid = mas[size / 2];

    //????? ??????
    do {
        //????????? ????????, ???? ??, ??????? ????? ?????????? ? ?????? ?????
        //? ????? ????? ??????? ??????????(????????? ?? ?????) ????????, ??????? ?????? ????????????
        while(mas[i] < mid) {
            i++;
        }
        //? ?????? ????? ?????????? ????????, ??????? ?????? ????????????
        while(mas[j] > mid) {
            j--;
        }

        //?????? ???????? ???????
        if (i <= j) {
            int tmp = mas[i];
            mas[i] = mas[j];
            mas[j] = tmp;

            i++;
            j--;
        }
    } while (i <= j);


    //??????????? ??????, ???? ????????, ??? ???????????
    if(j > 0) {
        //"????? ?????"
        qsortRecursive(mas, j + 1);
    }
    if (i < size) {
        //"????? ?????"
        qsortRecursive(&mas[i], size - i);
    }
}
