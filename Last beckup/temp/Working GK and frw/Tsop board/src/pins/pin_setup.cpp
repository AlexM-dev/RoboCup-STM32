#include "pins_setup.h"

#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3
#define CHANNEL4 4
Pin::Pin(uint16_t pinNumber,
        GPIO_TypeDef* GPIOx,
        GPIOMode_TypeDef GPIO_Mode,
        TIM_TypeDef* timer1,
        uint32_t channel1,
        uint32_t tim1,
        uint16_t GPIO_PinSource,
        uint32_t period,
        uint32_t prescaler)
{
    m_pinNumber = pinNumber;
    m_GPIOx = GPIOx;
    m_mode = GPIO_Mode;
		m_timer1 = timer1;
		m_channel1 = channel1;
		m_tim1 = tim1;
		m_GPIO_PinSource = GPIO_PinSource;
		//m_GPIO_AF_TIM = GPIO_AF_TIM;
    m_period = period;
		m_prescaler = prescaler;
    
}

void Pin::pinInit()
{
    if (m_GPIOx == GPIOA)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (m_GPIOx == GPIOB)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (m_GPIOx == GPIOC)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (m_GPIOx == GPIOD)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    else if (m_GPIOx == GPIOE)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    else if (m_GPIOx == GPIOF)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    else if (m_GPIOx == GPIOG)RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef pin;
	pin.GPIO_Pin = m_pinNumber;
	pin.GPIO_Speed = GPIO_Speed_2MHz;
	pin.GPIO_Mode = m_mode;
	GPIO_Init(m_GPIOx, &pin);
	//if(m_mode == GPIO_Mode_AF)GPIO_PinAFConfig(m_GPIOx, m_GPIO_PinSource, m_GPIO_AF_TIM);
}

GPIO_TypeDef * Pin::getGPIOx()
{
    return m_GPIOx;
}

uint16_t  Pin::getPinNumber()
{
    return m_pinNumber;
}

void Pin::pwmInit()
{
		if(m_tim1 != RCC_APB2Periph_TIM9)RCC_APB1PeriphClockCmd(m_tim1, ENABLE);
		else RCC_APB2PeriphClockCmd(m_tim1, ENABLE);
		TIM_TimeBaseInitTypeDef tim;
		tim.TIM_Prescaler = m_prescaler;
		tim.TIM_CounterMode = TIM_CounterMode_Up;
		tim.TIM_Period = m_period;
		tim.TIM_ClockDivision = TIM_CKD_DIV1;
		tim.TIM_RepetitionCounter = 0; 
		TIM_TimeBaseInit(m_timer1, &tim);
		//TIM_SelectOutputTrigger(m_timer1, TIM_TRGOSource_Update);
		TIM_Cmd(m_timer1, ENABLE);
		TIM_OCInitTypeDef ch;
		ch.TIM_OCMode = TIM_OCMode_PWM1;
		ch.TIM_OutputState = TIM_OutputState_Enable;
		ch.TIM_OutputNState = TIM_OutputNState_Disable;
		ch.TIM_Pulse = 0;
		ch.TIM_OCPolarity = TIM_OCPolarity_High;
		ch.TIM_OCNPolarity = TIM_OCNPolarity_High;
		ch.TIM_OCIdleState = TIM_OCIdleState_Set;
		ch.TIM_OCNIdleState = TIM_OCNIdleState_Set;
		//volatile int32_t reg = 50;

		if (m_channel1 == CHANNEL1){
		TIM_OC1Init(m_timer1, &ch);
		}
		else if (m_channel1 == CHANNEL2){
		TIM_OC2Init(m_timer1, &ch);
		}
		else if (m_channel1 == CHANNEL3){
		TIM_OC3Init(m_timer1, &ch);
		}
		else if (m_channel1 == CHANNEL4){
		TIM_OC4Init(m_timer1, &ch);

	}
}

	void Pin::pwm(uint32_t pulse)
	{	
		 if (m_channel1 == CHANNEL1)TIM_SetCompare1(m_timer1,pulse);
		 else if (m_channel1 == CHANNEL2)TIM_SetCompare2(m_timer1,pulse);
		 else if (m_channel1 == CHANNEL3)TIM_SetCompare3(m_timer1,pulse);
		 else if (m_channel1 == CHANNEL4)TIM_SetCompare4(m_timer1,pulse);
	}
		//TIM_OC4Init(timer2, &ch);
    void Pin::setBit()
    {
        m_GPIOx->ODR |= (m_pinNumber);
        //GPIO_SetBits(m_GPIOx, m_pinNumber);
    }

    void Pin::resetBit()
    {
        m_GPIOx->ODR &= ~(m_pinNumber);
        //GPIO_ResetBits(m_GPIOx, m_pinNumber);
    }
