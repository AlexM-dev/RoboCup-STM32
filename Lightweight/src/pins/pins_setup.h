#pragma once
#include "project_config.h"

class Pin
{
    public:
        Pin(uint16_t pinNumber,
            GPIO_TypeDef* GPIOx,
            GPIOMode_TypeDef GPIO_Mode,
						TIM_TypeDef* timer1,
						uint32_t channel1,
						uint32_t tim1,
						uint16_t GPIO_PinSource,
					  uint8_t GPIO_AF_TIM,
						uint32_t period,
						uint32_t prescaler,
						GPIOOType_TypeDef GPIO_OType_xx,
						GPIOPuPd_TypeDef GPIO_PuPd_xx);     
        void pinInit();
        GPIO_TypeDef* getGPIOx();
				uint16_t getPinNumber();
				void pwm(uint32_t pulse);
				void pwmInit();
				void setBit();
				void resetBit();
				bool readPin();
				uint32_t m_period;
    private:
        uint16_t m_pinNumber;
        GPIO_TypeDef * m_GPIOx;
        GPIOMode_TypeDef m_mode;
				TIM_TypeDef* m_timer1;
				uint32_t m_channel1;
				uint32_t m_tim1;
		    uint16_t m_GPIO_PinSource;
				uint8_t m_GPIO_AF_TIM;
				uint32_t m_prescaler;
				GPIOOType_TypeDef m_GPIO_OType_xx;
				GPIOPuPd_TypeDef m_GPIO_PuPd_xx;
};
