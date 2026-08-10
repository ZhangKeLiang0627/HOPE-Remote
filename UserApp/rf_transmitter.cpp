#include "rf_transmitter.hpp"

#include "gpio.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

// htim2 由 CubeMX 生成的 tim.c 定义，头文件已 extern "C" 声明

void RfTransmitter::play(const Signal& sig)
{
    const uint32_t count = sig.length();
    for (uint32_t i = 0; i < count; ++i)
    {
        const uint32_t seg    = sig.at(i);
        const uint32_t target = sig.us(seg);

        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,
                          sig.level(seg) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        while (__HAL_TIM_GET_COUNTER(&htim2) < target)
        {
        }
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}
