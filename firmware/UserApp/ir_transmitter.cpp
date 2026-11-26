#include "ir_transmitter.hpp"

#include "gpio.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

// htim2 由 CubeMX 生成的 tim.c 定义，头文件已 extern "C" 声明

// 38kHz 载波参数（软件翻转）：周期 26μs(≈38.5kHz)，占空比 1/3（开 9μs / 关 17μs）。
// HS0038 类接收头必须收到 38kHz 调制的突发才能解调出 mark；纯电平直放对方收不到。
namespace
{
    constexpr uint32_t kCarrierOnUs  = 9;    // 载波开（1/3 占空比）
    constexpr uint32_t kCarrierOffUs = 17;   // 载波关
}

void IrTransmitter::play(const Signal& sig)
{
    const uint32_t count = sig.length();
    for (uint32_t i = 0; i < count; ++i)
    {
        const uint32_t seg    = sig.at(i);
        const uint32_t target = sig.us(seg);

        __HAL_TIM_SET_COUNTER(&htim2, 0);

        if (sig.level(seg))
        {
            // mark：软件翻转 PA1 生成 38kHz 载波。TIM2 32 位自由计数，
            // 忙循环按时翻转；中断仅让单次翻转延迟几 μs，载波周期与段总时长不受影响。
            bool high = true;
            uint32_t next = kCarrierOnUs;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
            uint32_t cnt;
            while ((cnt = __HAL_TIM_GET_COUNTER(&htim2)) < target)
            {
                if (cnt >= next)
                {
                    high = !high;
                    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
                    next = cnt + (high ? kCarrierOnUs : kCarrierOffUs);
                }
            }
        }
        else
        {
            // space：载波关闭
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
            while (__HAL_TIM_GET_COUNTER(&htim2) < target)
            {
            }
        }
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}
