#include "ws2812b.hpp"

WS2812B::WS2812B(TIM_HandleTypeDef *_htim) : htim(_htim)
{
    // 尾部复位间隔同样填 0，即保持低电平
    for (uint32_t i = 0; i < WS2812B_BUF_SIZE; i++)
    {
        buffer[i] = 0;
    }
}

// 每个颜色位按 bit23→bit0 展开：1 记 T1H，0 记 T0H
void WS2812B::SetPixels(uint8_t _num, uint32_t _color)
{
    uint8_t i, j;
    for (j = 0; j < _num; j++)
    {
        for (i = 0; i < WS2812B_DATA_SIZE; i++)
        {
            buffer[i + j * WS2812B_DATA_SIZE] = ((_color << i) & 0x800000) ? WS2812B_T1H : WS2812B_T0H;
        }
    }
}

void WS2812B::SetPixels(uint8_t _num, uint32_t *_color)
{
    uint8_t i, j;
    for (j = 0; j < _num; j++)
    {
        for (i = 0; i < WS2812B_DATA_SIZE; i++)
        {
            buffer[i + j * WS2812B_DATA_SIZE] = ((_color[j] << i) & 0x800000) ? WS2812B_T1H : WS2812B_T0H;
        }
    }
}

void WS2812B::SetPixelAt(uint8_t _idx, uint32_t _color)
{
    if (_idx >= WS2812B_NUM)
        return;

    for (uint8_t i = 0; i < WS2812B_DATA_SIZE; i++)
    {
        buffer[i + _idx * WS2812B_DATA_SIZE] = ((_color << i) & 0x800000) ? WS2812B_T1H : WS2812B_T0H;
    }
}

void WS2812B::UpdatePixels(void)
{
    DMA_HandleTypeDef *dma = htim->hdma[TIM_DMA_ID_CC1];

    // 等上一帧发完：DMA 仍为 BUSY 时重装会被 HAL 静默丢弃整帧
    uint32_t guard = 200000UL;
    while (HAL_DMA_GetState(dma) == HAL_DMA_STATE_BUSY && guard-- > 0)
    {
    }

    // 复位通道状态（PWM+DMA 是一次性启动，发完后 ChannelState 停在 BUSY）。
    // 不可改用 HAL_TIM_PWM_Stop_DMA：其内部的 HAL_DMA_Abort_IT 会把 stream 置成
    // HAL_DMA_STATE_ABORT 且 HAL 不会自行复位，之后每帧都被静默跳过。
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);

    (void)HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_1, (uint32_t *)buffer, WS2812B_BUF_SIZE);
}
