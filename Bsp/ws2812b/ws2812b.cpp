#include "ws2812b.hpp"

WS2812B::WS2812B(TIM_HandleTypeDef *_htim) : htim(_htim)
{
    // 初始化缓冲区为 0
    uint32_t bufSize = WS2812B_DATA_SIZE * WS2812B_NUM + 50;
    for (uint32_t i = 0; i < bufSize; i++)
    {
        buffer[i] = 0;
    }
}

void WS2812B::SetPixels(uint8_t _num, uint32_t _color)
{
    uint8_t i, j;
    for (j = 0; j < _num; j++)
    {
        for (i = 0; i < WS2812B_DATA_SIZE; i++)
        {
            // 因为数据发送的顺序是GRB，高位先发，所以从高位开始判断，判断后比较值先放入缓存数组 
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
            // 因为数据发送的顺序是GRB，高位先发，所以从高位开始判断，判断后比较值先放入缓存数组 
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
        // 同 SetPixels：GRB、高位先发
        buffer[i + _idx * WS2812B_DATA_SIZE] = ((_color << i) & 0x800000) ? WS2812B_T1H : WS2812B_T0H;
    }
}

void WS2812B::UpdatePixels(void)
{
    DMA_HandleTypeDef *dma = htim->hdma[TIM_DMA_ID_CC1];

    // 1) 等上一帧真正发完（正常一帧 ~123µs）。
    //    若 DMA 还在 BUSY 就重装，HAL_DMA_Start_IT 会因状态非 READY 直接返回，
    //    整帧被静默丢弃。guard 仅作防御，避免异常时死等。
    uint32_t guard = 200000UL;
    while (HAL_DMA_GetState(dma) == HAL_DMA_STATE_BUSY && guard-- > 0)
    {
    }

    // 2) 复位定时器通道状态。
    //    HAL 的 PWM+DMA 是一次性启动，发完后 htim->ChannelState[] 停在 BUSY，
    //    不复位的话下一次 HAL_TIM_PWM_Start_DMA 直接返回 HAL_BUSY（灯不更新）。
    //
    //    这里必须用 HAL_TIM_PWM_Stop 而不是 HAL_TIM_PWM_Stop_DMA：
    //    Stop_DMA 内部走 HAL_DMA_Abort_IT，而本版 HAL 的 Abort_IT 会把 stream
    //    置成 HAL_DMA_STATE_ABORT 且永不复位；之后每次 HAL_DMA_Start_IT 都会
    //    静默跳过 → 灯永久不再更新（只能断电复位）。
    //    HAL_TIM_PWM_Stop 只动定时器、不碰 DMA，正好只做"通道状态置回 READY"。
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);

    // 3) 发送数据到 WS2812B
    (void)HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_1, (uint32_t *)buffer,
                                WS2812B_NUM * WS2812B_DATA_SIZE + 50);
}
