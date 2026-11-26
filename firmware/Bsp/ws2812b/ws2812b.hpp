#ifndef __WS2812B_HPP
#define __WS2812B_HPP

#include "tim.h"

// WS2812B 灯珠驱动（TIM PWM + DMA 发送）。
// 移植自 HOPE-Link/Bsp/ws2812b（原 TIM5_CH3 @PA2），本项目为 TIM3_CH1 @PA6。
// TIM3 与 TIM5 同挂 APB1、定时器时钟同为 84MHz，下面的计数值可直接沿用。

#define WS2812B_ARR 105 // 对应 CubeMX 中 TIM3 的 Period（105-1），即 800kHz 载波
#define WS2812B_T0H 35  // 0 编码高电平占 1/3 周期
#define WS2812B_T1H 70  // 1 编码高电平占 2/3 周期

#define WS2812B_NUM 2        // 灯珠个数
#define WS2812B_DATA_SIZE 24 // 每颗 24bit（GRB）

// 帧尾复位低电平，50 拍 ≈ 62.5µs（经典版要求 >50µs）。
// 新版（V5）要求 >300µs，若遇错色或级联错位可改为 300。
#define WS2812B_RESET_TICKS 50

#define WS2812B_BUF_SIZE (WS2812B_DATA_SIZE * WS2812B_NUM + WS2812B_RESET_TICKS)

class WS2812B
{
private:
    TIM_HandleTypeDef *htim;
    uint32_t buffer[WS2812B_BUF_SIZE];

public:
    explicit WS2812B(TIM_HandleTypeDef *_htim);

    // 打包 RGB 分量。灯珠先收 G 再 R 后 B（高位先发），而 SetPixels 按
    // bit23→bit0 顺序发送，故排列成 G<<16 | R<<8 | B。
    static uint32_t Color(uint8_t _r, uint8_t _g, uint8_t _b)
    {
        return (static_cast<uint32_t>(_g) << 16) |
               (static_cast<uint32_t>(_r) << 8) |
               static_cast<uint32_t>(_b);
    }

    // 前 _num 颗设为同一颜色；_num 不得超过 WS2812B_NUM
    void SetPixels(uint8_t _num, uint32_t _color);
    // 前 _num 颗依次取 _color[] 中的颜色
    void SetPixels(uint8_t _num, uint32_t *_color);
    // 只改第 _idx 颗，其余保持
    void SetPixelAt(uint8_t _idx, uint32_t _color);

    // 启动一帧 DMA 发送（非阻塞）。返回后约 123µs 内 buffer 仍被 DMA 读取，不可改写。
    void UpdatePixels(void);
};

#endif // __WS2812B_HPP
