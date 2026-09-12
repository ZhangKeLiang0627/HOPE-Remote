#ifndef __WS2812B_HPP
#define __WS2812B_HPP

#include "tim.h"

// 移植自 HOPE-Link/2.Firmware/Bsp/ws2812b（TIM5_CH3 @PA2）。
// 本项目改用 TIM3_CH1 @PA6，时序参数不变：TIM3 与 TIM5 同挂 APB1，
// 定时器时钟同为 84MHz，CubeMX 侧 Period = WS2812B_ARR-1 = 105-1，
// Prescaler = 1-1，即 84MHz / 105 = 800kHz 的 PWM 载波。

#define WS2812B_ARR 105 // TIM的自动重装值 / 使得PWM输出频率在800kHz
#define WS2812B_T0H 35  // 0编码高电平时间占1/3
#define WS2812B_T1H 70  // 1编码高电平时间占2/3

#define WS2812B_NUM 2   // 使用灯珠的个数
#define WS2812B_DATA_SIZE 24 // WS2812B传输一个数据的大小是3个字节（24bit）

// 时序：84MHz 计数 —— T0H 35 拍 ≈ 417ns（规格 400ns ±150ns）
//                    T1H 70 拍 ≈ 833ns（规格 800ns ±150ns）
// 一帧 = NUM×24 个颜色位，尾部再补 50 拍全 0（≈62µs 低电平）作复位间隔。
// 2 颗灯珠时一帧 = 98 拍 × 1.25µs ≈ 123µs。
// 传输期间不能改写 buffer，且 SetPixels 的 _num 不得超过 WS2812B_NUM。

class WS2812B
{
private:
    TIM_HandleTypeDef *htim;
    uint32_t buffer[WS2812B_DATA_SIZE * WS2812B_NUM + 50];

public:
    explicit WS2812B(TIM_HandleTypeDef *_htim);

    // 把 RGB 分量打包成 SetPixels 需要的 24bit 值。
    // 灯珠从数据线上先收 G7..G0，再 R7..R0，最后 B7..B0（高位先发），
    // 而 SetPixels 是按 bit23→bit0 的顺序发送，故排列成 G<<16 | R<<8 | B。
    static uint32_t Color(uint8_t _r, uint8_t _g, uint8_t _b)
    {
        return (static_cast<uint32_t>(_g) << 16) |
               (static_cast<uint32_t>(_r) << 8) |
               static_cast<uint32_t>(_b);
    }

    void SetPixels(uint8_t _num, uint32_t _color);
    void SetPixels(uint8_t _num, uint32_t *_color);

    // 只改单颗灯珠的缓存（不动其余灯珠），用于分别点亮/定位。
    void SetPixelAt(uint8_t _idx, uint32_t _color);

    void UpdatePixels(void);

};

#endif // __WS2812B_HPP
