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
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
                          sig.level(seg) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        while (__HAL_TIM_GET_COUNTER(&htim2) < target)
        {
        }
    }

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

// ---- EV1527 编解码 ----
// 实测结论（2026-08-19 板端验证，最终版）：
//   · 正确时序 = RCSwitch protocol 1 标准（与参考项目 ESP433RF 的 RCSwitch::send 一致）：
//        [24bit 数据 MSB first][sync: mark 1×pulse + space 31×pulse]
//     数据先行、sync 收尾、短载波 + 长空闲、bit0=1p+3p / bit1=3p+1p、pulse 320μs
//   · ⚠️ 串口模块（灵-R1A）只认「长载波 sync」（31p+1p），对标准 1p+31p 收不到帧
//     —— 模块回环（rfloop）结果不可用于验证回放正确性，以目标设备实测为准
//   · 回放用原码（code24），无需位移补偿

// 内部实现：inv=false 按「载波=mark(+)」解；inv=true 按反相（载波=space）解。
// 接收模块输出极性任意，公共 ev1527Decode 先试正向、失败再反相，做到极性无关。
static bool ev1527DecodePol(const Signal& sig, bool inv, uint32_t& code24, uint16_t& pulseUs)
{
    const uint32_t n = sig.length();
    if (n < 8)
        return false;

    // 1) sync = 最长无载波段（帧尾 31×pulse 长空闲）
    uint32_t syncIdx = 0, syncUs = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        const uint32_t seg = sig.at(i);
        const bool     carrier = inv ? !sig.level(seg) : sig.level(seg);
        if (!carrier)
        {
            const uint32_t us = sig.us(seg);
            if (us > syncUs) { syncUs = us; syncIdx = i; }
        }
    }
    if (syncUs < 5000 || syncUs > 50000)
        return false;                    // 真 sync≈10ms；噪声无此量级长空闲

    // 2) 脉宽 ≈ sync/31
    const uint32_t pulse = syncUs / 31;
    if (pulse < 150 || pulse > 1500)
        return false;

    // 3) 数据位在 sync 之前：帧 = [b23..b0][sync mark][sync space]，
    //    从帧头起连续 24 个 (mark,space) bit 对，MSB first
    uint32_t code = 0;
    for (uint32_t b = 0; b < 24; ++b)
    {
        const uint32_t mi = b * 2;       // MSB first：index 0 是 bit23
        if (mi + 1 >= n)
            return false;
        const bool ml = inv ? !sig.level(sig.at(mi))     : sig.level(sig.at(mi));
        const bool sl = inv ? !sig.level(sig.at(mi + 1)) : sig.level(sig.at(mi + 1));
        if (!ml || sl)
            return false;                // 必须 mark/space 交替
        const uint32_t m = sig.us(sig.at(mi));
        const uint32_t s = sig.us(sig.at(mi + 1));
        const uint32_t total = m + s;
        if (total < pulse * 3 || total > pulse * 5)
            return false;                // 每 bit ≈4×pulse(±1)
        code <<= 1;
        if (m > s)
            code |= 1;                   // mark 长 = '1'（3:1）
    }
    code24  = code;
    pulseUs = static_cast<uint16_t>(pulse);
    return true;
}

bool ev1527Decode(const Signal& sig, uint32_t& code24, uint16_t& pulseUs)
{
    // 先按当前极性（载波=mark）解；失败再反相试一次 → 接收模块输出极性无关。
    if (ev1527DecodePol(sig, false, code24, pulseUs))
        return true;
    return ev1527DecodePol(sig, true, code24, pulseUs);
}

void ev1527Encode(Signal& out, uint32_t code24, uint16_t pulseUs)
{
    const uint32_t p = pulseUs;
    out.clear();
    for (int b = 23; b >= 0; --b)        // MSB first，数据先行（RCSwitch 风格）
    {
        const bool one = (code24 >> b) & 1;
        out.append(true, one ? 3 * p : 1 * p);
        out.append(false, one ? 1 * p : 3 * p);
    }
    out.append(true, 1 * p);             // sync mark：短载波（帧尾）
    out.append(false, 31 * p);           // sync space：长空闲（≈10ms @320μs）
}

void encodeRfVariant(Signal& out, uint32_t code24, uint16_t pulseUs, uint8_t variant)
{
    const uint32_t p = pulseUs;
    out.clear();
    for (int b = 23; b >= 0; --b)        // MSB first，数据先行
    {
        const bool one = (code24 >> b) & 1;
        out.append(true, one ? 3 * p : 1 * p);
        out.append(false, one ? 1 * p : 3 * p);
    }
    if (variant == 2)
    {
        out.append(true, 31 * p);        // 长载波 sync（模块偏好，目标设备不认）
        out.append(false, 1 * p);
    }
    else
    {
        out.append(true, 1 * p);         // RCSwitch 标准 sync
        out.append(false, 31 * p);
    }
}
