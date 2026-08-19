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
// 实测结论（灵-R1A 串口版，2026-08-19 板端验证）：
//   · 模块只认「长载波 sync」：sync = mark 31×pulse + space 1×pulse
//     （标准 RCSwitch 的 1p+31p sync 模块收不到帧，pulse 320~900μs 均无效）
//   · 模块解码存在固定偏移：got = 0x800000 | (emit >> 1)
//     —— 回放时必须发射 位移补偿码 emit = code24 << 1，模块/目标设备才能解出 code24
//   · bit 编码：bit0 = mark 1×pulse + space 3×pulse；bit1 = mark 3×pulse + space 1×pulse
//   · pulse 320μs，MSB 先发

// 内部实现：inv=false 按「载波=mark(+)」解；inv=true 按反相（载波=space）解。
// 接收模块输出极性任意，公共 ev1527Decode 先试正向、失败再反相，做到极性无关。
static bool ev1527DecodePol(const Signal& sig, bool inv, uint32_t& code24, uint16_t& pulseUs)
{
    const uint32_t n = sig.length();
    if (n < 6)
        return false;

    // 1) sync = 最长载波段（≈31×pulse 长载波，远大于 bit1 的 3×pulse）
    uint32_t syncIdx = 0, syncUs = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        const uint32_t seg = sig.at(i);
        const bool     carrier = inv ? !sig.level(seg) : sig.level(seg);
        if (carrier)
        {
            const uint32_t us = sig.us(seg);
            if (us > syncUs) { syncUs = us; syncIdx = i; }
        }
    }
    if (syncUs < 5000 || syncUs > 50000)
        return false;                    // 真 sync≈10ms；噪声无此量级长载波

    // 2) 脉宽 ≈ sync/31
    const uint32_t pulse = syncUs / 31;
    if (pulse < 150 || pulse > 1500)
        return false;

    // 3) 长载波 + sync space 之后连续 24 个 (mark,space) bit 对，按 mark/space 长短判 0/1
    uint32_t code = 0;
    for (uint32_t b = 0; b < 24; ++b)
    {
        const uint32_t mi = syncIdx + 2 + b * 2;   // 跳过 sync space(1×pulse)
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
    out.append(true, 31 * p);            // sync mark：长载波（模块实测唯一可收的 sync）
    out.append(false, 1 * p);            // sync space
    for (int b = 23; b >= 0; --b)        // MSB 先发
    {
        const bool one = (code24 >> b) & 1;
        out.append(true, one ? 3 * p : 1 * p);
        out.append(false, one ? 1 * p : 3 * p);
    }
    // 帧尾 space 不入帧：RfTransmitter::play 播完自动拉低
}
