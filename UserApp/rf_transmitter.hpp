#ifndef RF_TRANSMITTER_HPP
#define RF_TRANSMITTER_HPP

#include "transmitter.hpp"
#include "signal.hpp"

// 射频回放：逐段直驱 PA5（mark=高 / space=低），用 TIM2 计段长。
// 433MHz 发射模块内部生成载波，DATA 直接给基带电平即可（无需软件载波翻转）。
class RfTransmitter : public Transmitter
{
public:
    void play(const Signal& sig) override;
};

// ---- EV1527 编解码（RCSwitch protocol 1 时序，参考 433_test_arduino 套件）----
// 帧结构：sync(31×pulse mark + 1×pulse space) + 24 bit(MSB 先发，每 bit 4×pulse)
//   bit0 = mark 1×pulse + space 3×pulse；bit1 = mark 3×pulse + space 1×pulse
// 存储极性约定：载波=mark(+)、无载波=space(-)（与 Signal 一致）。

// 从捕获段数组解码 24 位码。自动定位 sync(最长 mark，≈31×pulse)并估算脉宽，
// sync 之后连续 24 个 (mark,space) bit 对按 mark/space 长短判 0/1。
// 成功返回 true 并回填 code24（MSB 先发）与 pulseUs。
bool ev1527Decode(const Signal& sig, uint32_t& code24, uint16_t& pulseUs);

// 由 24 位码 + 脉宽生成标准 EV1527 帧（50 段：sync + 24bit）。用于重放前规范化存储。
void ev1527Encode(Signal& out, uint32_t code24, uint16_t pulseUs);

// RF 回放编码变体（实测结论，见 docs/rf-uart2-debug.md）：
//   variant 0 = RCSwitch 标准（sync 收尾 1p+31p + MSB，目标设备实测通杀，默认）
//   variant 2 = 长载波 sync 收尾（31p+1p + MSB，调试用）
void encodeRfVariant(Signal& out, uint32_t code24, uint16_t pulseUs, uint8_t variant);

#endif // RF_TRANSMITTER_HPP
