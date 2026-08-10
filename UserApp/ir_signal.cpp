#include "ir_signal.hpp"

bool IrSignal::append(bool level, uint32_t us)
{
    const uint32_t levelBit = level ? 0x80000000u : 0u;

    // 超长电平拆成多段同电平段
    while (us > kMaxSegUs)
    {
        if (len_ >= kMaxSegments)
            return false;
        seg_[len_++] = levelBit | kMaxSegUs;
        us -= kMaxSegUs;
    }

    // 段长至少 1μs；正常情况下边沿间隔 >0，此处防御性兜底
    if (us == 0)
        us = 1;

    if (len_ >= kMaxSegments)
        return false;

    seg_[len_++] = levelBit | us;
    return true;
}
