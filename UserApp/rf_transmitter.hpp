#ifndef RF_TRANSMITTER_HPP
#define RF_TRANSMITTER_HPP

#include "transmitter.hpp"

// 射频回放：逐段直驱 PA2（mark=高 / space=低），用 TIM2 计段长。
// 433MHz 发射模块内部生成载波，DATA 直接给基带电平即可（无需软件载波翻转）。
class RfTransmitter : public Transmitter
{
public:
    void play(const Signal& sig) override;
};

#endif // RF_TRANSMITTER_HPP
