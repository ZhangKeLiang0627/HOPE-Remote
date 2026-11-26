#ifndef TRANSMITTER_HPP
#define TRANSMITTER_HPP

#include "signal.hpp"

// 发射器抽象基类：IR/RF 各自重写 play()。
// IR 需在载波段软件翻转生成 38kHz 载波；RF 直驱基带电平。
// 两者共用段遍历/TIM2 计时框架。
class Transmitter
{
public:
    virtual ~Transmitter() = default;
    virtual void play(const Signal& sig) = 0;
};

#endif // TRANSMITTER_HPP
