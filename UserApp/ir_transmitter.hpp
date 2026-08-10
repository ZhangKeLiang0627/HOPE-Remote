#ifndef IR_TRANSMITTER_HPP
#define IR_TRANSMITTER_HPP

#include "ir_signal.hpp"

// 红外回放：逐段驱动 PA1，用 TIM2 计段长，忙循环。
// 载波段(mark)软件翻转 PA1 生成 38kHz 载波(HS0038 类接收头必需)；
// 无载波段(space)拉低。段间同电平段自然拼接还原原波形。
class IrTransmitter
{
public:
    // 阻塞回放整条信号，播完 PA1 拉低。
    void play(const IrSignal& sig);
};

#endif // IR_TRANSMITTER_HPP
