#ifndef RECEIVER_HPP
#define RECEIVER_HPP

#include <cstdint>
#include "signal.hpp"

// 录制状态
enum class CaptureState { Idle, WaitingEdge, Capturing, Done, Timeout, BufferFull };

// 遥控接收状态机（红外/射频通用，非阻塞，主循环逐轮调用 poll()）。
//
// 一体化接收头(HS0038 类 / 超外差射频接收模块)把载波解调为 TTL：空闲=高、收到载波=低，输出反相。
// 边沿靠 ADC1 相邻采样 |diff|>阈值 判定，用 TIM2 计每段时长。
// 保存到数组时电平取反，存真实 IR 信号(载波=高/mark、无载波=低/space)，与发送侧一致。
class Receiver
{
public:
    // 录制常量（可调）
    static constexpr uint16_t kEdgeThreshold = 100;   // 12bit ADC 差值判边沿
    static constexpr uint32_t kMinSegUs      = 20;    // 去抖：段长短于此判为假边沿丢弃(真实段≥500μs)
    static constexpr uint32_t kIdleEndUs     = 100000; // 空闲(高电平空间段)持续此值 → 信号结束(100ms)
    static constexpr uint32_t kTimeoutMs     = 15000; // 整个录制会话总上限

    // 启动录制（应答 xxNN 后调用）。adcChannel：0=IR(PA0/ADC1_CH0)，4=RF(PA4/ADC1_CH4)。
    // 清空信号、切换 ADC 通道、置单次转换、取空闲基准、记超时起点。
    void start(Signal& sig, uint8_t adcChannel);

    // 主循环逐轮调用，驱动 WaitingEdge → Capturing → Done/Timeout/BufferFull。
    CaptureState poll();

    Signal* activeSignal() { return sig_; }

private:
    uint16_t readAdc();

    // 仅在通道变化时停 ADC 并重配通道；返回 false 表示配置失败。
    bool selectChannel(uint8_t channel);

    Signal*      sig_        = nullptr;
    uint32_t     startTick_  = 0;
    uint16_t     prevSample_ = 0;
    bool         levelHigh_  = false;
    CaptureState state_      = CaptureState::Idle;
    uint8_t      channel_    = 0;   // 当前配置的 ADC 通道（0 或 4）
};

#endif // RECEIVER_HPP
