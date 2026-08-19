#ifndef IR_RECEIVER_HPP
#define IR_RECEIVER_HPP

#include <cstdint>
#include "signal.hpp"

// 录制状态
enum class CaptureState { Idle, WaitingEdge, WaitingGap, Capturing, Done, Timeout, BufferFull };

// 红外遥控接收状态机（非阻塞，主循环逐轮调用 poll()）。
//
// 一体化接收头(HS0038 类)把 38kHz 载波解调为 TTL：空闲=高、收到载波=低，输出反相。
// 边沿靠 ADC1(PA0/CH0) 相邻采样 |diff|>阈值 判定，用 TIM2 计每段时长。
// 保存到数组时电平取反，存真实 IR 信号(载波=高/mark、无载波=低/space)，与发送侧一致。
class IrReceiver
{
public:
    // 录制常量（可调）
    static constexpr uint16_t kEdgeThreshold = 100;   // 12bit ADC 差值判边沿
    static constexpr uint32_t kMinSegUs      = 20;    // 去抖：段长短于此判为假边沿丢弃(真实段≥500μs)
    static constexpr uint32_t kIdleEndUs     = 100000; // IR：空闲(高电平空间段)持续此值 → 信号结束(100ms)
    static constexpr uint32_t kTimeoutMs     = 15000; // 整个录制会话总上限

    // 启动录制（应答 xxNN 后调用）。IR=PA0/ADC1_CH0。
    // 清空信号、置单次转换、取空闲基准、记超时起点。
    void start(Signal& sig);

    // 主循环逐轮调用，驱动 WaitingEdge → Capturing → Done/Timeout/BufferFull。
    CaptureState poll();

    Signal* activeSignal() { return sig_; }

    // 读一次 ADC：单次 SWSTART 触发 + EOC 轮询，带 overrun 兜底
    //（长时间未读触发 overrun 后数据寄存器会冻结，每次读前清 OVR/EOC）。
    // 诊断命令(dbg/raw)必须走这里，不能自己手写 CONT+Poll 直读。
    uint16_t readAdc();

private:
    CaptureState pollIr(uint16_t sample, uint16_t diff);  // IR：首边沿即录，空闲100ms结束

    Signal*      sig_        = nullptr;
    uint32_t     startTick_  = 0;
    uint16_t     prevSample_ = 0;
    bool         levelHigh_  = false;
    CaptureState state_      = CaptureState::Idle;
    bool         idleLevel_  = true; // 空闲线电平：HS0038(红外)=高。空闲判定与存储极性以它为准。
};

#endif // IR_RECEIVER_HPP
