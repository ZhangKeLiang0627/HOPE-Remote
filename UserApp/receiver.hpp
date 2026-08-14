#ifndef RECEIVER_HPP
#define RECEIVER_HPP

#include <cstdint>
#include "signal.hpp"

// 录制状态
enum class CaptureState { Idle, WaitingEdge, WaitingGap, Capturing, Done, Timeout, BufferFull };

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
    static constexpr uint32_t kIdleEndUs     = 100000; // IR：空闲(高电平空间段)持续此值 → 信号结束(100ms)
    static constexpr uint32_t kTimeoutMs     = 15000; // 整个录制会话总上限

    // RF(PA4) 帧级捕获常量。远-R1 无信号时 DATA 输出随机噪声(连续同电平段最长~13ms)，
    // 真实 EV1527 帧为「码元段(400/1200μs)+帧间隔(连续空闲~10ms)」循环。故：
    //   · 用「连续空闲段≥kFrameGapUs」界定帧边界（起始触发 + 结束判定）
    //   · 用码元段长范围/段数校验真帧，噪声伪帧在校验被拒后清空重找
    static constexpr uint32_t kFrameGapUs   = 8000;  // 连续空闲段 ≥8ms = 帧间隔/帧边界（噪声低段≤13ms，靠校验过滤）
    static constexpr uint32_t kMinRfSegUs   = 250;   // 真帧码元段长下限（EV1527 最短 400μs；噪声大量短段被拒）
    static constexpr uint32_t kMaxRfSegUs   = 5000;  // 真帧码元段长上限（EV1527 sync 1.6ms；噪声超长段被拒）
    static constexpr uint32_t kMinRfSegs    = 10;    // 一帧至少段数（EV1527 = sync+20bit ≈ 41 段）
    static constexpr uint32_t kMaxRfSegs    = 80;    // 一帧最多段数（异常噪声长流被拒）
    static constexpr uint32_t kMinRfLongSeg = 800;   // 帧内至少一段≥800μs（sync 1.6ms / bit1 mark 1.2ms）

    // 启动录制（应答 xxNN 后调用）。adcChannel：0=IR(PA0/ADC1_CH0)，4=RF(PA4/ADC1_CH4)。
    // 清空信号、切换 ADC 通道、置单次转换、取空闲基准、记超时起点。
    void start(Signal& sig, uint8_t adcChannel);

    // 主循环逐轮调用，驱动 WaitingEdge → Capturing → Done/Timeout/BufferFull。
    CaptureState poll();

    Signal* activeSignal() { return sig_; }

    // 运行时切换 ADC 通道（0=IR/PA0，4=RF/PA4），仅在通道变化时停 ADC 重配；
    // 返回 false 表示配置失败。这是运行时通道配置，不属于 CubeMX 引脚初始化。
    bool selectChannel(uint8_t channel);

    // 当前 ADC 通道（0 或 4）。
    uint8_t channel() const { return channel_; }

    // 读一次 ADC。与 xxNN 录制共用同一路径：IR(PA0) 走单次 SWSTART，
    // RF(PA4) 走连续模式；两者均带 overrun 兜底（连续模式每次读前清 OVR/EOC，
    // 防止长时间未读触发 overrun 后数据寄存器冻结，读出的永远是旧值）。
    // 诊断命令(adcmon/dbg/raw)必须走这里，不能自己手写 CONT+Poll 直读。
    uint16_t readAdc();

private:
    CaptureState pollIr(uint16_t sample, uint16_t diff);  // IR：首边沿即录，空闲100ms结束
    CaptureState pollRf(uint16_t sample, uint16_t diff);  // RF：帧间隔界定+段长校验，抗空闲噪声
    bool validateFrame() const;                            // RF：段数/段长范围校验，真帧才保存

    Signal*      sig_        = nullptr;
    uint32_t     startTick_  = 0;
    uint16_t     prevSample_ = 0;
    bool         levelHigh_  = false;
    CaptureState state_      = CaptureState::Idle;
    uint8_t      channel_    = 0;   // 当前配置的 ADC 通道（0 或 4）
    bool         contMode_   = false; // 采样模式：RF(PA4) 走连续模式（raw 已验证稳定）
    bool         idleLevel_  = true; // 空闲线电平：HS0038(红外)=高，远-R1(射频)=低。
                                    // 空闲判定与存储极性都以它为准，做到模块输出极性无关。
};

#endif // RECEIVER_HPP
