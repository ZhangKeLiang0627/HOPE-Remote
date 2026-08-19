#ifndef RF_RECEIVER_HPP
#define RF_RECEIVER_HPP

#include <cstdint>

#include "usart.h"

// RF 串口帧接收器：接收 433 串口解码模块（灵-R1A 串口版）输出的
// "LC:xxxxxxxx" 帧。与 IR 的 Receiver（ADC 边沿状态机）职责对仗。
//
// 数据流：USART2 中断 → 环形缓冲（静态，回调无对象上下文）
//       → poll() 每调用至多处理一行：解析出 Frame 或暂存原始行
//       → take() 取 Frame / takeRawLine() 取原始行（rfmon 用）
class RfReceiver
{
public:
    struct Frame
    {
        uint32_t code32;   // hex 数值（8 位 hex = address+key 拼接）
        uint8_t  nHex;     // hex 位数（6 或 8）
    };

    // 清残留 + 启动 USART2 单字节中断接收
    void begin();

    // 消费环形缓冲至多处理一行；有行产出返回 true（可用 take/takeRawLine 消费）
    bool poll();

    // 取走一帧解析结果；无有效帧返回 false
    bool take(Frame& f);

    // 取走最近一条原始行；无则返回 false
    bool takeRawLine(char* out, uint8_t& len);

    // 清空残留字节/行/帧
    void flush();

    // ---- 中断回调入口（单实例，静态）----
    static uint8_t* rxBuf() { return &rxByte_; }
    static void feedByte();               // 取 rxByte_ 入环形缓冲
    static void resume();                 // 重新使能 USART2 接收

private:
    static constexpr uint32_t kRingSize = 128;

    static uint8_t  ring_[kRingSize];
    static volatile uint32_t ringHead_;
    static volatile uint32_t ringTail_;
    static uint8_t  rxByte_;

    char    line_[40];
    uint8_t lineLen_ = 0;
    char    pendingLine_[40];   // 最近一条完整原始行（rfmon）
    uint8_t pendingLen_ = 0;
    Frame   frame_;
    bool    frameValid_ = false;

    // 解析一行：匹配 "LC:" 前缀 + 6~8 位 hex
    static bool parseLine(const char* line, uint8_t len, Frame& f);
};

#endif // RF_RECEIVER_HPP
