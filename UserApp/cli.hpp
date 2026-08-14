#ifndef CLI_HPP
#define CLI_HPP

#include <cstdint>

#include "signal.hpp"
#include "storage.hpp"
#include "receiver.hpp"
#include "transmitter.hpp"
#include "ir_transmitter.hpp"
#include "rf_transmitter.hpp"

// 串口命令入口。
//
// 命令集（小写，\r 或 \n 结尾）：
//   xxNN   学习录制到槽 NN（00~95），空闲 100ms 自动保存，15s 超时
//   fsNN   回放槽 NN
//   slots  列出槽占用状态
//   help   列出用法
//
// 录制/回放是阻塞操作，期间 USART 中断照常收字节，命令排队处理。
class Cli
{
public:
    Cli(Storage& st, Signal& sig, Receiver& rx, IrTransmitter& irTx, RfTransmitter& rfTx);

    // 清环形缓冲并启动 HAL_UART_Receive_IT 单字节接收。
    void init();

    // 主循环调用：从环形缓冲攒行 → 解析 → 分发。
    void poll();

    // 带 \r\n 的应答输出（复用 Usart_debugMsg）。
    void reply(const char* fmt, ...);

private:
    void dispatch();
    void onLearn(uint16_t slot);
    void onSend(uint16_t slot);
    void onHelp();
    void onSlots();
    void onDump(uint16_t slot);  // 诊断：打印槽内全部段(带符号时长μs)
    void onClr(uint16_t slot);   // 清除槽内数据（擦整扇区并重写兄弟槽）
    void onDbg(uint8_t channel); // 诊断：1s 内 ADC 采样 min/max/avg/边沿数（0xFF=当前通道）
    void onRaw(uint8_t channel); // 诊断：固定 3s 窗口抓边沿(绕过空闲判定)并打印（0xFF=当前通道）
    void onAdcMon(uint8_t channel); // 诊断：ADC 长监听持续打印，'x' 退出（0xFF=当前通道）
    void onAdcTest(uint8_t channel); // 诊断：原始 ADC 直读逐点打印，'x' 退出（0xFF=当前通道）
    void onRfTest(); // 自测：PA2 驱动 T2L 发合成帧 + 同时采样 PA4 重建接收帧（空气回环）
    void onRfCw();   // 自测：PA2 拉高 1.5s 连续载波，采样 PA4 统计（判 R1 是否收到/饱和）
    void onRfKey();  // 自测：PA2 脉冲 30ms 触发 T2L(按键触发型)，全窗口抓 PA4 边沿验证是否爆发
    void onRfAb();   // 自测：PA2 高 500ms + 低 500ms 各采样统计，判 T2L 是否跟随 PA2 开关
    void onRfRec();  // 自测：载波开 200ms 后关，测 R1 从低电平恢复到噪声的时间(近距饱和恢复时间)
    void onRfSlow(); // 自测：慢帧回环(码元 2000μs)——信号弱时 R1 只能跟长段，慢帧可证完整帧跟踪
    void printRaw(const Signal& sig, uint32_t len); // 带符号逗号分隔打印，末尾 len=段数

    Storage&       storage_;
    Signal&        signal_;
    Receiver&      receiver_;
    IrTransmitter& irTransmitter_;
    RfTransmitter& rfTransmitter_;

    uint8_t line_[64];
    uint8_t lineLen_ = 0;
};

#endif // CLI_HPP
