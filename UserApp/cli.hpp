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
    void onDbg();                // 诊断：1s 内 ADC 采样 min/max/avg/边沿数
    void onRaw();                // 诊断：固定 200ms 窗口抓边沿(绕过空闲判定)并打印
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
