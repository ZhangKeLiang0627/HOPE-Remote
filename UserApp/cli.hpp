#ifndef CLI_HPP
#define CLI_HPP

#include <cstdint>

#include "signal.hpp"
#include "slot_store.hpp"
#include "ir_receiver.hpp"
#include "rf_receiver.hpp"
#include "transmitter.hpp"
#include "ir_transmitter.hpp"
#include "rf_transmitter.hpp"
#include "ws2812b.hpp"
#include "user_led.hpp"

// 串口命令入口（USART1 @115200）。
//
// 命令集（小写，\r 或 \n 结尾）：
//   xxNNN   学习遥控码到槽（000-095 IR 波形，100-611 RF 码值）
//   fsNNN   回放槽（RF 支持发射参数：fsNNN [帧数 簇数 帧间隔 簇间隔]）
//   slots   列出槽占用状态
//   duNNN   打印槽内容（IR 段数组 / RF 码值）
//   clNNN   清除槽数据
//   scNNN   RF 槽直接编程 EV1527 码：scNNN<hex6|hex8>
//   ledRRGGBB 设置两颗 RGB 灯珠颜色（每分量 2 位 hex，两颗同色）
//   lednRRGGBB 只设置第 n 颗（n=0/1），用于单独定位某一颗灯珠
//   uled0/uled1 手动熄灭/点亮 PC13 用户指示灯（硬件自检）
//   dbg     采样 ADC 1s（IR/PA0）统计
//   raw     捕获 3000ms 原始 IR 波形并打印
//   rfmon   监听 USART2 码流（RF 模块串口），x 退出
//   rfloop  回环自测：PA5 发射已知码 → USART2 回收 → 比对
//   evtest  EV1527 编解码自测
//   help    列出用法
//
// 录制/回放是阻塞操作，期间 USART1/2 中断照常收字节。
class Cli
{
public:
    Cli(IrStore& ir, RfStore& rf, RfReceiver& rfRx, Signal& sig, IrReceiver& rx,
        IrTransmitter& irTx, RfTransmitter& rfTx, WS2812B& led, UserLed& userLed);

    // 清环形缓冲并启动 USART1/USART2 单字节中断接收。
    void init();

    // 主循环调用：从环形缓冲攒行 → 解析 → 分发。
    void poll();

    // 带 \r\n 的应答输出（复用 Usart_debugMsg）。
    void reply(const char* fmt, ...);

private:
    void dispatch();
    void onLearn(uint16_t slot);
    void onLearnRf(uint16_t slot);      // RF 学习：RfReceiver 取帧，连续 2 帧去抖
    void onSend(uint16_t slot, uint8_t variant, uint8_t frames, uint8_t bursts,
                uint16_t frameGapMs, uint16_t burstGapMs);
    void onHelp();
    void onSlots();
    void onDump(uint16_t slot);
    void onClr(uint16_t slot);
    void onSetCode(uint16_t slot, uint32_t full, bool eight);
    void onEvTest();
    void onLed(uint8_t r, uint8_t g, uint8_t b);   // RGB 灯珠（PA6/TIM3_CH1）两颗同色
    void onLedAt(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);   // 只点第 idx 颗
    void onUserLed(bool lit);            // PC13 用户指示灯手动控制（硬件自检）
    void onDbg();                        // 诊断：1s ADC 采样统计（IR/PA0）
    void onRaw();                        // 诊断：3s 窗口抓 IR 原始边沿并打印
    void onRfMon();                      // 诊断：监听 USART2 码流，'x' 退出
    void onRfLoop();                     // 自测：PA5 发已知码 → RfReceiver 回收比对
    void onRfScan(uint16_t slot);        // 自测：15 种编码变体发射，目标设备实测定位
    void onRfScan2(uint16_t slot);       // 自测：sync-前置比例精扫 21 种，模块回收判 MATCH
    void printRaw(const Signal& sig, uint32_t len);

    IrStore&       irStore_;
    RfStore&       rfStore_;
    RfReceiver&    rfReceiver_;          // RF 串口帧接收器（USART2）
    Signal&        signal_;
    IrReceiver&    irReceiver_;
    IrTransmitter& irTransmitter_;
    RfTransmitter& rfTransmitter_;
    WS2812B&       led_;
    UserLed&       userLed_;             // PC13 用户指示灯（学习中常亮）

    uint8_t line_[64];
    uint8_t lineLen_ = 0;
};

#endif // CLI_HPP
