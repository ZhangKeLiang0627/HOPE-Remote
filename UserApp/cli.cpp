#include "cli.hpp"

#include <cstdio>
#include <cstring>
#include <cstdarg>

#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "interface_uart.h"
#include "stm32f4xx_hal.h"

// ---------------- USART1 接收环形缓冲 ----------------
// 生产者：USART1 RX 中断；消费者：主循环 poll()。单生产者单消费者。
namespace
{
    constexpr uint32_t kRingSize = 256;
    uint8_t  ring_[kRingSize];
    volatile uint32_t ringHead_ = 0;   // 写指针（ISR）
    volatile uint32_t ringTail_ = 0;   // 读指针（主循环）
    uint8_t  rxByte_ = 0;

    bool ringEmpty() { return ringHead_ == ringTail_; }

    uint8_t ringGet()
    {
        if (ringEmpty())
            return 0;
        const uint8_t c = ring_[ringTail_];
        ringTail_ = (ringTail_ + 1) % kRingSize;
        return c;
    }

    // 2~3 位十进制 → 槽号；非法或超 131 返回 0xFFFF
    uint16_t parseSlot(const char* digits, uint8_t n)
    {
        if (n < 2 || n > 3)
            return 0xFFFF;
        uint16_t v = 0;
        for (uint8_t i = 0; i < n; ++i)
        {
            if (digits[i] < '0' || digits[i] > '9')
                return 0xFFFF;
            v = static_cast<uint16_t>(v * 10 + static_cast<uint16_t>(digits[i] - '0'));
        }
        return (v <= Storage::kNumSlots) ? v : 0xFFFF;
    }
}

// 覆盖 HAL 弱回调：收到一字节 → 入环形缓冲 → 重新使能接收。
// 必须 extern "C" 以匹配 HAL 的 C 链接声明。
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    (void)huart;
    const uint32_t next = (ringHead_ + 1) % kRingSize;
    if (next != ringTail_)            // 缓冲满则丢弃
    {
        ring_[ringHead_] = rxByte_;
        ringHead_ = next;
    }
    HAL_UART_Receive_IT(&huart1, &rxByte_, 1);
}

Cli::Cli(Storage& st, Signal& sig, Receiver& rx, IrTransmitter& tx)
    : storage_(st), signal_(sig), receiver_(rx), transmitter_(tx)
{
}

void Cli::init()
{
    ringHead_ = 0;
    ringTail_ = 0;
    lineLen_  = 0;
    HAL_UART_Receive_IT(&huart1, &rxByte_, 1);
}

void Cli::poll()
{
    while (!ringEmpty())
    {
        const uint8_t c = ringGet();

        if (c == '\r' || c == '\n')
        {
            if (lineLen_ > 0)
            {
                line_[lineLen_] = '\0';
                dispatch();
                lineLen_ = 0;
            }
        }
        else if (lineLen_ < sizeof(line_) - 1)
        {
            line_[lineLen_++] = c;
        }
        // 行过长时丢弃多余字符
    }
}

void Cli::dispatch()
{
    // 小写归一
    for (uint32_t i = 0; i < lineLen_; ++i)
        if (line_[i] >= 'A' && line_[i] <= 'Z')
            line_[i] = static_cast<uint8_t>(line_[i] + ('a' - 'A'));

    const char* cmd = reinterpret_cast<const char*>(line_);

    if (strcmp(cmd, "help") == 0)  { onHelp();  return; }
    if (strcmp(cmd, "slots") == 0) { onSlots(); return; }

    if ((lineLen_ == 4 || lineLen_ == 5) && cmd[0] == 'x' && cmd[1] == 'x')
    {
        const uint16_t slot = parseSlot(cmd + 2, static_cast<uint8_t>(lineLen_ - 2));
        if (Storage::isValidSlot(slot))
            onLearn(slot);
        else
            reply("ERR");
        return;
    }

    if ((lineLen_ == 4 || lineLen_ == 5) && cmd[0] == 'f' && cmd[1] == 's')
    {
        const uint16_t slot = parseSlot(cmd + 2, static_cast<uint8_t>(lineLen_ - 2));
        if (Storage::isValidSlot(slot))
            onSend(slot);
        else
            reply("ERR");
        return;
    }

    if ((lineLen_ == 4 || lineLen_ == 5) && cmd[0] == 'd' && cmd[1] == 'u')
    {
        const uint16_t slot = parseSlot(cmd + 2, static_cast<uint8_t>(lineLen_ - 2));
        if (Storage::isValidSlot(slot))
            onDump(slot);
        else
            reply("ERR");
        return;
    }

    if (strcmp(cmd, "dbg") == 0) { onDbg(); return; }
    if (strcmp(cmd, "raw") == 0) { onRaw(); return; }

    reply("ERR");
}

void Cli::onLearn(uint16_t slot)
{
    reply("REC %u start", static_cast<unsigned>(slot));
    const uint32_t t0 = HAL_GetTick();
    receiver_.start(signal_, (slot < 96) ? 0 : 4);   // IR=ADC1_CH0(PA0), RF=ADC1_CH4(PA4)

    CaptureState st;
    do
    {
        st = receiver_.poll();
    } while (st == CaptureState::WaitingEdge || st == CaptureState::Capturing);
    const uint32_t elapsedMs = HAL_GetTick() - t0;

    if (st == CaptureState::Timeout)
    {
        reply("REC %u TIMEOUT (%ums)", static_cast<unsigned>(slot), static_cast<unsigned>(elapsedMs));
        return;
    }

    // 空信号（如首个边沿后只有噪声/空闲）不写 Flash
    if (signal_.length() == 0)
    {
        reply("REC %u EMPTY", static_cast<unsigned>(slot));
        return;
    }

    // 信号总时长 sumUs 与录制墙钟 elapsedMs 对比（注意空闲阈值 100ms 计入等待）：
    //   sumUs≈(elapsedMs-100)*1000 → 干净单帧：信号播完再等 100ms 空闲判定结束
    //   sumUs 明显偏小           → 录制在信号结束后仍在等待，说明边沿检测中途失效（ADC 状态问题）
    uint32_t sumUs = 0;
    for (uint32_t i = 0; i < signal_.length(); ++i)
        sumUs += signal_.us(signal_.at(i));

    // Done / BufferFull 均保存已录部分
    if (!storage_.save(slot, signal_.data(), static_cast<uint16_t>(signal_.length())))
    {
        reply("REC %u FLASHERR", static_cast<unsigned>(slot));
        return;
    }
    reply("REC %u OK (%u seg, %u us, %ums)", static_cast<unsigned>(slot),
          static_cast<unsigned>(signal_.length()), static_cast<unsigned>(sumUs),
          static_cast<unsigned>(elapsedMs));
}

void Cli::onSend(uint16_t slot)
{
    uint16_t len = 0;
    if (!storage_.load(slot, signal_.data(), len))
    {
        reply("FS %u EMPTY", static_cast<unsigned>(slot));
        return;
    }
    signal_.setLength(len);
    if (slot < 96)
    {
        transmitter_.play(signal_);
        reply("FS %u OK", static_cast<unsigned>(slot));
    }
    else
    {
        reply("FS %u NOTIMPL", static_cast<unsigned>(slot));   // Task 3 接入 RfTransmitter
    }
}

void Cli::onHelp()
{
    reply("LUMOS-Remote commands:");
    reply("  xxNNN  learn remote into slot (000-095 IR, 100-131 RF)");
    reply("  fsNNN  play slot (000-095 IR, 100-131 RF)");
    reply("  slots  list slot occupancy");
    reply("  duNNN  dump slot segment durations (debug)");
    reply("  dbg    sample ADC 1s: min/max/avg/edges (debug)");
    reply("  raw    capture 200ms raw signal & dump (debug)");
    reply("  help   show this");
}

void Cli::onSlots()
{
    uint16_t used = 0;
    uint16_t total = 0;
    for (uint16_t s = 0; s < Storage::kNumSlots; ++s)
    {
        if (!Storage::isValidSlot(s))
            continue;
        ++total;
        const uint16_t n = storage_.segCountOf(s);
        if (n > 0)
        {
            reply("  %03u: OK (%u seg)", static_cast<unsigned>(s), static_cast<unsigned>(n));
            ++used;
        }
    }
    reply("total: %u/%u used", static_cast<unsigned>(used), static_cast<unsigned>(total));
}

// 诊断：把槽内录制的每个段按带符号时长μs 打印，供外部 IR 分析工具直接解析。
//   - 正数 = 载波段(mark)，负数 = 无载波空间段(space)，逗号分隔，末尾 len=总段数
//   - 时长是规范值(如 9000/4500/562/1687) → 真实码被提前截断(空闲阈值/同步问题)
//   - 时长是杂乱的微秒级随机数     → 信号路径噪声(浮空/弱信号/接错)
void Cli::onDump(uint16_t slot)
{
    uint16_t len = 0;
    if (!storage_.load(slot, signal_.data(), len))
    {
        reply("DUMP %u EMPTY", static_cast<unsigned>(slot));
        return;
    }
    signal_.setLength(len);
    reply("DUMP %u %u seg:", static_cast<unsigned>(slot), static_cast<unsigned>(len));
    printRaw(signal_, len);
}

// 带符号逗号分隔打印：正=载波段(mark)、负=无载波空间段(space)（时长μs）。
// 全部段单行连续输出（分块裸发不插换行，避免 reply() 的 \r\n 分行），末尾 len=<总段数>。
void Cli::printRaw(const Signal& sig, uint32_t len)
{
    char buf[160];
    char* p = buf;
    uint8_t crlf[] = "\r\n";

    for (uint32_t i = 0; i < len; ++i)
    {
        // 预留尾部 len=NNNNN 空间(24B)，块写满就裸发一块再续写，串口侧拼成一行
        if (static_cast<uint32_t>(p - buf) > sizeof(buf) - 24)
        {
            Usart_sendString(&huart1, reinterpret_cast<uint8_t*>(buf));
            p = buf;
        }
        const uint32_t seg = sig.at(i);
        const int32_t v = sig.level(seg) ? static_cast<int32_t>(sig.us(seg))
                                         : -static_cast<int32_t>(sig.us(seg));
        p += sprintf(p, "%s%d", i ? "," : "", static_cast<int>(v));
    }

    p += sprintf(p, ",len=%u", static_cast<unsigned>(len));
    Usart_sendString(&huart1, reinterpret_cast<uint8_t*>(buf));
    Usart_sendString(&huart1, crlf);
}

// 诊断：连续采样 ADC 1s，统计 min/max/avg 与阈值穿越次数。
// 期间按遥控器按键，可看出 PA0 实际电平特征：
//   - 干净信号: 空闲≈4095、载波≈0，min≈0 max≈4095
//   - 浮空/噪声: 不按键时 min 也远小于 max、edges 很大
void Cli::onDbg()
{
    reply("DBG: press remote button within 1s...");

    ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    uint32_t minV = 4095, maxV = 0, sum = 0, cnt = 0, edges = 0;
    uint16_t prev = 0;
    {
        HAL_ADC_PollForConversion(&hadc1, 1);
        prev = static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
    }
    const uint32_t end = HAL_GetTick() + 1000;
    while (HAL_GetTick() < end)
    {
        HAL_ADC_PollForConversion(&hadc1, 1);
        const uint16_t v = static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sum += v;
        ++cnt;
        const uint16_t d = (v > prev) ? (v - prev) : (prev - v);
        if (d > Receiver::kEdgeThreshold)
            ++edges;
        prev = v;
    }
    HAL_ADC_Stop(&hadc1);

    reply("DBG min=%u max=%u avg=%u edges=%u",
          static_cast<unsigned>(minV), static_cast<unsigned>(maxV),
          static_cast<unsigned>(cnt ? sum / cnt : 0), static_cast<unsigned>(edges));
}

// 诊断：固定 200ms 窗口抓取原始边沿，不做空闲判定，直接打印全部段（带符号格式，载波=正）。
// 用于区分：
//   - 若打印出完整多帧 NEC(如 9000,-4500,562,-1687 ... 共 60~130 段) → 录制状态机被提前判结束(逻辑问题)
//   - 若打印仍是 9000,-4500,562,-1,1,-1 ... 后信号消失 → 信号本身(或接收路径)在 ~14ms 后就没有边沿了(硬件/协议)
void Cli::onRaw()
{
    reply("RAW: capturing 200ms, press & HOLD remote...");
    signal_.clear();

    ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    // 空闲基准
    uint32_t sum = 0;
    uint16_t prev = 0;
    for (int i = 0; i < 8; ++i)
    {
        HAL_ADC_PollForConversion(&hadc1, 1);
        prev = static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
        sum += prev;
    }
    prev = static_cast<uint16_t>(sum / 8);

    bool levelHigh = false;
    bool started   = false;
    const uint32_t end = HAL_GetTick() + 200;
    while (HAL_GetTick() < end)
    {
        HAL_ADC_PollForConversion(&hadc1, 1);
        const uint16_t v = static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1));
        const uint16_t d = (v > prev) ? (v - prev) : (prev - v);
        if (d > Receiver::kEdgeThreshold)
        {
            if (!started)
            {
                started = true;
                levelHigh = (v > prev);
                __HAL_TIM_SET_COUNTER(&htim2, 0);
            }
            else
            {
                const uint32_t duration = __HAL_TIM_GET_COUNTER(&htim2);
                // 存储时电平取反：与 xxNN 录制一致，数组存真实 IR 信号(载波=高/mark)
                if (!signal_.append(!levelHigh, duration))
                    break;   // 缓冲满
                __HAL_TIM_SET_COUNTER(&htim2, 0);
                levelHigh = (v > prev);
            }
        }
        prev = v;
    }
    HAL_ADC_Stop(&hadc1);

    const uint32_t len = signal_.length();
    reply("RAW %u seg:", static_cast<unsigned>(len));
    printRaw(signal_, len);
}

void Cli::reply(const char* fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Usart_debugMsg("%s", buf);
}
