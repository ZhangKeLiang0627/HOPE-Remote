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

Cli::Cli(Storage& st, Signal& sig, Receiver& rx, IrTransmitter& irTx, RfTransmitter& rfTx)
    : storage_(st), signal_(sig), receiver_(rx), irTransmitter_(irTx), rfTransmitter_(rfTx)
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
    if (strcmp(cmd, "rftst") == 0) { onRfTest(); return; }
    if (strcmp(cmd, "rfcw") == 0)  { onRfCw();  return; }
    if (strcmp(cmd, "rfkey") == 0) { onRfKey(); return; }
    if (strcmp(cmd, "rfab") == 0)  { onRfAb();  return; }
    if (strcmp(cmd, "rfrec") == 0) { onRfRec(); return; }
    if (strcmp(cmd, "rfslow") == 0) { onRfSlow(); return; }

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

    if ((lineLen_ == 4 || lineLen_ == 5) && cmd[0] == 'c' && cmd[1] == 'l')
    {
        const uint16_t slot = parseSlot(cmd + 2, static_cast<uint8_t>(lineLen_ - 2));
        if (Storage::isValidSlot(slot))
            onClr(slot);
        else
            reply("ERR");
        return;
    }

    // 诊断命令：dbg/raw 默认采「当前通道」（最后一次 xxNN 学习用的通道），
    // 也可 dbg0/dbg4、raw0/raw4 显式指定 IR/RF，避免分不清采的是哪一路。
    if (strcmp(cmd, "dbg") == 0)          { onDbg(0xFF); return; }
    if (cmd[0] == 'd' && cmd[1] == 'b' && cmd[2] == 'g' && cmd[4] == '\0' &&
        (cmd[3] == '0' || cmd[3] == '4')) { onDbg(static_cast<uint8_t>(cmd[3] - '0')); return; }
    if (strcmp(cmd, "raw") == 0)          { onRaw(0xFF); return; }
    if (cmd[0] == 'r' && cmd[1] == 'a' && cmd[2] == 'w' && cmd[4] == '\0' &&
        (cmd[3] == '0' || cmd[3] == '4')) { onRaw(static_cast<uint8_t>(cmd[3] - '0')); return; }

    // ADC 长监听：持续采样打印，输入 'x' 退出；adcmon0/adcmon4 显式指定通道
    if (strcmp(cmd, "adcmon") == 0)                                    { onAdcMon(0xFF); return; }
    if (cmd[0] == 'a' && cmd[1] == 'd' && cmd[2] == 'c' && cmd[3] == 'm' &&
        cmd[4] == 'o' && cmd[5] == 'n' && (cmd[6] == '0' || cmd[6] == '4') &&
        cmd[7] == '\0')                                                { onAdcMon(static_cast<uint8_t>(cmd[6] - '0')); return; }

    // 原始 ADC 直读：每行一个采样值+时间戳，肉眼观察电平跳变/噪声（无信号时
    // RF 接收机 DATA 会随机跳变）。adct0/adct4 显式指定通道，输入 'x' 退出。
    if (strcmp(cmd, "adct") == 0)                                      { onAdcTest(0xFF); return; }
    if (cmd[0] == 'a' && cmd[1] == 'd' && cmd[2] == 'c' && cmd[3] == 't' &&
        (cmd[4] == '0' || cmd[4] == '4') && cmd[5] == '\0')            { onAdcTest(static_cast<uint8_t>(cmd[4] - '0')); return; }

    reply("ERR");
}

void Cli::onLearn(uint16_t slot)
{
    reply("REC %u start", static_cast<unsigned>(slot));
    // RF 帧级捕获：先等一个「连续空闲≥8ms」帧间隔界定边界，再录整帧。
    // 对拷时需持续按住原遥控按键，让发射端周期循环发帧（帧间隔~10ms），
    // 直到 REC OK 出现再松手；松开后噪声伪帧会被段长校验拒绝并清空重找。
    if (slot >= 96)
        reply("hold remote button until REC OK");
    const uint32_t t0 = HAL_GetTick();
    receiver_.start(signal_, (slot < 96) ? 0 : 4);   // IR=ADC1_CH0(PA0), RF=ADC1_CH4(PA4)

    CaptureState st;
    do
    {
        st = receiver_.poll();
    } while (st == CaptureState::WaitingEdge || st == CaptureState::WaitingGap ||
             st == CaptureState::Capturing);
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

    // RF：只保存校验通过的整帧(Done)。BufferFull 通常是空闲噪声淹没缓冲，丢弃不存。
    if (slot >= 96 && st != CaptureState::Done)
    {
        reply("REC %u DISCARD (%s)", static_cast<unsigned>(slot),
              (st == CaptureState::BufferFull) ? "noise burst" : "no valid frame");
        return;
    }

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
        irTransmitter_.play(signal_);           // IR 保持单发（HS0038 场景已工作）
    }
    else
    {
        // RF 重放按真实 EV1527 遥控习惯重复 N 帧：按键期间码流连续发送 ~200-400ms，
        // 目标设备只认连续重复的码，单帧一闪不足以触发（实测 T2L 驱动正常，
        // 之前 fs 单发 33-150ms 一闪，目标设备与万用表都难捕捉）。
        constexpr int kRfRepeats = 8;
        for (int r = 0; r < kRfRepeats; ++r)
        {
            rfTransmitter_.play(signal_);
            if (r + 1 < kRfRepeats)
                HAL_Delay(10);                  // 帧间隔 ~10ms（EV1527 码间间隔量级）
        }
    }
    reply("FS %u OK", static_cast<unsigned>(slot));
}

void Cli::onHelp()
{
    reply("LUMOS-Remote commands:");
    reply("  xxNNN  learn remote into slot (000-095 IR, 100-131 RF)");
    reply("  fsNNN  play slot (000-095 IR, 100-131 RF)");
    reply("  slots  list slot occupancy");
    reply("  duNNN  dump slot segment durations (debug)");
    reply("  clNNN  clear slot data");
    reply("  dbg    sample ADC 1s: min/max/avg/edges (current channel)");
    reply("  dbg0   ditto on IR/PA0, dbg4 on RF/PA4");
    reply("  raw    capture 3000ms raw signal & dump (current channel)");
    reply("  raw0   ditto on IR/PA0, raw4 on RF/PA4");
    reply("  adcmon live ADC monitor, type x to stop (current channel)");
    reply("  adcmon0 ditto on IR/PA0, adcmon4 on RF/PA4");
    reply("  rftst  T2L->R1 air loopback self-test (drive TX + capture RX)");
    reply("  adct   raw ADC sample each line w/ timestamp, x to stop (current ch)");
    reply("  adct0  ditto on IR/PA0, adct4 on RF/PA4");
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

void Cli::onClr(uint16_t slot)
{
    if (storage_.erase(slot))
        reply("CL %u OK", static_cast<unsigned>(slot));
    else
        reply("CL %u ERR", static_cast<unsigned>(slot));
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
void Cli::onDbg(uint8_t channel)
{
    // 0xFF 表示沿用当前通道；显式给定则先切通道（仅在变化时停 ADC 重配）
    if (channel != 0xFF && !receiver_.selectChannel(channel))
    {
        reply("ERR");
        return;
    }
    reply("DBG ch=%u(%s): press remote button within 1s...",
          static_cast<unsigned>(receiver_.channel()),
          receiver_.channel() == 4 ? "RF/PA4" : "IR/PA0");

    // 读值走 receiver_.readAdc()（含 overrun 兜底），与 xxNN 录制同一路径
    ADC1->CR2 &= ~ADC_CR2_CONT;
    if (receiver_.channel() == 4)
        ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    uint32_t minV = 4095, maxV = 0, sum = 0, cnt = 0, edges = 0;
    uint16_t prev = 0;
    {
        prev = receiver_.readAdc();
    }
    const uint32_t end = HAL_GetTick() + 1000;
    while (HAL_GetTick() < end)
    {
        const uint16_t v = receiver_.readAdc();
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
void Cli::onRaw(uint8_t channel)
{
    // 0xFF 表示沿用当前通道；显式给定则先切通道（仅在变化时停 ADC 重配）
    if (channel != 0xFF && !receiver_.selectChannel(channel))
    {
        reply("ERR");
        return;
    }
    reply("RAW ch=%u(%s): capturing 3000ms, press & HOLD remote...",
          static_cast<unsigned>(receiver_.channel()),
          receiver_.channel() == 4 ? "RF/PA4" : "IR/PA0");
    signal_.clear();

    // 读值走 receiver_.readAdc()（含 overrun 兜底），与 xxNN 录制同一路径
    ADC1->CR2 &= ~ADC_CR2_CONT;
    if (receiver_.channel() == 4)
        ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    // 空闲基准
    uint32_t sum = 0;
    uint16_t prev = 0;
    for (int i = 0; i < 8; ++i)
    {
        prev = receiver_.readAdc();
        sum += prev;
    }
    prev = static_cast<uint16_t>(sum / 8);
    // 空闲线电平：HS0038(红外)空闲=高、远-R1(射频)空闲=低，存储极性以此为准
    const bool idleLevel = (prev > 2048);

    bool levelHigh = false;
    bool started   = false;
    const uint32_t end = HAL_GetTick() + 3000;
    while (HAL_GetTick() < end)
    {
        const uint16_t v = receiver_.readAdc();
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
                // 存储极性以空闲电平为准（与 xxNN 录制一致）：载波段存 mark(+)、空闲段
                // 存 space(-)。HS0038(空闲=高)等价于原取反；远-R1(空闲=低)自动翻转。
                if (!signal_.append(levelHigh != idleLevel, duration))
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

// 诊断：ADC 长监听 —— 进入后持续采样并逐窗口打印，直到输入 'x'(或 'q') 退出。
// 通道默认当前通道，可用 adcmon0/adcmon4 显式指定（与 dbg/raw 一致）。
// 每 ~40ms 打一行：v=窗口末次采样、min/max=窗口内最小/最大、ed=窗口内边沿数。
// 用途：实时观察 R1(或 HS0038) 输出电平特征——
//   干净 OOK: 每窗口见 min≈0 max≈4095 ed 上百 → 接收路径正常
//   弱接收:   电平停在中间档(如 ~2900)且 ed 个位数 → R1 只勉强听到
// 注：exit 由 UART 中断缓冲驱动，监听期间只识别单字符 'x'/'q'，不解析命令。
void Cli::onAdcMon(uint8_t channel)
{
    // 0xFF 表示沿用当前通道；显式给定则先切通道（仅在变化时停 ADC 重配）
    if (channel != 0xFF && !receiver_.selectChannel(channel))
    {
        reply("ERR");
        return;
    }
    reply("ADC-MON ch=%u(%s): type 'x' + Enter to stop",
          static_cast<unsigned>(receiver_.channel()),
          receiver_.channel() == 4 ? "RF/PA4" : "IR/PA0");

    // 与接收器录制同配置：RF(PA4)=连续模式、IR(PA0)=单次 SWSTART。
    // 读值一律走 receiver_.readAdc()（内含 overrun 兜底）——之前手写 CONT+Poll
    // 因每 40ms 打印阻塞触发 overrun，数据寄存器冻结，整屏读到恒定死值(3984)。
    ADC1->CR2 &= ~ADC_CR2_CONT;
    if (receiver_.channel() == 4)
        ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    // 空闲基准（边沿判定的前值）
    uint32_t sum = 0;
    uint16_t prev = 0;
    for (int i = 0; i < 8; ++i)
    {
        prev = receiver_.readAdc();
        sum += prev;
    }
    prev = static_cast<uint16_t>(sum / 8);

    const uint32_t kWindowMs = 40;
    bool   exit = false;
    uint32_t minV = 4095, maxV = 0, edges = 0;
    uint16_t lastV = prev;
    uint32_t windowEnd = HAL_GetTick() + kWindowMs;

    while (!exit)
    {
        // 检查退出：清空环形缓冲，命中 'x'/'q' 即退出（该行其余字节一并丢弃）
        while (!ringEmpty())
        {
            const uint8_t c = ringGet();
            if (c == 'x' || c == 'X' || c == 'q' || c == 'Q')
            {
                exit = true;
                break;
            }
        }
        if (exit)
            break;

        const uint16_t v = receiver_.readAdc();
        lastV = v;
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        const uint16_t d = (v > prev) ? (v - prev) : (prev - v);
        if (d > Receiver::kEdgeThreshold)
            ++edges;
        prev = v;

        // 每窗口打印一行并清零窗口统计
        if (HAL_GetTick() >= windowEnd)
        {
            reply("  v=%u min=%u max=%u ed=%u",
                  static_cast<unsigned>(lastV),
                  static_cast<unsigned>(minV), static_cast<unsigned>(maxV),
                  static_cast<unsigned>(edges));
            minV = 4095;
            maxV = 0;
            edges = 0;
            windowEnd = HAL_GetTick() + kWindowMs;
        }
    }

    HAL_ADC_Stop(&hadc1);
    reply("ADC-MON stopped");
}

// 诊断：原始 ADC 直读（固定单次模式），每采样一个打印一行 "t=<ms> adc=<值>"，
// 直到输入 'x' 退出。adct0/adct4 显式指定通道。
// 与 adcmon 的区别：adcmon 按窗口统计 min/max/edges；adct 逐点打原始值，
// 专门用来看电平跳变细节（如无信号时 RF 接收机 DATA 的随机噪声输出）。
void Cli::onAdcTest(uint8_t channel)
{
    // 0xFF 表示沿用当前通道；显式给定则先切通道（仅在变化时停 ADC 重配）
    if (channel != 0xFF && !receiver_.selectChannel(channel))
    {
        reply("ERR");
        return;
    }
    reply("ADC-T ch%u: raw samples, 'x' to stop", static_cast<unsigned>(receiver_.channel()));

    ADC1->CR2 &= ~ADC_CR2_CONT;   // 固定单次模式：每次显式 SWSTART，读数纯净
    HAL_ADC_Start(&hadc1);

    bool exit = false;
    while (!exit)
    {
        // 检查退出：清空环形缓冲，命中 'x'/'q' 即退出（该行其余字节一并丢弃）
        while (!ringEmpty())
        {
            const uint8_t c = ringGet();
            if (c == 'x' || c == 'X' || c == 'q' || c == 'Q')
            {
                exit = true;
                break;
            }
        }
        if (exit)
            break;

        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR | ADC_FLAG_EOC);
        ADC1->CR2 |= ADC_CR2_SWSTART;
        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
        {
            reply("t=%lu adc=%u",
                  static_cast<unsigned long>(HAL_GetTick()),
                  static_cast<unsigned>(HAL_ADC_GetValue(&hadc1)));
        }
    }

    HAL_ADC_Stop(&hadc1);
    reply("ADC-T stopped");
}

// rftst：T2L→R1 空气回环自测。MCU 经 PA2 驱动 T2L 发射合成帧，
// 同时 ADC 连续采样 PA4(R1 解调输出)，按边沿重建接收帧并打印收发对比。
// 验证：发射链路(PA2→T2L→天线→空气→R1) + 接收链路(R1→PA4→ADC) + 边沿捕获。
// TIM2 自由运行(不逐段复位)计接收段时长，避免与逐段驱动冲突。
void Cli::onRfTest()
{
    reply("RFTST T2L->R1 air loopback");

    // 合成一帧 EV1527 风格信号：sync mark(1.6ms) + 20bit(bit0=1T/3T) + 收尾 space
    Signal tx;
    tx.append(true, 1600);
    for (int i = 0; i < 20; ++i)
    {
        tx.append(false, 1200);
        tx.append(true, 400);
    }
    tx.append(false, 1200);

    // 切到 PA4 连续模式（同 xxNN 录制路径）
    receiver_.selectChannel(4);
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    // 逐段驱动 PA2 发帧；段内高频采样 PA4，用 TIM2 绝对计数重建接收边沿
    Signal rx;
    uint32_t prev       = receiver_.readAdc();
    uint32_t lastEdgeUs = 0;
    bool     havePrev   = false;
    const uint32_t nTx  = tx.length();
    uint32_t cursor     = __HAL_TIM_GET_COUNTER(&htim2);

    for (uint32_t i = 0; i < nTx; ++i)
    {
        const uint32_t target   = tx.us(tx.at(i));
        const uint32_t deadline = cursor + target;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,
                          tx.level(tx.at(i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        while (__HAL_TIM_GET_COUNTER(&htim2) < deadline)
        {
            const uint16_t s = receiver_.readAdc();
            const uint16_t d = (s > prev) ? (s - prev) : (prev - s);
            if (d > Receiver::kEdgeThreshold)
            {
                const uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
                if (havePrev)
                    rx.append(s > prev, now - lastEdgeUs);   // R1 载波=高=mark
                else
                    havePrev = true;
                lastEdgeUs = now;
            }
            prev = s;
        }
        cursor = deadline;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_ADC_Stop(&hadc1);

    reply("TX %u seg:", static_cast<unsigned>(nTx));
    printRaw(tx, nTx);
    reply("RX %u seg:", static_cast<unsigned>(rx.length()));
    printRaw(rx, rx.length());
}

// rfcw：连续载波测试。PA2 拉高 1.5s(T2L 持续辐射 433MHz)，同时采样 PA4 统计。
// 判读：avg≈4095(输出持续高) = 收到载波但被饱和钳位/OOK 无法解调；
//       avg≈1049(与空闲噪声一致) = T2L 未辐射或 R1 收不到。
void Cli::onRfCw()
{
    reply("RFCW T2L carrier ON 1.5s, sampling PA4...");
    receiver_.selectChannel(4);
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);      // 载波开
    uint32_t minV = 4095, maxV = 0, sum = 0, cnt = 0, edges = 0;
    uint16_t prevV = receiver_.readAdc();
    const uint32_t end = HAL_GetTick() + 1500;
    while (HAL_GetTick() < end)
    {
        const uint16_t v = receiver_.readAdc();
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sum += v;
        ++cnt;
        const uint16_t d = (v > prevV) ? (v - prevV) : (prevV - v);
        if (d > Receiver::kEdgeThreshold)
            ++edges;
        prevV = v;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // 载波关
    HAL_ADC_Stop(&hadc1);
    reply("RFCW min=%u max=%u avg=%u edges=%u",
          static_cast<unsigned>(minV), static_cast<unsigned>(maxV),
          static_cast<unsigned>(cnt ? sum / cnt : 0), static_cast<unsigned>(edges));
}

// rfkey：按键触发测试。若 T2L 是带内置编码器的按键触发模块，PA2 上升沿会触发它
// 发送自身码帧（可能只有一次爆发），持续拉高反而不发。这里先采 100ms 空闲基线，
// 再 PA2 高 30ms 模拟一次按键，继续采 250ms，全程用 TIM2 抓 PA4 边沿重建段序列。
// 判读：触发后若出现 ~40 段的 EV1527 码帧 → T2L 是触发型，已确认能发射；
//       全程仍只有噪声 → T2L 未辐射/未连接/未供电。
void Cli::onRfKey()
{
    reply("RFKEY trigger T2L (PA2 30ms pulse), capture PA4 350ms");
    receiver_.selectChannel(4);
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    Signal     rx;
    uint32_t   prev       = receiver_.readAdc();
    uint32_t   lastEdgeUs = 0;
    bool       havePrev   = false;
    const uint32_t t0     = __HAL_TIM_GET_COUNTER(&htim2);

    // 窗口 1：0~100ms 空闲基线
    while (__HAL_TIM_GET_COUNTER(&htim2) - t0 < 100000)
    {
        const uint16_t s = receiver_.readAdc();
        const uint16_t d = (s > prev) ? (s - prev) : (prev - s);
        if (d > Receiver::kEdgeThreshold)
        {
            const uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
            if (havePrev) rx.append(s > prev, now - lastEdgeUs);
            else          havePrev = true;
            lastEdgeUs = now;
        }
        prev = s;
    }

    // 触发：PA2 高 30ms 再低（模拟一次按键）
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    const uint32_t tPulse = __HAL_TIM_GET_COUNTER(&htim2);
    while (__HAL_TIM_GET_COUNTER(&htim2) - tPulse < 30000)
    {
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

    // 窗口 2：触发后再采 250ms（T2L 若发码帧会在此出现）
    while (__HAL_TIM_GET_COUNTER(&htim2) - t0 < 350000)
    {
        const uint16_t s = receiver_.readAdc();
        const uint16_t d = (s > prev) ? (s - prev) : (prev - s);
        if (d > Receiver::kEdgeThreshold)
        {
            const uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
            if (havePrev) rx.append(s > prev, now - lastEdgeUs);
            else          havePrev = true;
            lastEdgeUs = now;
        }
        prev = s;
    }

    HAL_ADC_Stop(&hadc1);
    reply("RFKEY RX %u seg:", static_cast<unsigned>(rx.length()));
    printRaw(rx, rx.length());
}

// rfab：AB 相位测试。PA2 高 500ms(载波ON) 再低 500ms(载波OFF)，各相位采样 PA4 统计。
// 判读（R1 反极性：载波=输出低）：
//   HI 相位 avg≈0(输出稳低) 且 LO 相位 avg≈1039(恢复噪声) → T2L 跟随 PA2 开关 ✓
//   两相位都 avg≈0(持续低) → T2L 起振后锁死不停振 ✗
void Cli::onRfAb()
{
    reply("RFAB PA2 HI 500ms + LO 500ms, sampling PA4 each");
    receiver_.selectChannel(4);
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    uint32_t minV = 4095, maxV = 0, sum = 0, cnt = 0, edges = 0;
    uint16_t prevV = receiver_.readAdc();
    const uint32_t end = HAL_GetTick() + 500;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);       // 相位A：载波ON
    while (HAL_GetTick() < end)
    {
        const uint16_t v = receiver_.readAdc();
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sum += v;
        ++cnt;
        const uint16_t d = (v > prevV) ? (v - prevV) : (prevV - v);
        if (d > Receiver::kEdgeThreshold)
            ++edges;
        prevV = v;
    }
    reply("RFAB HI: min=%u max=%u avg=%u edges=%u",
          static_cast<unsigned>(minV), static_cast<unsigned>(maxV),
          static_cast<unsigned>(cnt ? sum / cnt : 0), static_cast<unsigned>(edges));

    minV = 4095; maxV = 0; sum = 0; cnt = 0; edges = 0;
    prevV = receiver_.readAdc();
    const uint32_t end2 = HAL_GetTick() + 500;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);      // 相位B：载波OFF
    while (HAL_GetTick() < end2)
    {
        const uint16_t v = receiver_.readAdc();
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sum += v;
        ++cnt;
        const uint16_t d = (v > prevV) ? (v - prevV) : (prevV - v);
        if (d > Receiver::kEdgeThreshold)
            ++edges;
        prevV = v;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_ADC_Stop(&hadc1);
    reply("RFAB LO: min=%u max=%u avg=%u edges=%u",
          static_cast<unsigned>(minV), static_cast<unsigned>(maxV),
          static_cast<unsigned>(cnt ? sum / cnt : 0), static_cast<unsigned>(edges));
}

// rfslow：慢帧回环测试。信号弱时 R1 只对长载波段有响应(400μs mark 掉出检测门限，
// 1600μs sync 能检出)，故用码元 2000μs 的慢帧，验证 R1 能否跟完整帧。
// 帧结构：sync(1600) + 10×[space(2000), mark(2000)] + space(2000) = 22 段/43.6ms，
// 全部 1600~2000μs，满足 validateFrame(≥10段/≥1长段/段长250~5000)。
void Cli::onRfSlow()
{
    reply("RFSLOW slow frame (mark/space 2000us) loopback");
    Signal tx;
    tx.append(true, 1600);                       // sync
    for (int i = 0; i < 10; ++i)
    {
        tx.append(false, 2000);
        tx.append(true, 2000);
    }
    tx.append(false, 2000);

    receiver_.selectChannel(4);
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    Signal rx;
    uint32_t prev       = receiver_.readAdc();
    uint32_t lastEdgeUs = 0;
    bool     havePrev   = false;
    const uint32_t nTx  = tx.length();
    uint32_t cursor     = __HAL_TIM_GET_COUNTER(&htim2);

    for (uint32_t i = 0; i < nTx; ++i)
    {
        const uint32_t target   = tx.us(tx.at(i));
        const uint32_t deadline = cursor + target;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,
                          tx.level(tx.at(i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        while (__HAL_TIM_GET_COUNTER(&htim2) < deadline)
        {
            const uint16_t s = receiver_.readAdc();
            const uint16_t d = (s > prev) ? (s - prev) : (prev - s);
            if (d > Receiver::kEdgeThreshold)
            {
                const uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
                if (havePrev)
                    rx.append(!(s > prev), now - lastEdgeUs);   // 反极性：R1 载波=低=mark
                else
                    havePrev = true;
                lastEdgeUs = now;
            }
            prev = s;
        }
        cursor = deadline;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_ADC_Stop(&hadc1);

    reply("TX %u seg:", static_cast<unsigned>(nTx));
    printRaw(tx, nTx);
    reply("RX %u seg:", static_cast<unsigned>(rx.length()));
    printRaw(rx, rx.length());
}

// rfrec：恢复时间测试。依次驱动 PA2 发 200μs/2ms/20ms 载波突发，每次突发后
// 连续采样 PA4 累计边沿数，测「边沿数首次达到 20」的时刻 = R1 从饱和低电平恢复
// 到空闲噪声的时间。量化「近距饱和锁死时间 vs 突发长度」，判断近距能否做 OOK。
// 判读：200μs 恢复 <1ms → 近距短突发可跟踪，可试慢速 OOK；都 >1s → 近距必饱和，
//       回环必须拉开 T2L 与 R1 距离(1~3m)或衰减。
void Cli::onRfRec()
{
    reply("RFREC recovery vs burst length");
    receiver_.selectChannel(4);
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    static const uint32_t bursts[] = { 200, 2000, 20000 };
    for (unsigned i = 0; i < sizeof(bursts) / sizeof(bursts[0]); ++i)
    {
        const uint32_t us = bursts[i];
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);     // 突发 ON
        const uint32_t tb = __HAL_TIM_GET_COUNTER(&htim2);
        while (__HAL_TIM_GET_COUNTER(&htim2) - tb < us)
        {
        }
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // 突发 OFF

        uint32_t edges = 0;
        uint16_t prev = receiver_.readAdc();
        const uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < 3000)
        {
            const uint16_t v = receiver_.readAdc();
            const uint16_t d = (v > prev) ? (v - prev) : (prev - v);
            if (d > Receiver::kEdgeThreshold)
                ++edges;
            prev = v;
            if (edges >= 20)
            {
                reply("  %u us burst -> recover %u ms", static_cast<unsigned>(us), static_cast<unsigned>(HAL_GetTick() - t0));
                goto next;
            }
        }
        reply("  %u us burst -> NO recover in 3s (edges=%u)", static_cast<unsigned>(us), static_cast<unsigned>(edges));
    next:;
    }
    HAL_ADC_Stop(&hadc1);
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
