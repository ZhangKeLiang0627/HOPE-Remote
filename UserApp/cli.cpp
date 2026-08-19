#include "cli.hpp"

#include <cstdio>
#include <cstring>
#include <cstdarg>

#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "interface_uart.h"
#include "stm32f4xx_hal.h"

// ---------------- USART1 接收环形缓冲（CLI 命令） ----------------
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

    // ---------------- USART2 接收环形缓冲（RF 模块串口） ----------------
    constexpr uint32_t kRfRingSize = 128;
    uint8_t  rfRing_[kRfRingSize];
    volatile uint32_t rfRingHead_ = 0;   // 写指针（ISR）
    volatile uint32_t rfRingTail_ = 0;   // 读指针（主循环）
    uint8_t  rfRxByte_ = 0;

    // 槽号上限：RF 码值槽 100~611
    constexpr uint16_t kMaxSlot = RfStore::kBaseSlot + RfStore::kNumSlots - 1;

    // RF 回放脉宽（跟随 433_test_arduino 实测：RCSwitch protocol 1）
    constexpr uint16_t kRfPulseUs = 320;

    // RF 回放帧间隔：接收端靠帧尾长空闲(31p≈10ms)判定帧结束，
    // 间隔 10ms 时相邻帧"粘连"导致个别帧解调失败（实测灯时好时坏），
    // 原装遥控器典型帧间隔 20~40ms，取 30ms。
    constexpr uint16_t kRfFrameGapMs = 30;

    // RF 回放簇间隔：目标设备（灯）接收端常为低功耗轮询模式，第一簇仅用于
    // "唤醒"，第二簇才完整收到 → 单簇发射经常无反应（实测需"连点"）。
    // 固件自动多簇连发（簇间隔 300ms，模拟快速连按）。
    constexpr uint16_t kRfBurstGapMs = 300;

    bool validSlot(uint16_t slot)
    {
        return IrStore::isValidSlot(slot) || RfStore::isValidSlot(slot);
    }

    // 2~4 位十进制 → 槽号；非法或超上限返回 0xFFFF
    uint16_t parseSlot(const char* digits, uint8_t n)
    {
        if (n < 2 || n > 4)
            return 0xFFFF;
        uint16_t v = 0;
        for (uint8_t i = 0; i < n; ++i)
        {
            if (digits[i] < '0' || digits[i] > '9')
                return 0xFFFF;
            v = static_cast<uint16_t>(v * 10 + static_cast<uint16_t>(digits[i] - '0'));
        }
        return (v <= kMaxSlot) ? v : 0xFFFF;
    }

    // 解析空格分隔的无符号数（用于 fsNNN 的可选参数）
    static bool readNum(const char*& p, uint32_t& v)
    {
        while (*p == ' ')
            ++p;
        if (*p < '0' || *p > '9')
            return false;
        v = 0;
        while (*p >= '0' && *p <= '9')
        {
            v = v * 10 + static_cast<uint32_t>(*p - '0');
            ++p;
        }
        return true;
    }

    // RF 回放编码变体：
    //   0 = RCSwitch 标准（sync 收尾 1p+31p + MSB，实测可控制灯，默认）
    //   2 = 长载波 sync 收尾（31p+1p + MSB，fsb 调试用）
    void encodeRfVariant(Signal& out, uint32_t code, uint16_t pulse, uint8_t variant)
    {
        const uint32_t p = pulse;
        out.clear();
        for (int b = 23; b >= 0; --b)        // MSB first，数据先行
        {
            const bool one = (code >> b) & 1;
            out.append(true, one ? 3 * p : 1 * p);
            out.append(false, one ? 1 * p : 3 * p);
        }
        if (variant == 2)
        {
            out.append(true, 31 * p);        // 长载波 sync（模块偏好，目标设备不认）
            out.append(false, 1 * p);
        }
        else
        {
            out.append(true, 1 * p);         // RCSwitch 标准 sync
            out.append(false, 31 * p);
        }
    }
}

// 覆盖 HAL 弱回调：USART1 → 命令环形缓冲；USART2 → RF 环形缓冲。
// 必须 extern "C" 以匹配 HAL 的 C 链接声明。
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == &huart1)
    {
        const uint32_t next = (ringHead_ + 1) % kRingSize;
        if (next != ringTail_)            // 缓冲满则丢弃
        {
            ring_[ringHead_] = rxByte_;
            ringHead_ = next;
        }
        HAL_UART_Receive_IT(&huart1, &rxByte_, 1);
    }
    else if (huart == &huart2)
    {
        const uint32_t next = (rfRingHead_ + 1) % kRfRingSize;
        if (next != rfRingTail_)
        {
            rfRing_[rfRingHead_] = rfRxByte_;
            rfRingHead_ = next;
        }
        HAL_UART_Receive_IT(&huart2, &rfRxByte_, 1);
    }
}

Cli::Cli(IrStore& ir, RfStore& rf, Signal& sig, Receiver& rx,
         IrTransmitter& irTx, RfTransmitter& rfTx)
    : irStore_(ir), rfStore_(rf), signal_(sig), receiver_(rx),
      irTransmitter_(irTx), rfTransmitter_(rfTx)
{
}

void Cli::init()
{
    ringHead_ = 0;
    ringTail_ = 0;
    lineLen_  = 0;
    HAL_UART_Receive_IT(&huart1, &rxByte_, 1);
    rfRingHead_ = 0;
    rfRingTail_ = 0;
    rfLineLen_  = 0;
    HAL_UART_Receive_IT(&huart2, &rfRxByte_, 1);
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

    if (strcmp(cmd, "help") == 0)    { onHelp();    return; }
    if (strcmp(cmd, "slots") == 0)   { onSlots();   return; }
    if (strcmp(cmd, "dbg") == 0)     { onDbg();     return; }
    if (strcmp(cmd, "raw") == 0)     { onRaw();     return; }
    if (strcmp(cmd, "evtest") == 0)  { onEvTest();  return; }
    if (strcmp(cmd, "rfmon") == 0)   { onRfMon();   return; }
    if (strcmp(cmd, "rfloop") == 0)  { onRfLoop();  return; }
    if (strcmp(cmd, "rfraw") == 0)   { onRfRaw();   return; }

    // rfscan2NNN：sync-前置比例精扫（mark 28~40p × space 1~3p），模块回收自动判 MATCH
    if (lineLen_ == 10 && strncmp(cmd, "rfscan2", 7) == 0)
    {
        const uint16_t slot = parseSlot(cmd + 7, 3);
        if (RfStore::isValidSlot(slot))
            onRfScan2(slot);
        else
            reply("ERR");
        return;
    }

    // rfscanNNN：发射槽 NNN 码的 8 种编码变体，供目标设备实测定位波形
    if (lineLen_ == 9 && strncmp(cmd, "rfscan", 6) == 0)
    {
        const uint16_t slot = parseSlot(cmd + 6, 3);
        if (RfStore::isValidSlot(slot))
            onRfScan(slot);
        else
            reply("ERR");
        return;
    }

    // fsxNNN 系列（6 字符：fss 短按 / fsl 长按 / fsa 兼容 / fsb 调试）
    // ——必须在 fsNNN 通配分支之前，否则 "fss103" 会被 fsNNN 匹配导致解析失败
    if (lineLen_ == 6 && cmd[0] == 'f' && cmd[1] == 's' &&
        (cmd[2] == 's' || cmd[2] == 'l' || cmd[2] == 'a' || cmd[2] == 'b'))
    {
        const uint16_t slot = parseSlot(cmd + 3, 3);
        if (!RfStore::isValidSlot(slot))
        {
            reply("ERR");
            return;
        }
        if (cmd[2] == 's')      onSend(slot, 0, 3, 2, 30, 300);   // 短按：2簇×3帧
        else if (cmd[2] == 'l') onSend(slot, 0, 15, 3, 30, 300);  // 长按：3簇×15帧
        else if (cmd[2] == 'a') onSend(slot, 0, 8, 3, 30, 300);   // 兼容旧 fsa
        else                    onSend(slot, 2, 8, 3, 30, 300);   // fsb：长载波 sync
        return;
    }

    // fsNNN[ f b g bg]：RF/IR 回放，可选 4 个发射参数（空格分隔）：
    //   f=每簇帧数(1~30) b=簇数(1~10) g=帧间隔ms(0~1000) bg=簇间隔ms(0~2000)
    //   默认 8帧×3簇，帧间隔30ms，簇间隔300ms。例：fs103 1 1 = 单帧单簇
    if (cmd[0] == 'f' && cmd[1] == 's')
    {
        const char* p = cmd + 2;
        uint32_t slotVal = 0;
        uint8_t  nd = 0;
        while (*p >= '0' && *p <= '9' && nd < 4)
        {
            slotVal = slotVal * 10 + static_cast<uint32_t>(*p - '0');
            ++p;
            ++nd;
        }
        if (nd < 2 || nd > 4)
        {
            reply("ERR");
            return;
        }
        const uint16_t slot = static_cast<uint16_t>(slotVal);
        if (!validSlot(slot))
        {
            reply("ERR");
            return;
        }
        uint32_t f = 8, b = 3, g = 30, bg = 300;
        if (*p == ' ')
        {
            uint32_t v;
            if (readNum(p, v)) f = v;
            if (readNum(p, v)) b = v;
            if (readNum(p, v)) g = v;
            if (readNum(p, v)) bg = v;
        }
        if (f > 30)   f = 30;
        if (b > 10)   b = 10;
        if (g > 1000) g = 1000;
        if (bg > 2000) bg = 2000;
        onSend(slot, 0, static_cast<uint8_t>(f), static_cast<uint8_t>(b),
               static_cast<uint16_t>(g), static_cast<uint16_t>(bg));
        return;
    }

    // xxNNN / duNNN / clNNN：按槽号路由 IR/RF
    const bool twoChar =
        (cmd[0] == 'x' && cmd[1] == 'x') ||
        (cmd[0] == 'd' && cmd[1] == 'u') ||
        (cmd[0] == 'c' && cmd[1] == 'l');
    if (twoChar && lineLen_ >= 4 && lineLen_ <= 6)
    {
        const uint16_t slot = parseSlot(cmd + 2, static_cast<uint8_t>(lineLen_ - 2));
        if (!validSlot(slot))
        {
            reply("ERR");
            return;
        }
        if (cmd[0] == 'x' && cmd[1] == 'x') onLearn(slot);
        else if (cmd[0] == 'd' && cmd[1] == 'u') onDump(slot);
        else onClr(slot);
        return;
    }

    // scNNN<hex6|hex8>: 把 EV1527 码直接编程到 RF 槽(100~611)，
    // hex8 取前 6 位(地址码，与 433_test_arduino/RCSwitch 约定一致)。
    if (lineLen_ >= 11 && cmd[0] == 's' && cmd[1] == 'c')
    {
        const uint16_t slot = parseSlot(cmd + 2, 3);
        if (!RfStore::isValidSlot(slot))
        {
            reply("ERR");
            return;
        }
        const char* h  = cmd + 5;
        uint32_t hn    = lineLen_ - 5;
        if (hn > 0 && h[0] == ' ')   // 容忍 sc100 62E7E8 之间的空格
        {
            ++h;
            --hn;
        }
        if (hn != 6 && hn != 8)
        {
            reply("ERR");
            return;
        }
        uint32_t full = 0;
        for (uint32_t i = 0; i < hn; ++i)
        {
            const char c = h[i];
            uint8_t v;
            if (c >= '0' && c <= '9')      v = static_cast<uint8_t>(c - '0');
            else if (c >= 'a' && c <= 'f') v = static_cast<uint8_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v = static_cast<uint8_t>(c - 'A' + 10);
            else { reply("ERR"); return; }
            full = (full << 4) | v;
        }
        onSetCode(slot, full, hn == 8);
        return;
    }

    reply("ERR");
}

void Cli::onLearn(uint16_t slot)
{
    reply("REC %u start", static_cast<unsigned>(slot));
    if (RfStore::isValidSlot(slot))
    {
        // RF：USART2 收串口模块解码帧。按住原遥控按键循环发帧，
        // 连续 3 帧相同码即认为有效，500ms 无新帧自动保存。
        reply("hold remote button until REC OK");
        onLearnRf(slot);
        return;
    }

    // ---- IR 路径（HS0038）：首边沿即录，空闲 100ms 判定结束 ----
    const uint32_t t0 = HAL_GetTick();
    receiver_.start(signal_, 0);   // IR=ADC1_CH0(PA0)

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

    if (signal_.length() == 0)
    {
        reply("REC %u EMPTY", static_cast<unsigned>(slot));
        return;
    }

    uint32_t sumUs = 0;
    for (uint32_t i = 0; i < signal_.length(); ++i)
        sumUs += signal_.us(signal_.at(i));

    if (!irStore_.save(slot, signal_))
    {
        reply("REC %u FLASHERR", static_cast<unsigned>(slot));
        return;
    }
    reply("REC %u OK (%u seg, %u us, %ums)", static_cast<unsigned>(slot),
          static_cast<unsigned>(signal_.length()), static_cast<unsigned>(sumUs),
          static_cast<unsigned>(elapsedMs));
}

// RF 学习：消费 USART2 环形缓冲攒行 → 解析 "LC:xxxxxxxx" →
// 连续 2 帧相同码即确认（参考 433_test_arduino 的两次相同确认），立即保存。
// 15s 总超时兜底。
void Cli::onLearnRf(uint16_t slot)
{
    constexpr uint8_t kNeedMatch = 2;   // 两帧相同即确认（按下瞬间即可学完）

    // 清掉残留帧与行缓冲，避免上次数据误判
    while (rfRingHead_ != rfRingTail_)
        rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
    rfLineLen_ = 0;

    const uint32_t t0 = HAL_GetTick();
    uint32_t lastCode = 0;
    uint8_t  matchCount = 0;
    bool     haveFirst = false;
    uint8_t  lastHex = 0;              // 最近一次解析成功的 hex 位数

    for (;;)
    {
        if (HAL_GetTick() - t0 >= Receiver::kTimeoutMs)
        {
            reply("REC %u TIMEOUT (%ums)", static_cast<unsigned>(slot),
                  static_cast<unsigned>(HAL_GetTick() - t0));
            return;
        }

        while (rfRingHead_ != rfRingTail_)
        {
            const uint8_t c = rfRing_[rfRingTail_];
            rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;

            if (c == '\r' || c == '\n')
            {
                if (rfLineLen_ > 0)
                {
                    rfLine_[rfLineLen_] = '\0';
                    uint32_t code32 = 0;
                    uint8_t  nHex = 0;
                    if (parseRfLine(reinterpret_cast<const char*>(rfLine_), rfLineLen_, code32, nHex))
                    {
                        if (!haveFirst || code32 != lastCode)
                        {
                            lastCode = code32;
                            matchCount = 1;
                            haveFirst = true;
                        }
                        else
                        {
                            ++matchCount;
                        }
                        lastHex = nHex;
                        // 连续两帧相同 → 立即确认保存（不等空闲）
                        if (matchCount >= kNeedMatch)
                        {
                            const uint32_t code24 = (lastHex == 8) ? ((lastCode >> 8) & 0xFFFFFFu)
                                                                   : (lastCode & 0xFFFFFFu);
                            CodeTraits::Payload p;
                            p.pulseUs = kRfPulseUs;
                            p.code24  = code24;
                            if (!rfStore_.save(slot, p))
                            {
                                reply("REC %u FLASHERR", static_cast<unsigned>(slot));
                                return;
                            }
                            reply("REC %u OK 0x%06lX pulse=%u", static_cast<unsigned>(slot),
                                  static_cast<unsigned long>(code24),
                                  static_cast<unsigned>(p.pulseUs));
                            return;
                        }
                    }
                    rfLineLen_ = 0;
                }
            }
            else if (rfLineLen_ < sizeof(rfLine_) - 1)
            {
                rfLine_[rfLineLen_++] = c;
            }
        }
    }
}

void Cli::onSend(uint16_t slot, uint8_t variant, uint8_t frames, uint8_t bursts,
                 uint16_t frameGapMs, uint16_t burstGapMs)
{
    if (IrStore::isValidSlot(slot))
    {
        if (!irStore_.load(slot, signal_))
        {
            reply("FS %u EMPTY", static_cast<unsigned>(slot));
            return;
        }
        irTransmitter_.play(signal_);           // IR 单发
        reply("FS %u OK", static_cast<unsigned>(slot));
        return;
    }

    // RF：读码值 → 编码 → PA5 直驱，参数化发射（帧/簇/间隔全可调）
    CodeTraits::Payload p;
    if (!rfStore_.load(slot, p))
    {
        reply("FS %u EMPTY", static_cast<unsigned>(slot));
        return;
    }
    encodeRfVariant(signal_, p.code24, p.pulseUs, variant);
    for (uint8_t b = 0; b < bursts; ++b)
    {
        for (uint8_t r = 0; r < frames; ++r)
        {
            rfTransmitter_.play(signal_);
            if (r + 1 < frames && frameGapMs > 0)
                HAL_Delay(frameGapMs);
        }
        if (b + 1 < bursts && burstGapMs > 0)
            HAL_Delay(burstGapMs);
    }
    reply("FS %u OK (v%u %ux%u g%u/%u)", static_cast<unsigned>(slot),
          static_cast<unsigned>(variant),
          static_cast<unsigned>(bursts), static_cast<unsigned>(frames),
          static_cast<unsigned>(frameGapMs), static_cast<unsigned>(burstGapMs));
}

void Cli::onHelp()
{
    reply("HOPE-Remote commands:");
    reply("  xxNNN  learn remote (000-095 IR, 100-611 RF)");
    reply("  fsNNN  play RF slot [f b g bg] (default 8x3, gap 30/300ms)");
    reply("         e.g. fs103 1 1 = single frame; fs103 5 3 40 400");
    reply("  fssNNN RF short-press (2 bursts x 3 frames)");
    reply("  fslNNN RF long-press (3 bursts x 15 frames)");
    reply("  fsbNNN RF long-carrier sync variant (debug)");
    reply("  slots  list slot occupancy");
    reply("  duNNN  dump slot (IR segments / RF code)");
    reply("  clNNN  clear slot data");
    reply("  scNNN  set EV1527 code into RF slot: scNNN<hex6|hex8>");
    reply("  dbg    sample IR ADC 1s: min/max/avg/edges");
    reply("  raw    capture 3000ms IR raw signal & dump");
    reply("  rfmon  listen RF UART2 stream, x to stop");
    reply("  rfloop RF air loopback self-test (TX->RX compare)");
    reply("  rfraw  sample PA4 RF demod waveform (capture real remote)");
    reply("  rfscanNNN emit 15 codec variants for target device test");
    reply("  rfscan2NNN sync-FIRST ratio scan 21 variants (module match)");
    reply("  evtest EV1527 encode->decode roundtrip self-test");
    reply("  help   show this");
}

void Cli::onSlots()
{
    uint16_t used = 0;
    uint16_t total = 0;

    for (uint16_t s = 0; s < IrStore::kNumSlots; ++s)
    {
        ++total;
        const uint16_t n = irStore_.countOf(s);
        if (n > 0)
        {
            reply("  %03u: OK (%u seg)", static_cast<unsigned>(s), static_cast<unsigned>(n));
            ++used;
        }
    }
    for (uint16_t s = RfStore::kBaseSlot; s < static_cast<uint16_t>(RfStore::kBaseSlot + RfStore::kNumSlots); ++s)
    {
        ++total;
        if (rfStore_.countOf(s) > 0)
        {
            CodeTraits::Payload p;
            rfStore_.load(s, p);
            reply("  %03u: OK (code 0x%06lX)", static_cast<unsigned>(s),
                  static_cast<unsigned long>(p.code24));
            ++used;
        }
    }
    reply("total: %u/%u used", static_cast<unsigned>(used), static_cast<unsigned>(total));
}

// 打印槽内容：IR 段数组（带符号时长）或 RF 码值
void Cli::onDump(uint16_t slot)
{
    if (IrStore::isValidSlot(slot))
    {
        if (!irStore_.load(slot, signal_))
        {
            reply("DUMP %u EMPTY", static_cast<unsigned>(slot));
            return;
        }
        reply("DUMP %u %u seg:", static_cast<unsigned>(slot),
              static_cast<unsigned>(signal_.length()));
        printRaw(signal_, signal_.length());
        return;
    }

    CodeTraits::Payload p;
    if (!rfStore_.load(slot, p))
    {
        reply("DUMP %u EMPTY", static_cast<unsigned>(slot));
        return;
    }
    reply("DUMP %u code=0x%06lX pulse=%u", static_cast<unsigned>(slot),
          static_cast<unsigned long>(p.code24), static_cast<unsigned>(p.pulseUs));
}

void Cli::onClr(uint16_t slot)
{
    const bool ok = IrStore::isValidSlot(slot) ? irStore_.erase(slot)
                                               : rfStore_.erase(slot);
    if (ok)
        reply("CL %u OK", static_cast<unsigned>(slot));
    else
        reply("CL %u ERR", static_cast<unsigned>(slot));
}

void Cli::onSetCode(uint16_t slot, uint32_t full, bool eight)
{
    // 与 433_test_arduino 一致：hex8 = 地址6位+键值2位，实际取前 24 位
    const uint32_t code24 = eight ? ((full >> 8) & 0xFFFFFF) : (full & 0xFFFFFF);
    CodeTraits::Payload p;
    p.pulseUs = kRfPulseUs;
    p.code24  = code24;
    if (!rfStore_.save(slot, p))
    {
        reply("SC %u FLASHERR", static_cast<unsigned>(slot));
        return;
    }
    reply("SC %u SET 0x%06lX pulse=%u", static_cast<unsigned>(slot),
          static_cast<unsigned long>(code24), static_cast<unsigned>(p.pulseUs));
}

void Cli::onEvTest()
{
    reply("EVTEST EV1527 codec self-test");
    const uint32_t codes[]  = {0x62E7E8u, 0x000000u, 0xFFFFFFu, 0x1A2B3Cu};
    const uint16_t pulses[] = {350, 320, 380, 350};
    bool allOk = true;
    for (int i = 0; i < 4; ++i)
    {
        Signal s;
        ev1527Encode(s, codes[i], pulses[i]);
        uint32_t code = 0;
        uint16_t pulse = 0;
        if (ev1527Decode(s, code, pulse) && code == codes[i])
        {
            reply("  #%d OK  0x%06lX (enc pulse %u -> dec pulse %u)", i,
                  static_cast<unsigned long>(codes[i]),
                  static_cast<unsigned>(pulses[i]), static_cast<unsigned>(pulse));
        }
        else
        {
            allOk = false;
            reply("  #%d FAIL enc 0x%06lX -> dec 0x%06lX", i,
                  static_cast<unsigned long>(codes[i]), static_cast<unsigned long>(code));
        }
    }
    reply(allOk ? "EVTEST PASS" : "EVTEST FAIL");
}

// 带符号逗号分隔打印：正=载波段(mark)、负=无载波空间段(space)（时长μs）。
void Cli::printRaw(const Signal& sig, uint32_t len)
{
    char buf[160];
    char* p = buf;
    uint8_t crlf[] = "\r\n";

    for (uint32_t i = 0; i < len; ++i)
    {
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

// 解析 USART2 一行：形如 "LC:55C31129"。前缀大小写容错，hex 取 6~8 位。
// 成功返回 true，code32 为 hex 数值，nHex 为 hex 位数。
bool Cli::parseRfLine(const char* line, uint8_t len, uint32_t& code32, uint8_t& nHex)
{
    const char* h = nullptr;
    uint8_t  hl = 0;

    if (len >= 9 && line[0] == 'L' && line[1] == 'C' && line[2] == ':')
    {
        h = line + 3;
        hl = static_cast<uint8_t>(len - 3);
    }
    else if (len >= 9 && line[0] == 'l' && line[1] == 'c' && line[2] == ':')
    {
        h = line + 3;
        hl = static_cast<uint8_t>(len - 3);
    }
    else
    {
        return false;
    }

    if (hl < 6 || hl > 8)
        return false;

    uint32_t v = 0;
    for (uint8_t i = 0; i < hl; ++i)
    {
        const char c = h[i];
        uint8_t d;
        if (c >= '0' && c <= '9')      d = static_cast<uint8_t>(c - '0');
        else if (c >= 'a' && c <= 'f') d = static_cast<uint8_t>(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') d = static_cast<uint8_t>(c - 'A' + 10);
        else return false;
        v = (v << 4) | d;
    }
    code32 = v;
    nHex   = hl;
    return true;
}

// 诊断：连续采样 ADC 1s 统计（IR/PA0）。期间按红外遥控器按键可看电平特征。
void Cli::onDbg()
{
    receiver_.selectChannel(0);
    reply("DBG ch=0(IR/PA0): press IR remote button within 1s...");

    ADC1->CR2 &= ~ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    uint32_t minV = 4095, maxV = 0, sum = 0, cnt = 0, edges = 0;
    uint16_t prev = receiver_.readAdc();
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

// 诊断：固定 3000ms 窗口抓 IR 原始边沿（HS0038 空闲=高）。
void Cli::onRaw()
{
    receiver_.selectChannel(0);
    reply("RAW ch=0(IR/PA0): capturing 3000ms, press & HOLD IR remote...");
    signal_.clear();

    ADC1->CR2 &= ~ADC_CR2_CONT;
    HAL_ADC_Start(&hadc1);

    uint32_t sum = 0;
    uint16_t prev = 0;
    for (int i = 0; i < 8; ++i)
    {
        prev = receiver_.readAdc();
        sum += prev;
    }
    prev = static_cast<uint16_t>(sum / 8);
    const bool idleLevel = (prev > 2048);   // HS0038 空闲=高

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

// 诊断：监听 USART2（RF 模块串口）码流，逐行打印，输入 'x' 退出。
void Cli::onRfMon()
{
    reply("RFMON listening UART2 (RF module), type 'x' + Enter to stop");
    while (rfRingHead_ != rfRingTail_)
        rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
    rfLineLen_ = 0;

    bool exit = false;
    while (!exit)
    {
        // UART1 退出检测
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

        // UART2 码流逐行打印
        while (rfRingHead_ != rfRingTail_)
        {
            const uint8_t c = rfRing_[rfRingTail_];
            rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
            if (c == '\r' || c == '\n')
            {
                if (rfLineLen_ > 0)
                {
                    rfLine_[rfLineLen_] = '\0';
                    reply("RF: %s", reinterpret_cast<const char*>(rfLine_));
                    rfLineLen_ = 0;
                }
            }
            else if (rfLineLen_ < sizeof(rfLine_) - 1)
            {
                rfLine_[rfLineLen_++] = c;
            }
        }
    }
    reply("RFMON stopped");
}

// 自测：RF 空气回环（标准 EV1527 时序）。注意：串口模块只认长载波 sync，
// 对标准 1p+31p 收不到帧属正常——本命令仅验证发射链路执行，正确性以目标设备实测为准。
void Cli::onRfLoop()
{
    constexpr uint32_t kTestCode = 0x55C311u;
    reply("RFLOOP standard EV1527 (sync-first 1p+31p, LSB): emit 0x55C311");

    while (rfRingHead_ != rfRingTail_)
        rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
    rfLineLen_ = 0;

    Signal tx;
    ev1527Encode(tx, kTestCode, kRfPulseUs);

    constexpr int kRepeats = 8;
    for (int r = 0; r < kRepeats; ++r)
    {
        rfTransmitter_.play(tx);
        if (r + 1 < kRepeats)
            HAL_Delay(kRfFrameGapMs);
    }

    uint32_t got = 0;
    bool     have = false;
    const uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 2000 && !have)
    {
        while (rfRingHead_ != rfRingTail_)
        {
            const uint8_t c = rfRing_[rfRingTail_];
            rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
            if (c == '\r' || c == '\n')
            {
                if (rfLineLen_ > 0)
                {
                    rfLine_[rfLineLen_] = '\0';
                    uint32_t code32 = 0;
                    uint8_t  nh = 0;
                    if (parseRfLine(reinterpret_cast<const char*>(rfLine_), rfLineLen_, code32, nh))
                    {
                        got = (nh == 8) ? ((code32 >> 8) & 0xFFFFFFu) : (code32 & 0xFFFFFFu);
                        have = true;
                    }
                    rfLineLen_ = 0;
                }
            }
            else if (rfLineLen_ < sizeof(rfLine_) - 1)
            {
                rfLine_[rfLineLen_++] = c;
            }
        }
    }

    if (have && got == kTestCode)
        reply("RFLOOP PASS 0x%06lX [TARGET MATCH]", static_cast<unsigned long>(got));
    else if (have)
        reply("RFLOOP MISMATCH got 0x%06lX want 0x55C311",
              static_cast<unsigned long>(got));
    else
        reply("RFLOOP no rx (normal: module rejects standard sync; verify via target device)");
}

// 诊断：PA4 波形采样（解调 DATA 输入）。用于抓取真实遥控器的空中波形，
// 分析 sync/bit 结构后精确复刻。PA4 需接串口模块的 DATA 解调输出（若有）。
void Cli::onRfRaw()
{
    if (!receiver_.selectChannel(4))
    {
        reply("ERR");
        return;
    }
    reply("RFRAW ch=4(PA4): capturing 3000ms, press & HOLD RF remote...");
    signal_.clear();

    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_CONT;          // RF 解调输出走连续模式（旧方案经验）
    HAL_ADC_Start(&hadc1);

    uint32_t sum = 0;
    uint16_t prev = 0;
    for (int i = 0; i < 8; ++i)
    {
        prev = receiver_.readAdc();
        sum += prev;
    }
    prev = static_cast<uint16_t>(sum / 8);
    const bool idleLevel = (prev > 2048);   // 空闲电平自适应（高或低）

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
                if (!signal_.append(levelHigh != idleLevel, duration))
                    break;
                __HAL_TIM_SET_COUNTER(&htim2, 0);
                levelHigh = (v > prev);
            }
        }
        prev = v;
    }
    HAL_ADC_Stop(&hadc1);

    const uint32_t len = signal_.length();
    reply("RFRAW %u seg:", static_cast<unsigned>(len));
    printRaw(signal_, len);
}

// 自测：sync-前置比例精扫。模块对 sync-FIRST 31p+1p 解出偏移码
//（0x800000|(code>>1)），说明 sync 比例与真实遥控器不同导致 bit 对齐错位。
// 精扫 mark(28~40p) × space(1~3p) 共 21 种，模块回收自动标 TARGET MATCH。
void Cli::onRfScan2(uint16_t slot)
{
    CodeTraits::Payload p;
    if (!rfStore_.load(slot, p))
    {
        reply("RFSCAN2 slot EMPTY");
        return;
    }
    const uint32_t code  = p.code24;
    const uint16_t pulse = p.pulseUs;

    static const uint16_t kMarks[] = {28, 30, 31, 32, 34, 36, 40};
    static const uint16_t kSpaces[] = {1, 2, 3};

    reply("RFSCAN2 sync-FIRST scan code=0x%06lX pulse=%u, 21 variants",
          static_cast<unsigned long>(code), static_cast<unsigned>(pulse));

    uint8_t idx = 0;
    for (uint8_t ms = 0; ms < sizeof(kMarks) / sizeof(kMarks[0]); ++ms)
    {
        for (uint8_t ss = 0; ss < sizeof(kSpaces) / sizeof(kSpaces[0]); ++ss)
        {
            ++idx;
            while (rfRingHead_ != rfRingTail_)
                rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
            rfLineLen_ = 0;

            Signal tx;
            tx.clear();
            tx.append(true, kMarks[ms] * pulse);    // sync mark（前置）
            tx.append(false, kSpaces[ss] * pulse);  // sync space
            for (int b = 23; b >= 0; --b)
            {
                const bool one = (code >> b) & 1;
                tx.append(true, one ? 3 * pulse : pulse);
                tx.append(false, one ? pulse : 3 * pulse);
            }

            for (int r = 0; r < 6; ++r)
            {
                rfTransmitter_.play(tx);
                HAL_Delay(20);
            }

            uint32_t rxCode = 0;
            bool     haveRx = false;
            const uint32_t t0 = HAL_GetTick();
            while (HAL_GetTick() - t0 < 1000 && !haveRx)
            {
                while (rfRingHead_ != rfRingTail_)
                {
                    const uint8_t c = rfRing_[rfRingTail_];
                    rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
                    if (c == '\r' || c == '\n')
                    {
                        if (rfLineLen_ > 0)
                        {
                            rfLine_[rfLineLen_] = '\0';
                            uint32_t code32 = 0;
                            uint8_t  nh = 0;
                            if (parseRfLine(reinterpret_cast<const char*>(rfLine_), rfLineLen_, code32, nh))
                            {
                                rxCode = (nh == 8) ? ((code32 >> 8) & 0xFFFFFFu)
                                                   : (code32 & 0xFFFFFFu);
                                haveRx = true;
                            }
                            rfLineLen_ = 0;
                        }
                    }
                    else if (rfLineLen_ < sizeof(rfLine_) - 1)
                    {
                        rfLine_[rfLineLen_++] = c;
                    }
                }
            }

            if (haveRx)
                reply("#%u sync%up+%up -> rx 0x%06lX%s", static_cast<unsigned>(idx),
                      static_cast<unsigned>(kMarks[ms]), static_cast<unsigned>(kSpaces[ss]),
                      static_cast<unsigned long>(rxCode),
                      (rxCode == code) ? " [TARGET MATCH]" : "");
            else
                reply("#%u sync%up+%up -> no rx", static_cast<unsigned>(idx),
                      static_cast<unsigned>(kMarks[ms]), static_cast<unsigned>(kSpaces[ss]));
        }
    }
    reply("RFSCAN2 done - report MATCH # to me");
}

// 自测：发射槽内码的编码变体，每个变体发射后监听 USART2 回收模块解码输出。
// 双重用途：① 模块对哪个变体能解出目标码 → 该变体链路正确；
//           ② 用户观察目标设备在哪个 # 响应（含 pulse 变体）。
void Cli::onRfScan(uint16_t slot)
{
    CodeTraits::Payload p;
    if (!rfStore_.load(slot, p))
    {
        reply("RFSCAN slot EMPTY");
        return;
    }
    const uint32_t code  = p.code24;
    const uint16_t pulse = p.pulseUs;

    struct Variant
    {
        uint16_t syncMark;   // sync 载波 = N×pulse
        uint16_t syncSpace;  // sync 空闲 = N×pulse
        bool     shifted;    // 位移补偿 emit = code<<1
        bool     lsbFirst;   // bit 低位先发
        bool     syncLast;   // sync 在帧尾（RCSwitch 风格，参考项目实际时序）
        uint16_t pulse;      // 0 = 用槽内 pulse；非 0 = 覆盖 pulse
    };
    const Variant kVar[] = {
        { 31,  1, false, false, false,    0 },   // 1: sync-first 31p+1p 原码
        { 31,  1, true,  false, false,    0 },   // 2: sync-first 31p+1p 位移
        {  1, 31, false, false, false,    0 },   // 3: sync-first 1p+31p 原码
        {  1, 31, true,  false, false,    0 },   // 4: sync-first 1p+31p 位移
        {  4,124, false, false, false,    0 },   // 5: sync-first 4p+124p 原码
        {  4,124, true,  false, false,    0 },   // 6: sync-first 4p+124p 位移
        { 31,  1, false, true,  false,    0 },   // 7: sync-first 31p+1p 原码 LSB
        {  1, 31, false, true,  false,    0 },   // 8: sync-first 1p+31p 原码 LSB
        {  1, 31, false, false, true,     0 },   // 9: sync-LAST 1p+31p 原码
        {  1, 31, true,  false, true,     0 },   // 10: sync-LAST 1p+31p 位移
        { 31,  1, false, false, true,     0 },   // 11: sync-LAST 31p+1p 原码 ★当前 fs
        {  1, 31, false, true,  true,     0 },   // 12: sync-LAST 1p+31p 原码 LSB
        { 31,  1, false, false, true,   350 },   // 13: sync-LAST 31p+1p 原码 pulse 350
        { 31,  1, false, false, true,   380 },   // 14: sync-LAST 31p+1p 原码 pulse 380
        { 31,  1, false, false, true,   400 },   // 15: sync-LAST 31p+1p 原码 pulse 400
    };

    reply("RFSCAN code=0x%06lX pulse=%u, 15 variants - watch target device",
          static_cast<unsigned long>(code), static_cast<unsigned>(pulse));

    for (uint8_t i = 0; i < sizeof(kVar) / sizeof(kVar[0]); ++i)
    {
        while (rfRingHead_ != rfRingTail_)
            rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
        rfLineLen_ = 0;

        const uint32_t emit = kVar[i].shifted ? ((code & 0x7FFFFFu) << 1) : code;
        const uint16_t p    = kVar[i].pulse ? kVar[i].pulse : pulse;

        Signal tx;
        tx.clear();
        const bool syncFirst = !kVar[i].syncLast;
        if (syncFirst)
        {
            tx.append(true, kVar[i].syncMark * p);
            tx.append(false, kVar[i].syncSpace * p);
        }
        if (kVar[i].lsbFirst)
        {
            for (int b = 0; b < 24; ++b)
            {
                const bool one = (emit >> b) & 1;
                tx.append(true, one ? 3 * p : p);
                tx.append(false, one ? p : 3 * p);
            }
        }
        else
        {
            for (int b = 23; b >= 0; --b)
            {
                const bool one = (emit >> b) & 1;
                tx.append(true, one ? 3 * p : p);
                tx.append(false, one ? p : 3 * p);
            }
        }
        if (!syncFirst)
        {
            tx.append(true, kVar[i].syncMark * p);
            tx.append(false, kVar[i].syncSpace * p);
        }

        for (int r = 0; r < 8; ++r)
        {
            rfTransmitter_.play(tx);
            HAL_Delay(30);
        }

        // 监听 1.5s 回收模块解码输出
        uint32_t rxCode = 0;
        bool     haveRx = false;
        const uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < 1500 && !haveRx)
        {
            while (rfRingHead_ != rfRingTail_)
            {
                const uint8_t c = rfRing_[rfRingTail_];
                rfRingTail_ = (rfRingTail_ + 1) % kRfRingSize;
                if (c == '\r' || c == '\n')
                {
                    if (rfLineLen_ > 0)
                    {
                        rfLine_[rfLineLen_] = '\0';
                        uint32_t code32 = 0;
                        uint8_t  nh = 0;
                        if (parseRfLine(reinterpret_cast<const char*>(rfLine_), rfLineLen_, code32, nh))
                        {
                            rxCode = (nh == 8) ? ((code32 >> 8) & 0xFFFFFFu)
                                               : (code32 & 0xFFFFFFu);
                            haveRx = true;
                        }
                        rfLineLen_ = 0;
                    }
                }
                else if (rfLineLen_ < sizeof(rfLine_) - 1)
                {
                    rfLine_[rfLineLen_++] = c;
                }
            }
        }

        if (haveRx)
            reply("#%u %s sync%up+%up %s%s -> rx 0x%06lX%s",
                  static_cast<unsigned>(i + 1),
                  syncFirst ? "FIRST" : "LAST ",
                  static_cast<unsigned>(kVar[i].syncMark),
                  static_cast<unsigned>(kVar[i].syncSpace),
                  kVar[i].shifted ? "shift" : "orig ",
                  kVar[i].lsbFirst ? " LSB" : "",
                  static_cast<unsigned long>(rxCode),
                  (rxCode == code) ? " [TARGET MATCH]" : "");
        else
            reply("#%u %s sync%up+%up %s%s -> no rx",
                  static_cast<unsigned>(i + 1),
                  syncFirst ? "FIRST" : "LAST ",
                  static_cast<unsigned>(kVar[i].syncMark),
                  static_cast<unsigned>(kVar[i].syncSpace),
                  kVar[i].shifted ? "shift" : "orig ",
                  kVar[i].lsbFirst ? " LSB" : "");
    }
    reply("RFSCAN done - which # toggled target? module rx above");
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
