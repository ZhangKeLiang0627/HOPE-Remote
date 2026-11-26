#include "rf_receiver.hpp"

#include <cstring>

// 环形缓冲与 HAL 接收缓冲（静态：HAL_UART_RxCpltCallback 无对象上下文）
uint8_t  RfReceiver::ring_[RfReceiver::kRingSize];
volatile uint32_t RfReceiver::ringHead_ = 0;
volatile uint32_t RfReceiver::ringTail_ = 0;
uint8_t  RfReceiver::rxByte_ = 0;

void RfReceiver::begin()
{
    ringHead_ = 0;
    ringTail_ = 0;
    lineLen_ = 0;
    pendingLen_ = 0;
    frameValid_ = false;
    HAL_UART_Receive_IT(&huart2, &rxByte_, 1);
}

void RfReceiver::feedByte()
{
    const uint32_t next = (ringHead_ + 1) % kRingSize;
    if (next != ringTail_)          // 缓冲满则丢弃
    {
        ring_[ringHead_] = rxByte_;
        ringHead_ = next;
    }
}

void RfReceiver::resume()
{
    HAL_UART_Receive_IT(&huart2, &rxByte_, 1);
}

void RfReceiver::flush()
{
    while (ringHead_ != ringTail_)
        ringTail_ = (ringTail_ + 1) % kRingSize;
    lineLen_ = 0;
    pendingLen_ = 0;
    frameValid_ = false;
}

bool RfReceiver::parseLine(const char* line, uint8_t len, Frame& f)
{
    const char* h = nullptr;
    uint8_t     hl = 0;

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
    f.code32 = v;
    f.nHex   = hl;
    return true;
}

bool RfReceiver::poll()
{
    while (ringHead_ != ringTail_)
    {
        const uint8_t c = ring_[ringTail_];
        ringTail_ = (ringTail_ + 1) % kRingSize;

        if (c == '\r' || c == '\n')
        {
            if (lineLen_ > 0)
            {
                line_[lineLen_] = '\0';
                // 暂存原始行（rfmon 消费）
                pendingLen_ = lineLen_;
                std::memcpy(pendingLine_, line_, static_cast<size_t>(lineLen_) + 1);
                // 尝试解析 LC:hex 帧
                frameValid_ = parseLine(line_, lineLen_, frame_);
                lineLen_ = 0;
                return true;             // 产出一行，可消费
            }
        }
        else if (lineLen_ < sizeof(line_) - 1)
        {
            line_[lineLen_++] = c;
        }
    }
    return false;
}

bool RfReceiver::take(Frame& f)
{
    if (!frameValid_)
        return false;
    f = frame_;
    frameValid_ = false;
    return true;
}

bool RfReceiver::takeRawLine(char* out, uint8_t& len)
{
    if (pendingLen_ == 0)
        return false;
    std::memcpy(out, pendingLine_, static_cast<size_t>(pendingLen_) + 1);
    len = pendingLen_;
    pendingLen_ = 0;
    return true;
}
