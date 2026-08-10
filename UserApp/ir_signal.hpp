#ifndef IR_SIGNAL_HPP
#define IR_SIGNAL_HPP

#include <array>
#include <cstdint>

// 红外波形数据载体。
//
// 段编码（4 字节/段）：
//   uint32_t seg : bit31 = 电平(1=载波段/mark, 0=无载波空间段/space) | bit30:0 = 时长(1~2147483647 μs)
// 电平为真实 IR 信号约定（接收时已对 HS0038 反相输出取反），发送直接按此电平驱动。
// 超长电平由 append() 自动拆成多段同电平（防御性，真实 IR 段不会触发）。
//
// 缓冲固定 2KB（510 段），与 Flash 槽容量一致。此对象应作为全局/静态成员持有，避免大数组入栈。
class IrSignal
{
public:
    static constexpr uint32_t kMaxSegments = 510;     // 2KB 槽 / 4B（与 IrStorage::kMaxSegsPerSlot 对齐）
    static constexpr uint32_t kMaxSegUs    = 0x7FFFFFFF; // bit30:0 上限

    void clear() { len_ = 0; }

    // 设置段数（供从 Flash 载入后使用）。n 超上限返回 false 且不变。
    bool setLength(uint32_t n)
    {
        if (n > kMaxSegments)
            return false;
        len_ = n;
        return true;
    }

    // 追加一段。us > kMaxSegUs 时自动分包；缓冲满返回 false（已录部分保留）。
    bool append(bool level, uint32_t us);

    bool empty() const { return len_ == 0; }
    uint32_t length() const { return len_; }
    uint32_t at(uint32_t i) const { return seg_[i]; }

    bool level(uint32_t seg) const { return (seg >> 31) & 1; }
    uint32_t us(uint32_t seg) const { return seg & 0x7FFFFFFFu; }

    uint32_t* data() { return seg_.data(); }
    const uint32_t* data() const { return seg_.data(); }

private:
    std::array<uint32_t, kMaxSegments> seg_;
    uint32_t len_ = 0;
};

#endif // IR_SIGNAL_HPP
