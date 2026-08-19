#ifndef SLOT_STORE_HPP
#define SLOT_STORE_HPP

#include <cstdint>
#include <cstring>

#include "stm32f4xx_hal.h"
#include "signal.hpp"

// ============================================================================
// 统一槽位存储层（C++ 模板）
//
// 设计：Flash 槽区管理流程（地址映射 / 整槽 0xFF 检测 / 覆盖写暂存重写 /
//       擦扇区 / 校验）只实现一次，放入 SlotStore<Traits> 模板；每类存储的
//       差异（槽大小、魔数、槽头布局、序列化）收敛到 Traits 中。
//
//   IrStore = SlotStore<WaveTraits>   → IR 波形槽（2KB/槽，槽 0~95）
//   RfStore = SlotStore<CodeTraits>   → RF 码值槽（8B/槽，槽 100~611）
//
// 覆盖写（暂存兄弟槽 → 擦整扇区 → 重写）沿用原 Storage 的既有行为，
// 平移时保持 IR 波形槽读写格式（"IR04" 魔数 + 8B 槽头 + 段数组）不变。
// ============================================================================

// ---------------- Flash 原语（实现见 slot_store.cpp） ----------------
namespace FlashDriver
{
    bool unlock();
    void lock();
    bool programWord(uint32_t addr, uint32_t word);
    bool eraseSector(uint32_t sector);
}

// 覆盖写/擦除时的同扇区暂存缓冲（64KB = 16384 字），两种存储共享。
// 擦除操作串行发生，同一时刻只会有一个存储使用，故无并发冲突。
extern uint32_t g_scratchSegs_[16384];
constexpr uint32_t kScratchBytes = 16384u * 4u;

// ---------------- 槽组描述：槽号区间 → 物理扇区 ----------------
struct SlotGroup
{
    uint16_t base;      // 组内首槽号
    uint16_t count;     // 组内槽数
    uint32_t sector;    // FLASH_SECTOR_x
    uint32_t baseAddr;  // 扇区基址
};

// ---------------- 波形槽（IR）：2KB/槽，槽头 8B ----------------
// 槽头：word0 = "IR04"(4B) | word1 = reserved(1B)+XOR校验(1B)+段数(u16)
// 数据：段数组 uint32_t，最多 510 段（2048B − 8B 槽头）
struct WaveTraits
{
    using Payload = Signal;

    static constexpr uint16_t kSlotBytes     = 2048;
    static constexpr uint16_t kSlotsPerGroup = 32;
    static constexpr uint8_t  kNumGroups     = 3;
    static constexpr uint16_t kBaseSlot      = 0;
    static constexpr uint16_t kNumSlots      = 96;      // 3 组 × 32
    static constexpr uint16_t kMaxPayload    = 510;     // 段数上限
    static constexpr uint32_t kMagicWord     = 0x34305249u;  // "IR04" 小端

    static const SlotGroup* groups()
    {
        static const SlotGroup g[] = {
            {  0, 32, FLASH_SECTOR_4, 0x08010000u },
            { 32, 32, FLASH_SECTOR_5, 0x08020000u },
            { 64, 32, FLASH_SECTOR_6, 0x08040000u },
        };
        return g;
    }

    // 段数组 XOR 校验（对每个 word 的 4 字节逐个异或）
    static uint8_t checksum(const uint32_t* segs, uint16_t n)
    {
        uint8_t cks = 0;
        for (uint16_t i = 0; i < n; ++i)
        {
            cks ^= static_cast<uint8_t>(segs[i] & 0xFF);
            cks ^= static_cast<uint8_t>((segs[i] >> 8) & 0xFF);
            cks ^= static_cast<uint8_t>((segs[i] >> 16) & 0xFF);
            cks ^= static_cast<uint8_t>((segs[i] >> 24) & 0xFF);
        }
        return cks;
    }

    // 把波形打包成槽字节（8B 槽头 + 段数组）。n 段须 ≤ kMaxPayload。
    static void encode(const Payload& sig, uint8_t* slotBytes)
    {
        const uint16_t count = static_cast<uint16_t>(sig.length());
        uint32_t* words = reinterpret_cast<uint32_t*>(slotBytes);
        words[0] = kMagicWord;
        words[1] = (0xFFu << 24) | (static_cast<uint32_t>(checksum(sig.data(), count)) << 16)
                 | static_cast<uint32_t>(count);
        std::memcpy(words + 2, sig.data(), static_cast<size_t>(count) * 4u);
    }

    // 从槽字节解出波形。调用前须已通过 validate()。
    static void decode(const uint8_t* slotBytes, Payload& sig)
    {
        const uint32_t* words = reinterpret_cast<const uint32_t*>(slotBytes);
        const uint16_t count  = static_cast<uint16_t>(words[1] & 0xFFFF);
        sig.setLength(count);
        std::memcpy(sig.data(), words + 2, static_cast<size_t>(count) * 4u);
    }

    // 槽头有效：魔数 + 段数合法 + 校验和匹配
    static bool validate(const uint8_t* slotBytes)
    {
        const uint32_t* words = reinterpret_cast<const uint32_t*>(slotBytes);
        if (words[0] != kMagicWord)
            return false;
        const uint16_t count = static_cast<uint16_t>(words[1] & 0xFFFF);
        if (count == 0 || count > kMaxPayload)
            return false;
        const uint8_t cks = static_cast<uint8_t>((words[1] >> 16) & 0xFF);
        return checksum(words + 2, count) == cks;
    }

    // 槽内段数；空/无效返回 0
    static uint16_t countOf(const uint8_t* slotBytes)
    {
        const uint32_t* words = reinterpret_cast<const uint32_t*>(slotBytes);
        return (words[0] == kMagicWord) ? static_cast<uint16_t>(words[1] & 0xFFFF) : 0;
    }
};

// ---------------- 码值槽（RF）：8B/槽，存一个 EV1527 码 ----------------
// 布局：'R''F'(2B) + pulse(2B,u16) + code24(3B,小端) + XOR(1B，前 7B 异或)
struct CodeTraits
{
    struct Payload
    {
        uint16_t pulseUs;   // 编码脉宽（μs），默认 320
        uint32_t code24;    // 24 位 EV1527 码
    };

    static constexpr uint16_t kSlotBytes     = 8;
    static constexpr uint16_t kSlotsPerGroup = 512;    // 使用区 4KB，暂存无压力
    static constexpr uint8_t  kNumGroups     = 1;
    static constexpr uint16_t kBaseSlot      = 100;
    static constexpr uint16_t kNumSlots      = 512;    // 槽号 100~611
    static constexpr uint16_t kMaxPayload    = 1;

    static const SlotGroup* groups()
    {
        static const SlotGroup g[] = {
            {100, 512, FLASH_SECTOR_7, 0x08060000u },
        };
        return g;
    }

    static void encode(const Payload& p, uint8_t* slotBytes)
    {
        slotBytes[0] = 'R';
        slotBytes[1] = 'F';
        slotBytes[2] = static_cast<uint8_t>(p.pulseUs & 0xFF);
        slotBytes[3] = static_cast<uint8_t>((p.pulseUs >> 8) & 0xFF);
        slotBytes[4] = static_cast<uint8_t>(p.code24 & 0xFF);
        slotBytes[5] = static_cast<uint8_t>((p.code24 >> 8) & 0xFF);
        slotBytes[6] = static_cast<uint8_t>((p.code24 >> 16) & 0xFF);
        uint8_t cks = 0;
        for (int i = 0; i < 7; ++i)
            cks ^= slotBytes[i];
        slotBytes[7] = cks;
    }

    static void decode(const uint8_t* slotBytes, Payload& p)
    {
        p.pulseUs = static_cast<uint16_t>(slotBytes[2] | (slotBytes[3] << 8));
        p.code24  = static_cast<uint32_t>(slotBytes[4])
                  | (static_cast<uint32_t>(slotBytes[5]) << 8)
                  | (static_cast<uint32_t>(slotBytes[6]) << 16);
    }

    static bool validate(const uint8_t* slotBytes)
    {
        if (slotBytes[0] != 'R' || slotBytes[1] != 'F')
            return false;
        uint8_t cks = 0;
        for (int i = 0; i < 7; ++i)
            cks ^= slotBytes[i];
        return cks == slotBytes[7];
    }

    static uint16_t countOf(const uint8_t* slotBytes)
    {
        return (slotBytes[0] == 'R' && slotBytes[1] == 'F') ? 1 : 0;
    }
};

// ---------------- 通用槽位存储模板 ----------------
template <typename Traits>
class SlotStore
{
public:
    using SlotId  = uint16_t;
    using Payload = typename Traits::Payload;

    static constexpr uint16_t kSlotsPerGroup = Traits::kSlotsPerGroup;
    static constexpr uint16_t kNumSlots      = Traits::kNumSlots;
    static constexpr uint16_t kBaseSlot      = Traits::kBaseSlot;

    // 槽号是否落在本存储的槽区
    static bool isValidSlot(SlotId slot)
    {
        return groupOf(slot) != nullptr;
    }

    // 写入：槽空直接编程；已写槽走「暂存兄弟 → 擦扇区 → 重写」覆盖路径
    bool save(SlotId slot, const Payload& p)
    {
        if (!isValidSlot(slot))
            return false;
        if (slotErased(slot))
        {
            FlashDriver::unlock();
            const bool ok = programSlot(slot, p);
            FlashDriver::lock();
            return ok;
        }

        ScratchEntry sibs[kSlotsPerGroup];
        uint8_t nSib = 0;
        if (!collectSiblings(slot, sibs, nSib))
            return false;

        FlashDriver::unlock();
        if (!FlashDriver::eraseSector(groupOf(slot)->sector))
        {
            FlashDriver::lock();
            return false;
        }
        for (uint8_t i = 0; i < nSib; ++i)
        {
            if (!programBytes(sibs[i].slot, scratchPtr(sibs[i].byteOffset)))
            {
                FlashDriver::lock();
                return false;
            }
        }
        const bool ok = programSlot(slot, p);   // 兄弟槽先于本槽，掉电时本槽最多退化为空
        FlashDriver::lock();
        return ok;
    }

    // 读取 + 校验；槽空/无效返回 false
    bool load(SlotId slot, Payload& p) const
    {
        if (!isValid(slot))
            return false;
        Traits::decode(slotBytes(slot), p);
        return true;
    }

    // 槽头有效（魔数 + 校验和）
    bool isValid(SlotId slot) const
    {
        if (groupOf(slot) == nullptr)
            return false;
        return Traits::validate(slotBytes(slot));
    }

    // 擦除槽：同扇区其它有效槽暂存并重写；空槽 no-op
    bool erase(SlotId slot)
    {
        if (groupOf(slot) == nullptr)
            return false;
        if (!isValid(slot))
            return true;

        ScratchEntry sibs[kSlotsPerGroup];
        uint8_t nSib = 0;
        if (!collectSiblings(slot, sibs, nSib))
            return false;

        FlashDriver::unlock();
        if (!FlashDriver::eraseSector(groupOf(slot)->sector))
        {
            FlashDriver::lock();
            return false;
        }
        for (uint8_t i = 0; i < nSib; ++i)
        {
            if (!programBytes(sibs[i].slot, scratchPtr(sibs[i].byteOffset)))
            {
                FlashDriver::lock();
                return false;
            }
        }
        FlashDriver::lock();
        return true;
    }

    // 槽内负载数（波形=段数 / 码值=1）；空/无效返回 0
    uint16_t countOf(SlotId slot) const
    {
        if (!isValid(slot))
            return 0;
        return Traits::countOf(slotBytes(slot));
    }

private:
    struct ScratchEntry
    {
        uint16_t slot;
        uint16_t byteOffset;   // 在 g_scratchSegs_ 中的字节偏移
    };

    static const SlotGroup* groupOf(SlotId slot)
    {
        const SlotGroup* g = Traits::groups();
        for (int i = 0; i < Traits::kNumGroups; ++i)
        {
            if (slot >= g[i].base && slot < static_cast<uint16_t>(g[i].base + g[i].count))
                return &g[i];
        }
        return nullptr;
    }

    static uint32_t slotAddr(SlotId slot)
    {
        const SlotGroup* g = groupOf(slot);
        return g->baseAddr + static_cast<uint32_t>(slot - g->base) * Traits::kSlotBytes;
    }

    static const uint8_t* slotBytes(SlotId slot)
    {
        return reinterpret_cast<const uint8_t*>(slotAddr(slot));
    }

    static uint8_t* scratchPtr(uint32_t byteOffset)
    {
        return reinterpret_cast<uint8_t*>(g_scratchSegs_) + byteOffset;
    }

    // 整槽是否擦除态（全 0xFF）
    static bool slotErased(SlotId slot)
    {
        const uint8_t* p = slotBytes(slot);
        for (uint32_t i = 0; i < Traits::kSlotBytes; ++i)
        {
            if (p[i] != 0xFF)
                return false;
        }
        return true;
    }

    // 编程一个槽（槽头 + 数据，按字写入）。目标区须已擦除。
    static bool programSlot(SlotId slot, const Payload& p)
    {
        uint8_t buf[Traits::kSlotBytes];
        Traits::encode(p, buf);
        return programBytes(slot, buf);
    }

    static bool programBytes(SlotId slot, const uint8_t* buf)
    {
        const uint32_t addr = slotAddr(slot);
        const uint32_t nWords = Traits::kSlotBytes / 4u;
        const uint32_t* words = reinterpret_cast<const uint32_t*>(buf);
        for (uint32_t w = 0; w < nWords; ++w)
        {
            if (!FlashDriver::programWord(addr + w * 4, words[w]))
                return false;
        }
        return true;
    }

    // 收集同扇区其它有效槽整槽字节到暂存缓冲。scratch 满或读取失败返回 false。
    bool collectSiblings(SlotId slot, ScratchEntry* sibs, uint8_t& nSib)
    {
        nSib = 0;
        const SlotGroup* g = groupOf(slot);
        if (g == nullptr)
            return false;
        uint32_t used = 0;
        for (uint16_t s = g->base; s < static_cast<uint16_t>(g->base + g->count); ++s)
        {
            if (s == slot || !isValid(s))
                continue;
            if (used + Traits::kSlotBytes > kScratchBytes)
                return false;
            sibs[nSib].slot = s;
            sibs[nSib].byteOffset = static_cast<uint16_t>(used);
            std::memcpy(scratchPtr(used), slotBytes(s), Traits::kSlotBytes);
            used += Traits::kSlotBytes;
            ++nSib;
        }
        return true;
    }
};

using IrStore = SlotStore<WaveTraits>;
using RfStore = SlotStore<CodeTraits>;

#endif // SLOT_STORE_HPP
