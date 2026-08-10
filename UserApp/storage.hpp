#ifndef STORAGE_HPP
#define STORAGE_HPP

#include <cstdint>

// Flash 槽位管理：统一槽号空间，IR 与 RF 分区独立存储。
//
//   槽号 0~95    → 红外，物理扇区 4/5/6（每扇区前 64KB 放 32 槽 × 2KB）
//   槽号 96~99   → 保留/无效
//   槽号 100~131 → 射频，扇区7 前 64KB（32 槽 × 2KB）
//
// 槽头(8B)：魔数 "IR04"(4B) + segCount(u16) + XOR校验(1B) + 保留(1B)。
// 段为 4B/段(uint32)，每槽最多 510 段（2048B − 8B 槽头）。电平取反(载波=高)存储。
// 首次写入直接编程无需擦除；覆盖写入需擦整扇区并重写同扇区其它有效槽。
// 擦除态全 0xFF，魔数不符即视为空/无效槽。
class Storage
{
public:
    static constexpr uint8_t  kSlotsPerGroup  = 32;    // 每物理扇区最多 32 槽
    static constexpr uint16_t kNumSlots       = 132;   // 槽号空间 0~131（含 96~99 空洞）
    static constexpr uint16_t kMaxSegsPerSlot = 510;   // (2KB - 8B) / 4B

    // 槽号是否有效（0~95 或 100~131）。
    static bool isValidSlot(uint16_t slot);

    // 段数组 → 保存。槽空则直接编程；槽有数据则擦整扇区 + 重写同扇区其它槽。
    // 成功返回 true。
    bool save(uint16_t slot, const uint32_t* segData, uint16_t segCount);

    // 读槽数据到 out，segCount 返回实际段数。槽无效/空返回 false。
    bool load(uint16_t slot, uint32_t* out, uint16_t& segCount);

    // 槽头有效（魔数 + 校验和）返回 true。
    bool isValid(uint16_t slot) const;

    // 擦除槽；同扇区其它有效槽会被暂存并重写。空槽为 no-op。
    bool erase(uint16_t slot);

    // 槽内段数；空/无效槽返回 0。
    uint16_t segCountOf(uint16_t slot) const;

private:
    struct ScratchEntry
    {
        uint16_t slot;       // 同扇区兄弟槽号
        uint16_t count;      // 段数
        uint16_t segOffset;  // 在 scratchSegs_ 中的起始段偏移
    };

    static uint32_t slotAddr(uint16_t slot);      // 槽号 → 槽基址
    static uint32_t sectorOf(uint16_t slot);      // 槽号 → FLASH_SECTOR_*
    static uint32_t capacitySegs(uint16_t slot);  // 槽容量（段数）

    // 编程一个槽（槽头 + 数据）。目标区须已擦除(0xFF)。调用前需 HAL_FLASH_Unlock。
    static bool programSlot(uint16_t slot, const uint32_t* segData, uint16_t segCount);

    // 擦一个物理扇区。调用前需 HAL_FLASH_Unlock。
    static bool eraseSector(uint32_t sector);

    // 收集同扇区其它有效槽到 scratchSegs_ 暂存。scratch 满或读取失败返回 false。
    bool collectSiblings(uint16_t slot, ScratchEntry* sibs, uint8_t& nSib);
};

#endif // STORAGE_HPP
