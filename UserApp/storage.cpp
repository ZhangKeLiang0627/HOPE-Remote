#include "storage.hpp"
#include "stm32f4xx_hal.h"

namespace
{
    // 槽头魔数 "IR04" 按小端组成 32bit 字（字节序 49 52 30 34）。
    // IR03→旧 64KB/128KB 一槽一扇区布局；IR04→96×2KB 逻辑槽布局。
    // 魔数递增使旧约定槽自然失效（无需擦除旧区）。
    constexpr uint32_t kMagicWord = 0x34305249u;

    // 每物理扇区的槽数上限：覆盖写入时需把同扇区其它有效槽暂存 RAM，
    // 64KB 暂存 / 2KB 每槽 = 32 槽。
    constexpr uint8_t  kSlotsPerSector = 32;

    // 每槽 2KB = 512 字（整块擦除态检查用）。
    constexpr uint32_t kWordsPerSlot = 512;   // 2KB / 4B

    // 覆盖写入/擦除时的同扇区暂存缓冲（64KB = 16384 段），放 .bss。
    constexpr uint32_t kScratchSegs = 16384;
    uint32_t scratchSegs_[kScratchSegs];

    uint8_t computeChecksum(const uint32_t* segs, uint16_t n)
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
}

uint32_t Storage::slotAddr(uint8_t slot)
{
    if (slot < 32)
        return 0x08010000u + static_cast<uint32_t>(slot) * 0x800u;          // 扇区4
    if (slot < 64)
        return 0x08020000u + static_cast<uint32_t>(slot - 32) * 0x800u;     // 扇区5 前部
    return 0x08040000u + static_cast<uint32_t>(slot - 64) * 0x800u;         // 扇区6 前部
}

uint32_t Storage::sectorOf(uint8_t slot)
{
    if (slot < 32) return FLASH_SECTOR_4;
    if (slot < 64) return FLASH_SECTOR_5;
    return FLASH_SECTOR_6;
}

uint32_t Storage::capacitySegs(uint8_t slot)
{
    (void)slot;
    return kMaxSegsPerSlot;   // 510 段
}

bool Storage::programSlot(uint8_t slot, const uint32_t* segData, uint16_t segCount)
{
    if (segCount > kMaxSegsPerSlot)
        return false;
    const uint32_t addr = slotAddr(slot);
    const uint32_t word0 = kMagicWord;
    const uint8_t  cks   = computeChecksum(segData, segCount);
    const uint32_t word1 = (0xFFu << 24) | (static_cast<uint32_t>(cks) << 16)
                         | static_cast<uint32_t>(segCount);

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word0) != HAL_OK)
        return false;
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4, word1) != HAL_OK)
        return false;

    for (uint32_t w = 0; w < segCount; ++w)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 8 + w * 4, segData[w]) != HAL_OK)
            return false;
    }
    return true;
}

bool Storage::eraseSector(uint32_t sector)
{
    FLASH_EraseInitTypeDef eraseInit = {0};
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Banks = FLASH_BANK_1;
    eraseInit.Sector = sector;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sectorError = 0;
    return HAL_FLASHEx_Erase(&eraseInit, &sectorError) == HAL_OK;
}

bool Storage::collectSiblings(uint8_t slot, ScratchEntry* sibs, uint8_t& nSib)
{
    nSib = 0;
    const uint8_t base = static_cast<uint8_t>((slot / kSlotsPerSector) * kSlotsPerSector);
    uint32_t used = 0;
    for (uint8_t s = base; s < base + kSlotsPerSector; ++s)
    {
        if (s == slot || !isValid(s))
            continue;
        const uint16_t cnt = segCountOf(s);
        if (used + cnt > kScratchSegs)
            return false;
        sibs[nSib].slot = s;
        sibs[nSib].segOffset = static_cast<uint16_t>(used);
        if (!load(s, &scratchSegs_[used], sibs[nSib].count))
            return false;
        used += sibs[nSib].count;
        ++nSib;
    }
    return true;
}

bool Storage::save(uint8_t slot, const uint32_t* segData, uint16_t segCount)
{
    if (slot >= kNumSlots || segData == nullptr)
        return false;
    if (segCount == 0 || segCount > capacitySegs(slot))
        return false;

    // 槽目标区是否整块擦除态(0xFF)：是则直接编程（无擦除）；否则走覆盖路径。
    // 不能用 isValid() 判"空"——旧约定残留数据(如 IR03)非擦除态，直接编程只能 1→0，
    // 会把魔数写不回去导致槽永久不可写。
    const uint32_t* p = reinterpret_cast<const uint32_t*>(slotAddr(slot));
    bool slotErased = true;
    for (uint32_t i = 0; i < kWordsPerSlot; ++i)
    {
        if (p[i] != 0xFFFFFFFFu)
        {
            slotErased = false;
            break;
        }
    }
    if (slotErased)
    {
        HAL_FLASH_Unlock();
        const bool ok = programSlot(slot, segData, segCount);
        HAL_FLASH_Lock();
        return ok;
    }

    // 覆盖写入：先暂存同扇区其它有效槽 → 擦整扇区 → 重写兄弟槽 → 写本槽
    ScratchEntry sibs[kSlotsPerSector];
    uint8_t nSib = 0;
    if (!collectSiblings(slot, sibs, nSib))
        return false;

    HAL_FLASH_Unlock();
    if (!eraseSector(sectorOf(slot)))
    {
        HAL_FLASH_Lock();
        return false;
    }
    for (uint8_t i = 0; i < nSib; ++i)
    {
        if (!programSlot(sibs[i].slot, &scratchSegs_[sibs[i].segOffset], sibs[i].count))
        {
            HAL_FLASH_Lock();
            return false;
        }
    }
    const bool ok = programSlot(slot, segData, segCount);   // 兄弟槽先于本槽，掉电时本槽最多退化为空
    HAL_FLASH_Lock();
    return ok;
}

bool Storage::load(uint8_t slot, uint32_t* out, uint16_t& segCount)
{
    if (!isValid(slot))
    {
        segCount = 0;
        return false;
    }

    const uint8_t* p = reinterpret_cast<const uint8_t*>(slotAddr(slot));
    segCount = static_cast<uint16_t>(p[4]) | (static_cast<uint16_t>(p[5]) << 8);
    const uint32_t* src = reinterpret_cast<const uint32_t*>(p + 8);
    for (uint16_t i = 0; i < segCount; ++i)
        out[i] = src[i];
    return true;
}

bool Storage::isValid(uint8_t slot) const
{
    if (slot >= kNumSlots)
        return false;

    const uint8_t* p = reinterpret_cast<const uint8_t*>(slotAddr(slot));
    // 魔数须与 save 写入一致（IR04）。旧约定数据(IR01/IR02/IR03)在此一律判空。
    if (p[0] != 'I' || p[1] != 'R' || p[2] != '0' || p[3] != '4')
        return false;

    const uint16_t count = static_cast<uint16_t>(p[4]) | (static_cast<uint16_t>(p[5]) << 8);
    if (count == 0 || count > capacitySegs(slot))
        return false;

    const uint32_t* src = reinterpret_cast<const uint32_t*>(p + 8);
    if (computeChecksum(src, count) != p[6])
        return false;

    return true;
}

bool Storage::erase(uint8_t slot)
{
    if (slot >= kNumSlots)
        return false;
    if (!isValid(slot))
        return true;   // 空槽 no-op

    ScratchEntry sibs[kSlotsPerSector];
    uint8_t nSib = 0;
    if (!collectSiblings(slot, sibs, nSib))
        return false;

    HAL_FLASH_Unlock();
    if (!eraseSector(sectorOf(slot)))
    {
        HAL_FLASH_Lock();
        return false;
    }
    for (uint8_t i = 0; i < nSib; ++i)
    {
        if (!programSlot(sibs[i].slot, &scratchSegs_[sibs[i].segOffset], sibs[i].count))
        {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

uint16_t Storage::segCountOf(uint8_t slot) const
{
    if (!isValid(slot))
        return 0;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(slotAddr(slot));
    return static_cast<uint16_t>(p[4]) | (static_cast<uint16_t>(p[5]) << 8);
}
