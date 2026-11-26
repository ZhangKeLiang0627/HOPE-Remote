#include "slot_store.hpp"

// 覆盖写/擦除时的同扇区暂存缓冲（64KB）。两种存储共享；
// 擦除操作串行，同一时刻仅一个存储使用，无并发冲突。
uint32_t g_scratchSegs_[16384];

namespace FlashDriver
{
    bool unlock()
    {
        return HAL_FLASH_Unlock() == HAL_OK;
    }

    void lock()
    {
        HAL_FLASH_Lock();
    }

    bool programWord(uint32_t addr, uint32_t word)
    {
        return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, word) == HAL_OK;
    }

    bool eraseSector(uint32_t sector)
    {
        FLASH_EraseInitTypeDef eraseInit = {0};
        eraseInit.TypeErase     = FLASH_TYPEERASE_SECTORS;
        eraseInit.Banks         = FLASH_BANK_1;
        eraseInit.Sector        = sector;
        eraseInit.NbSectors     = 1;
        eraseInit.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
        uint32_t sectorError = 0;
        return HAL_FLASHEx_Erase(&eraseInit, &sectorError) == HAL_OK;
    }
}
