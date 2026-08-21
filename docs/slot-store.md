# 槽位存储层设计文档（SlotStore）

> 本文档梳理 HOPE-Remote 的**统一槽位存储层**——Flash 槽区管理（地址映射 / 整槽擦除态检测 / 覆盖写暂存重写 / 擦扇区 / 校验）的全部实现与两类存储的差异。
> 日常使用与 CLI 命令见根目录 [README.md](../README.md)；RF 串口模块的调试与回放时序见 [docs/rf-uart2-debug.md](rf-uart2-debug.md)。

## 1. 设计目标

Flash 槽区管理流程（地址映射、`0xFF` 整槽检测、覆盖写暂存重写、擦扇区、校验）**只实现一次**，收敛进模板 `SlotStore<Traits>`；每类存储的差异（槽大小、魔数、槽头布局、序列化）则收敛进各自的 `Traits`。

- **IR 波形槽**：`IrStore = SlotStore<WaveTraits>`，存完整红外波形（段数组），2KB/槽，槽 0-95。
- **RF 码值槽**：`RfStore = SlotStore<CodeTraits>`，只存一个 EV1527 码值（24bit + 脉宽），8B/槽，槽 100-611。

原 `storage.hpp/cpp`（旧 `Storage` 类）已被移除；平移时**保持 IR 波形槽读写格式不变**（`"IR04"` 魔数 + 8B 槽头 + 段数组），旧工程数据可直接兼容。

## 2. 架构总览

```
                  ┌─────────────────────────────────────────────┐
                  │              SlotStore<Traits>              │
                  │  save / load / isValid / erase / countOf   │
                  │  （覆盖写 = 暂存兄弟槽 → 擦扇区 → 重写）     │
                  └───────────────┬───────────────┬─────────────┘
                                  │ 特化            │ 特化
                  ┌───────────────▼──────┐   ┌──────▼──────────────┐
                  │   WaveTraits (IR)    │   │   CodeTraits (RF)   │
                  │  encode/decode/      │   │  encode/decode/     │
                  │  validate/checksum/  │   │  validate/countOf/  │
                  │  countOf / groups    │   │  groups             │
                  └───────────────┬──────┘   └──────┬──────────────┘
                                  │                │
                  ┌───────────────▼────────────────▼──────────────┐
                  │        FlashDriver 命名空间（HAL 封装）         │
                  │  unlock / lock / programWord / eraseSector     │
                  │  共享暂存缓冲 g_scratchSegs_[16384]（64KB）     │
                  └────────────────────────────────────────────────┘
```

`main.cpp` 以成员方式各持一个实例，注入 `Cli`：

```cpp
IrStore    irStore_;   // IR 波形槽 0-95（扇区4/5/6）
RfStore    rfStore_;   // RF 码值槽 100-611（扇区7 前 4KB）
// ...
Cli cli_{irStore_, rfStore_, rfReceiver_, signal_, irReceiver_,
         irTransmitter_, rfTransmitter_};
```

## 3. 槽位物理布局

| 槽号 | 介质 | 每槽大小 | 槽数 | 物理扇区 | 扇区基址 | 槽内格式 |
|---|---|---|---|---|---|---|
| 0-95 | 红外 | 2KB | 96（3 组 × 32） | SECTOR_4 / 5 / 6 | 0x08010000 / 0x08020000 / 0x08040000 | `"IR04"` 头 + 段数组 |
| 96-99 | — | — | 保留（无效区，两类存储均不覆盖） | — | — | — |
| 100-611 | 射频 | 8B | 512 | SECTOR_7 | 0x08060000 | `"RF"` 头 + pulse + code24 + XOR |

槽号路由规则：`SlotStore<Traits>::isValidSlot(slot)` 通过 `groupOf()` 判定。
- `0 ≤ slot < 96` → IR 路径（`IrStore`）
- `100 ≤ slot ≤ 611` → RF 路径（`RfStore`）
- `96-99` 与越界值 → 两路均拒绝，命令回 `ERR`。

## 4. 槽头与序列化格式

### 4.1 红外波形槽（WaveTraits）

完整 IR 波形由 `Signal` 承载——段数组，每段 4 字节：`bit31 = 电平（1=载波段/mark，0=空间段/space）`，`bit30:0 = 时长（μs）`。槽容量与 `Signal` 缓冲严格对齐，编译期断言保障：

```cpp
static_assert(Signal::kMaxSegments == WaveTraits::kMaxPayload,
              "Signal 缓冲须与 IR 槽容量一致");   // 均为 510 段 / 2KB
```

槽字节布局（2KB = 8B 槽头 + 510×4B 段数组）：

```
偏移   内容
[0..3] word0 = 0x34305249  ("IR04" 小端魔数)
[4..7] word1 = (0xFF<<24) | (cksum<<16) | count
               bits 0..15  : 段数 count（1-510）
               bits 16..23 : 段数组 XOR 校验
               bits 24..31 : 保留 0xFF
[8..]  段数组 uint32_t，最多 510 段（2048B − 8B 槽头）
```

- **校验**：对段数组中每个 `uint32_t` 的 4 字节逐字节异或，存入 `word1` 的 bits 16-23。
- `validate()`：魔数匹配 + `0 < count ≤ 510` + 校验和匹配，三者齐备才判定有效。

### 4.2 射频码值槽（CodeTraits）

只存一个 EV1527 码，布局极简（8B）：

```
偏移   内容
[0]    'R'  (0x52)
[1]    'F'  (0x46)
[2]    pulseUs & 0xFF          （编码脉宽 μs，默认 320）
[3]    (pulseUs >> 8) & 0xFF
[4]    code24 & 0xFF          （24bit EV1527 码，小端）
[5]    (code24 >> 8) & 0xFF
[6]    (code24 >> 16) & 0xFF
[7]    XOR( [0..6] )            （前 7 字节异或校验）
```

- `decode()` 还原 `pulseUs` 与 `code24`；`validate()` 校验魔数与 XOR。
- `code24` 取 hex 前 24 位，与 RCSwitch / 433_test_arduino 约定一致（hex8 时取 `>>8`，hex6 时取低 24 位）。

## 5. 核心 API（SlotStore\<Traits\>）

| 方法 | 说明 |
|---|---|
| `static bool isValidSlot(slot)` | 槽号是否落在本存储区（路由判定） |
| `bool save(slot, payload)` | 写入：空槽直接编程；已写槽走覆盖写路径 |
| `bool load(slot, payload)` | 读取 + 校验；空/无效返回 `false` |
| `bool isValid(slot)` | 槽头校验通过（魔数 + 校验和） |
| `bool erase(slot)` | 擦除：先暂存同扇区其它有效槽再擦扇区；空槽 no-op |
| `uint16_t countOf(slot)` | 负载数（IR=段数 / RF=1）；空/无效返回 0 |

### 覆盖写 / 擦除机制（掉电安全）

Flash 只能从 `1` 擦到 `0`，无法直接覆盖。模板统一实现如下：

1. **空槽直写**：`slotErased()` 检测整槽全 `0xFF` → 直接 `programSlot()`。
2. **已写槽覆盖**：
   - `collectSiblings()`：把**同扇区其它有效槽**整槽字节复制到共享暂存缓冲 `g_scratchSegs_`；
   - `eraseSector()`：擦整扇区；
   - 先重写所有兄弟槽，再 `programSlot()` 写本槽。
3. **擦除**：同覆盖写，但跳过目标槽（目标槽留在擦除态）。

> **掉电安全要点**：兄弟槽先于本槽落盘。若中途掉电，本槽最多退化为「空」（仍可被 `validate()` 拒收），**不会破坏同扇区既有数据**。

### FlashDriver 原语（实现见 slot_store.cpp）

| 函数 | 封装 |
|---|---|
| `unlock()` | `HAL_FLASH_Unlock()` |
| `lock()` | `HAL_FLASH_Lock()` |
| `programWord(addr, word)` | `HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, …)` |
| `eraseSector(sector)` | `HAL_FLASHEx_Erase()`，Bank1 / 单扇区 / `VOLTAGE_RANGE_3` |

- **共享暂存缓冲**：`uint32_t g_scratchSegs_[16384]` = 64KB。IR 单扇区 32×2KB = 64KB（31 兄弟槽 = 62KB，恰不溢出）；RF 单扇区 512×8B = 4KB，远小于缓冲，无压力。
- 擦除操作串行发生，同一时刻仅一个存储使用，`IrStore` 与 `RfStore` 无并发冲突。

## 6. CLI 命令 → 存储层映射

所有录制/回放是阻塞操作，期间 USART1/2 中断照常收字节。

| 命令 | 流向 | 存储层调用 |
|---|---|---|
| `xxNNN` | 学习 | IR：`IrReceiver` 采波形 → `irStore_.save(slot, signal_)`；RF：`onLearnRf` 取 `LC:hex` 帧 → `rfStore_.save(slot, p)` |
| `fsNNN [f b g bg]` | 回放 | IR：`irStore_.load` → `IrTransmitter::play`；RF：`rfStore_.load` → `encodeRfVariant` → `RfTransmitter::play`（参数化帧/簇/间隔） |
| `slots` | 列表 | 遍历 `irStore_.countOf` / `rfStore_.countOf`，打印占用与码值 |
| `duNNN` | 查看 | `load` 后打印 IR 段数组或 RF `code24` + `pulseUs` |
| `clNNN` | 清除 | `erase(slot)` |
| `scNNN<hex>` | 直接编程 | 解析 hex → `rfStore_.save(slot, p)`（免学习写入已知码） |

### RF 学习去抖

`onLearnRf` 轮询 `RfReceiver` 取 `LC:hex` 帧，**连续 2 帧相同码即确认保存**（按下瞬间完成），15s 总超时兜底。比早期「连读相同码 + 500ms 静默」更灵敏，实测长按 1 秒即可学完。

## 7. 约束与注意事项

- **槽号范围**：IR 0-95、RF 100-611；96-99 为保留无效区，命令会回 `ERR`。
- **信号缓冲对齐**：`Signal::kMaxSegments`（510）必须与 `WaveTraits::kMaxPayload`（510）一致，否则编译失败。
- **IR 槽容量**：单槽 2KB 上限 ≈ 510 段；超长红外协议（如空调长码）可能超出，需关注 `BufferFull` 返回。
- **RF 校验局限**：8B 布局仅含单字节 XOR，用于检测整字节翻转，非抗误码强校验。
- **发射参数与存储解耦**：存储只记 `code24` + `pulseUs`；实际发射的帧数/簇数/间隔由 `fsNNN` 参数决定（默认 `8 1 5 300`），不在槽位内持久化。

## 8. STM32F401 Flash 扇区分布与读/擦/写原理

本节解释硬件底层：Flash 扇区是如何划分的、为什么代码与数据要分区，以及「读 / 擦除 / 写 / 删除」各自依赖的物理规则——这是理解上面覆盖写机制的前提。

### 8.1 扇区分布（STM32F401RE，512KB）

STM32F4 的 Flash 不是均匀分页，而是**不等长扇区**；擦除只能以扇区为最小单位。F401RE 的布局为：

| 扇区 | 容量 | 地址范围 | 本工程用途 |
|---|---|---|---|
| 0 | 16 KB | 0x0800_0000 – 0x0800_3FFF | 固件（代码 + 中断向量） |
| 1 | 16 KB | 0x0800_4000 – 0x0800_7FFF | 固件 |
| 2 | 16 KB | 0x0800_8000 – 0x0800_BFFF | 固件 |
| 3 | 16 KB | 0x0800_C000 – 0x0800_FFFF | 固件 |
| 4 | 64 KB | 0x0801_0000 – 0x0801_FFFF | IR 波形组 0（槽 0-31） |
| 5 | 128 KB | 0x0802_0000 – 0x0803_FFFF | IR 波形组 1（槽 32-63，用前 64KB） |
| 6 | 128 KB | 0x0804_0000 – 0x0805_FFFF | IR 波形组 2（槽 64-95，用前 64KB） |
| 7 | 128 KB | 0x0806_0000 – 0x0807_FFFF | RF 码值槽（槽 100-611，仅用前 4KB） |

- 代码（扇区 0-3，共 64KB）与数据（扇区 4-7）严格分区，**互不重叠**，因此擦除数据扇区绝不会破坏固件。
- 扇区 5/6/7 物理上是 128KB，但我们每个 IR 组只用前 64KB（32 槽 × 2KB），RF 仅用 4KB；剩余空间保持擦除态（全 `0xFF`）闲置。
- 槽地址计算：`slotAddr = 扇区基址 + (槽号 − 组基) × 每槽字节`，与代码 `groupOf()` / `slotAddr()` 一致。

### 8.2 NOR Flash 物理特性（读 / 写 / 擦除的根本约束）

```
bit 只能单向翻转：   1 ──编程──▶ 0      （写）
                    0 ──擦除──▶ 1      （擦除，按扇区）
                    0 ──×──▶ 0          （不能再写 0→1，必须擦除）
```

- **位只能从 1 变成 0（编程/写）**，无法从 0 直接变回 1。
- **变回 1 只能靠擦除，且擦除以扇区为最小粒度**（F401 没有「页/字擦除」，只有扇区擦除与整片擦除）。
- 擦除后整扇区的每个 bit 都是 1，表现为全 `0xFF`。
- 由此推出「改写一个已写槽」的本质：**先把整扇区擦成全 1，再把需要为 0 的位写 0**。这正是第 5 节覆盖写机制（暂存兄弟槽 → 擦扇区 → 重写）的硬件根源。

### 8.3 解锁 / 加锁

Flash 复位后处于锁定态（`FLASH_CR.LOCK=1`），任何写/擦操作前必须解锁：

```cpp
HAL_FLASH_Unlock();          // 向 FLASH_KEYR 写入解锁序列(KEY1/KEY2)
// ... erase / program ...
HAL_FLASH_Lock();            // 操作结束后重新上锁
```

代码里每个 `save()` / `erase()` 序列都成对包裹 `unlock → … → lock`，与 `slot_store.cpp` 的 `FlashDriver` 实现一致。

### 8.4 编程（写）

- **最小单位 = 字（32-bit）**，代码用 `FLASH_TYPEPROGRAM_WORD`。
- **地址必须 4 字节对齐**：槽地址 = `扇区基址 + (槽号−组基)×kSlotBytes`，而 `kSlotBytes`（2048 或 8）都是 4 的倍数，天然对齐。
- `HAL_FLASH_Program()` 内部等待 `FLASH_SR.BUSY` 清除并置 `EOP`（编程结束）标志。
- **只能把 1 写成 0**；若目标位已是 0，再写 0 不会改变它（不会「翻转」成 1）——因此写入前该位必须是擦除态 1。

### 8.5 擦除

```cpp
FLASH_EraseInitTypeDef e = {0};
e.TypeErase    = FLASH_TYPEERASE_SECTORS;
e.Banks        = FLASH_BANK_1;
e.Sector       = sector;          // 如 FLASH_SECTOR_5
e.NbSectors    = 1;
e.VoltageRange = FLASH_VOLTAGE_RANGE_3;   // 2.7V–3.6V，匹配板载 3.3V，支持字编程
HAL_FLASHEx_Erase(&e, &sectorError);
```

- **以扇区为单位**：擦一个扇区会清空该扇区全部内容（64KB 或 128KB），无法只清某个槽。
- 擦除耗时毫秒级且**随扇区容量增长**；期间 Flash 总线被占用。F401 为**单 Flash bank**，CPU 取指来自同一 bank，因此擦除/编程时 CPU 会**阻塞停顿**。我们的 `save/erase` 是阻塞调用，期间不做实时任务，这是设计上可接受的前提。

### 8.6 「删除 / 清除」的实现

硬件没有「只清一个槽」的能力。本工程的 `erase(slot)` 实现为：

1. 把**同扇区其余有效槽（兄弟槽）**整槽字节暂存到 RAM 缓冲；
2. 擦除**整扇区**；
3. 把兄弟槽写回，**目标槽不参与重写** → 留在全 `0xFF` 的擦除态。

之后读取时：`validate()` 因魔数不符判为无效；`slotErased()` 检测全 `0xFF` 也判为空。这就是「删除」——不是把槽位清零，而是**让它回到未被写入的擦除态**。

### 8.7 为什么要 64KB 暂存缓冲 `g_scratchSegs_`

改写一个槽时，要保住同扇区的其它槽，必须先把它们搬到 RAM：

- 缓冲 = `uint32_t[16384]` = **64KB**。
- IR 在扇区 4（64KB = 32 槽）改写时最多 31 兄弟槽 × 2KB = 62KB；扇区 5/6（128KB 物理但只用 64KB）同理 62KB；RF 仅 4KB。均 < 64KB，充裕。
- 约束：`collectSiblings()` 在 `used + kSlotBytes > kScratchBytes` 时返回 `false` 拒绝改写，即**单扇区内同时存在的有效槽总大小不能超过 64KB**。当前布局远未触及该上限（若未来在某 128KB 扇区放超过 32 个 2KB 槽，则会触发）。

### 8.8 与「EEPROM 仿真」的区别

标准 EEPROM 仿真用两个扇区来回搬迁 + 状态字管理磨损均衡。本工程简化为**单扇区 + 同扇区兄弟暂存**，原因是：① 每组槽数固定且少；② 改写频率低（CLI 命令触发，非高频）；③ 掉电安全靠「兄弟槽先于本槽落盘」保证（中途掉电本槽最多退化为空，不破坏既有数据）。对遥控码这类低频、小量存储，简化方案更直观、更省 RAM/Flash。

## 9. 文件索引

| 文件 | 职责 |
|---|---|
| `UserApp/slot_store.hpp` | 模板 `SlotStore<Traits>`、`WaveTraits`、`CodeTraits`、`SlotGroup`、`FlashDriver` 声明 |
| `UserApp/slot_store.cpp` | `FlashDriver` 实现、共享暂存缓冲 `g_scratchSegs_` |
| `UserApp/signal.hpp` | `Signal` 波形载体（IR 负载），2KB/510 段 |
| `UserApp/main.cpp` | `IrStore` / `RfStore` 实例化并注入 `Cli` |
| `UserApp/cli.cpp` | `xx/fs/slots/du/cl/sc` 等命令到存储 API 的映射 |
| `UserApp/ir_receiver.*` / `rf_receiver.*` / `*_transmitter.*` | 收发器，消费存储层读写结果 |
