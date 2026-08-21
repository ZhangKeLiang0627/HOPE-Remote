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

## 8. 文件索引

| 文件 | 职责 |
|---|---|
| `UserApp/slot_store.hpp` | 模板 `SlotStore<Traits>`、`WaveTraits`、`CodeTraits`、`SlotGroup`、`FlashDriver` 声明 |
| `UserApp/slot_store.cpp` | `FlashDriver` 实现、共享暂存缓冲 `g_scratchSegs_` |
| `UserApp/signal.hpp` | `Signal` 波形载体（IR 负载），2KB/510 段 |
| `UserApp/main.cpp` | `IrStore` / `RfStore` 实例化并注入 `Cli` |
| `UserApp/cli.cpp` | `xx/fs/slots/du/cl/sc` 等命令到存储 API 的映射 |
| `UserApp/ir_receiver.*` / `rf_receiver.*` / `*_transmitter.*` | 收发器，消费存储层读写结果 |
