# HOPE-Remote Firmware

STM32F401 裸机固件：CubeMX 生成底层初始化，应用层为 C++ 手写，全部功能通过 USART1 命令行走通。

- 目标器件：`STM32F401RETx`（LQFP64，SYSCLK 84 MHz，HSE 25 MHz）
- 构建：Keil MDK 5.35，ARM Compiler 6.16（AC6 / armclang）
- 依赖：STM32F4xx HAL（仓库内自带 `Drivers/`）

## 1. 环境

| 项 | 说明 |
|---|---|
| Keil MDK | 5.35 及以上，需安装 STM32F4xx_DFP 器件包 |
| 调试器 | DAPLink / CMSIS-DAP（SWD：PA13/PA14，NRST 可选） |
| OpenOCD | 0.12.x，仅脚本烧录时需要 |
| Python 3 + pyserial | 仅 `scripts/rftest.py` 自动化自测时需要 |

## 2. 编译

**IDE**：打开 `MDK-ARM/HOPE-Remote.uvprojx`，`F7` 编译，产物 `MDK-ARM/HOPE-Remote/HOPE-Remote.hex`。

**脚本**：

```bash
./scripts/build.sh
```

`build.sh` 内部硬编码了 `UV4.exe` 的绝对路径，换机器需改那一行；脚本会额外核对关键 `.o` 的时间戳、并从 `build_log.htm` 里筛 Error。

## 3. 烧录

**Keil**：`F8`（Download），首次使用在 Options → Debug 选 CMSIS-DAP。

**OpenOCD**：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 4000" \
  -c "program MDK-ARM/HOPE-Remote/HOPE-Remote.hex verify reset exit"
```

`scripts/flash.sh` 封装了同样流程，同样硬编码了 OpenOCD 安装目录。

## 4. 连接串口

| 项 | 值 |
|---|---|
| 端口 | USART1：PA9 (TX) / PA10 (RX) |
| 参数 | 115200，8 数据位，无校验，1 停止位 |
| 物理口 | 板载 CH340N → Type-C（同一根线供电 + 通信） |
| 行结尾 | `\r` 或 `\n` 均可 |
| 大小写 | 命令不敏感（`xx000` = `XX000`） |

Windows 下可直接用脚本发一条命令：

```powershell
./scripts/serial.ps1 -Cmd help -Port COM7
```

## 5. 引脚映射

| 引脚 | 方向 | 功能 |
|---|---|---|
| PA9 / PA10 | AF | USART1：CLI 调试串口 @115200 |
| PA3 | AF | USART2_RX：433 接收模块 TX @9600 |
| PA2 | AF | USART2_TX：当前未接，预留 |
| PA0 | AIN | ADC1_IN0：红外接收头输出（HS0038 / VS1838B，空闲=高） |
| PA1 | OUT | 红外发射：软件翻转 38 kHz 载波（周期 26 µs，占空比 1/3） |
| PA5 | OUT | 射频发射：进发射模块 DATA，mark=高，3.3 V 直驱 |
| PA6 | AF | TIM3_CH1 PWM + DMA1_Stream4：两颗 WS2812B 串联数据线 |
| PC13 | OUT | 用户灯，低电平点亮（学习期间常亮） |
| PA13 / PA14 | AF | SWDIO / SWCLK |

定时器分配：TIM2 自由计数 @1 MHz 作为全局 µs 时基（录制与发射共用）；TIM3 800 kHz 供 WS2812B。

## 6. 目录结构

```
firmware/
├─ UserApp/           应用层
│  ├─ main.cpp        App 组装：实例化各模块并注入 Cli
│  ├─ cli.cpp/.hpp    命令循环、解析、分发、各命令实现
│  ├─ slot_store.hpp  槽位存储模板 SlotStore<Traits> + IR/RF 两种 Traits
│  ├─ slot_store.cpp  FlashDriver 原语与共享暂存缓冲
│  ├─ signal.hpp/.cpp 波形载体（2 KB / 510 段）
│  ├─ ir_receiver.*   红外录制状态机（ADC 边沿 + 段时长）
│  ├─ ir_transmitter.*红外回放（38 kHz 软件载波）
│  ├─ rf_receiver.*   射频串口帧解析（LC:hex）
│  ├─ rf_transmitter.*射频回放 + EV1527 编解码
│  └─ transmitter.hpp 发射器接口
├─ Bsp/               板级驱动
│  ├─ usart/          printf 风格串口输出
│  ├─ ws2812b/        PWM + DMA 驱动
│  └─ user_led/       PC13 用户灯 + 作用域守卫
├─ Core/              CubeMX 生成：时钟、GPIO、USART、ADC、TIM、中断
├─ Drivers/           CMSIS + STM32F4xx HAL
├─ MDK-ARM/           Keil 工程、启动文件、构建产物
├─ scripts/           build.sh / flash.sh / serial.ps1 / rftest.py
└─ HOPE-Remote.ioc    CubeMX 工程（改底层配置从这里重生成）
```

## 7. 使用

### 7.1 槽位模型

| 槽号区间 | 介质 | 每槽 | 数量 | 存储位置 |
|---|---|---|---|---|
| 000-095 | 红外波形 | 2 KB | 96 | 扇区 4 / 5 / 6 各前 64 KB |
| 096-099 | 保留 | — | — | 不可用，命令回 `ERR` |
| 100-611 | 射频码值 | 8 B | 512 | 扇区 7 前 4 KB |

槽号 2-4 位十进制，前导零可省（`xx00` = `xx000`）。两类槽共用一套命令，按槽号自动路由到对应存储。

存储层细节（槽头格式、覆盖写、掉电安全）见 [../docs/slot-store.md](../docs/slot-store.md)。

### 7.2 命令分组

| 分组 | 命令 | 用途 |
|---|---|---|
| 学习 | `xxNNN` | 把遥控器按键录进槽 |
| 回放 | `fsNNN`、`fss` / `fsl` / `fsa` / `fsb` | 发射槽内码，射频可调帧数与间隔 |
| 管理 | `slots`、`duNNN`、`clNNN`、`scNNN<hex>` | 列表 / 查看 / 清除 / 直接写码 |
| 灯与自检 | `ledRRGGBB`、`lednRRGGBB`、`uled0/1` | RGB 灯珠与用户灯 |
| 调试 | `rfmon`、`rfloop`、`rfscanNNN`、`rfscan2NNN`、`evtest`、`dbg`、`raw` | 串口监听、回环、编码变体扫描、ADC 统计、原始波形 |
| 帮助 | `help` | 板端命令列表 |

逐条语法、参数范围、返回值与示例见 [../docs/cli-reference.md](../docs/cli-reference.md)；射频调试命令的实测结论见 [../docs/rf-uart2-debug.md](../docs/rf-uart2-debug.md)。

### 7.3 典型流程

**红外学习 / 回放**

```
xx000        →  REC 0 start
（按遥控器按键）
             →  REC 0 OK (68 seg, 67432 us, 1123ms)
fs000        →  FS 0 OK
```

**射频学习 / 调发射参数**

```
xx103        →  REC 103 OK 0x62E7E8 pulse=320
fs103 1 1    →  单帧单簇（最小粒度，用于逐台定位）
fs103 15 1   →  15 帧单簇（长按类目标）
fs103        →  默认 8 帧 × 1 簇，帧间隔 5 ms，簇间隔 300 ms
```

**免学习直接写码**

```
sc100 62E7E8   →  SC 100 SET 0x62E7E8 pulse=320
```

## 8. 代码结构

```
        USART1 (CLI)  ─┐
        PA0   (IR RX) ─┤
        USART2 (RF RX)─┴─→  Cli  ──→  IrStore / RfStore  ──→  FlashDriver  ──→  Flash
                             │
                             └──→  IrTransmitter (PA1) / RfTransmitter (PA5)
                                   WS2812B (PA6) / UserLed (PC13)
```

| 文件 | 职责 |
|---|---|
| `main.cpp` | 组装 App：持有全部对象，注入 `Cli`；编译期断言 `Signal` 与 IR 槽容量一致 |
| `cli.cpp` | 环形缓冲收命令行 → 归一化 → 按前缀分发 → 调存储 / 收发器；含全部调试命令 |
| `slot_store.hpp` | `SlotStore<Traits>` 模板：地址映射、整槽擦除检测、兄弟槽暂存、覆盖写、校验 |
| `signal.hpp` | 段数组（bit31 电平，bit30:0 µs），上限 510 段 |
| `ir_receiver.*` | 非阻塞状态机：ADC 相邻采样差值判边沿，空闲 100 ms 判结束，15 s 总超时 |
| `rf_receiver.*` | USART2 中断 → 环形缓冲 → 按行解析 `LC:hex`（6 或 8 位） |
| `rf_transmitter.*` | `ev1527Encode` / `ev1527Decode` / `encodeRfVariant`，GPIO 按段时长翻转 |
| `ws2812b.*` | TIM3 PWM + DMA，GRB 24 bit/颗，帧尾 50 拍复位 |

## 9. 已知限制

- **红外容量**：单槽 ≤ 510 段 / 2 KB，超长码截断并回 `BufferFull`。
- **射频协议**：仅 EV1527 24 bit 定长码，滚动码无法学习。
- **回放阻塞**：`fsNNN` 与学习命令均为忙等，期间命令不响应。
- **串口模块偏好**：灵-R1A 只认长载波 sync，`rfloop` 的 `no rx` 不代表发射失败，正确性以目标设备实测为准。
- **发射参数不落盘**：槽内只存 `code24` + `pulseUs`，帧数 / 簇数 / 间隔每次由命令给出。

## 10. 相关文档

| 文档 | 内容 |
|---|---|
| [../README.md](../README.md) | 项目总览与快速开始 |
| [../docs/cli-reference.md](../docs/cli-reference.md) | CLI 完整参考 |
| [../docs/slot-store.md](../docs/slot-store.md) | 槽位存储层与 Flash 原理 |
| [../docs/rf-uart2-debug.md](../docs/rf-uart2-debug.md) | 射频模块调试与回放时序 |
| [../hardware/README.md](../hardware/README.md) | 板级规格与设计文件 |
