# HOPE-Remote

基于 STM32F401RE 的遥控码学习/复现设备（红外 IR + 433MHz 射频 RF）。

## 功能

- IR 遥控码学习与回放，槽 0-95 × 2KB Flash 存储（波形）
- 433MHz RF 遥控码学习与回放，槽 100-611 × 8B Flash 存储（码值）
- RF 学习走串口解码模块（灵-R1A 串口版 → USART2 @ 9600，输出 `LC:xxxxxxxx`）
- RF 回放走 PA5 直驱发射模块（远-T2L，DATA 基带电平）
- 串口 CLI 调试（USART1 @ 115200）

## 槽位布局

| 槽号 | 介质 | 存储格式 | 物理位置 |
|---|---|---|---|
| 0-95 | 红外 | 2KB 波形槽（510 段） | 扇区4/5/6 各前 64KB（每扇区 32 槽） |
| 96-99 | 无效（保留） | — | — |
| 100-611 | 射频 | 8B 码值槽（code24 + pulse） | 扇区7 前 4KB（512 槽） |

## 硬件引脚

| 引脚 | 功能 |
|---|---|
| PA9 | 串口调试 TX（USART1_TX） |
| PA10 | 串口调试 RX（USART1_RX） |
| PA3 | RF 串口接收（USART2_RX，灵-R1A 串口版 TX，9600） |
| PA2 | USART2_TX（当前未用，预留） |
| PA0 | IR 接收（ADC1_IN0，HS0038 输出） |
| PA1 | IR 发射（38kHz 载波软件翻转） |
| PA5 | RF 发射（远-T2L DATA，直驱基带电平，mark=高） |

## CLI 命令

| 命令 | 说明 |
|---|---|
| `xxNNN` | 学习遥控码到槽（000-095 IR，100-611 RF） |
| `fsNNN` | 播放槽（RF 支持发射参数，见下） |
| `slots` | 列出槽占用情况 |
| `duNNN` | 打印槽内容（IR 段时长 / RF 码值） |
| `clNNN` | 清除槽数据 |
| `scNNN` | RF 槽直接编程 EV1527 码：`scNNN<hex6\|hex8>` |
| `help` | 显示帮助 |

调试命令（`rfmon`/`rfloop`/`rfscan`/`rfscan2`/`evtest`/`dbg`/`raw`）见 [docs/rf-uart2-debug.md](docs/rf-uart2-debug.md)。

> **槽号格式**：`NNN` 支持 2-3 位。IR 槽 0-95；RF 槽 100-611（3 位）。96-99 为保留无效区。

> **RF 学习**：按住原遥控按键，连续 2 帧相同码即确认保存（按下瞬间完成），15s 超时。
> 码值取 hex 前 24 位（与 433_test_arduino/RCSwitch 约定一致）。

> **RF 回放**：RCSwitch 标准时序（数据先行 24bit MSB + sync 收尾 1p+31p，pulse 320μs）。
> 默认发射参数 **8 帧 × 1 簇 × 帧间隔 5ms × 簇间隔 300ms**（多盏灯实测通杀），
> 可用 `fsNNN [帧数 簇数 帧间隔ms 簇间隔ms]` 覆盖（如 `fs103 1 1` 单帧、`fs103 8 1 5 300` 显式指定）。

## 目录结构

- `UserApp/` — 应用逻辑（signal、slot_store 模板 + Traits、receiver、Transmitter 基类 + IR/RF 子类、CLI）
- `Bsp/` — 板级驱动
- `Core/` — CubeMX 生成代码（引脚初始化由 CubeMX 配置生成）
- `Drivers/` — HAL 驱动
- `MDK-ARM/` — Keil 工程
- `docs/` — 调试与测试手册

## 构建

用 Keil MDK 打开 `MDK-ARM/HOPE-Remote.uvprojx`，编译后下载。
