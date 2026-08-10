# HOPE-Remote

基于 STM32F401RE 的遥控码学习/复现设备（红外 IR + 433MHz 射频 RF）。

## 功能

- IR 遥控码学习与回放，槽 0~95 × 2KB Flash 存储
- 433MHz RF 遥控码学习与回放，槽 100~131 × 2KB Flash 存储（扇区7 前 64KB）
- 串口 CLI 调试（USART1 @ 115200）
- 原始波形捕获、ADC 采样诊断

## 槽位布局

| 槽号 | 介质 | 物理扇区 |
|---|---|---|
| 0~95 | 红外 | 扇区4/5/6 各前 64KB（每扇区 32 槽） |
| 96~99 | 无效（保留） | — |
| 100~131 | 射频 | 扇区7 前 64KB（32 槽） |

## 硬件引脚

| 引脚 | 功能 |
|---|---|
| PA9 | 串口调试 TX（USART1_TX） |
| PA10 | 串口调试 RX（USART1_RX） |
| PA0 | IR 接收（ADC1_IN0，HS0038 输出） |
| PA1 | IR 发射（38kHz 载波软件翻转） |
| PA4 | RF 接收（ADC1_IN4；RXB6 DATA 须经 10kΩ 串 + 20kΩ 对地分压，模拟模式不 5V 耐受） |
| PA2 | RF 发射（TX470 DATA，直驱基带电平） |

## CLI 命令

| 命令 | 说明 |
|---|---|
| `xxNNN` | 学习遥控码到槽（000-095 IR，100-131 RF） |
| `fsNNN` | 播放槽 |
| `slots` | 列出槽占用情况 |
| `duNNN` | 打印槽段时长（调试） |
| `dbg` | ADC 采样 1s：min/max/avg/edges（调试） |
| `raw` | 捕获 200ms 原始信号并打印（调试） |
| `help` | 显示帮助 |

> **槽号格式说明**：`NNN` 支持 **2 位或 3 位**，两者等价，无需强制补前导零。
> - IR 槽 0~95：`xx00` 或 `xx000` 均有效（2 位即可）
> - RF 槽 100~131：槽号本身是 3 位，必须写 `xx100`-`xx131`
> - 槽号 96~99 为保留无效区，任何写法都会返回 `ERR`；1 位槽号（如 `xx0`）同样返回 `ERR`

## 目录结构

- `UserApp/` — 应用逻辑（signal/storage/receiver、Transmitter 基类 + IR/RF 子类、CLI）
- `Bsp/` — 板级驱动
- `Core/` — CubeMX 生成代码（引脚初始化由 CubeMX 配置生成）
- `Drivers/` — HAL 驱动
- `MDK-ARM/` — Keil 工程

## 构建

用 Keil MDK 打开 `MDK-ARM/HOPE-Remote.uvprojx`，编译后下载。
