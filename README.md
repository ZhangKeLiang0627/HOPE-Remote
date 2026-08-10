# HOPE-Remote

基于 STM32F401RE 的红外遥控码学习/复现设备（由 STM32F405 工程移植）。

## 功能

- IR 遥控码学习与回放，96 槽 × 2KB Flash 存储
- 串口 CLI 调试（USART1 @ 115200）
- 原始波形捕获、ADC 采样诊断

## CLI 命令

| 命令 | 说明 |
|---|---|
| `xxNN` | 学习遥控码到槽 NN（00-95） |
| `fsNN` | 播放槽 NN |
| `slots` | 列出槽占用情况 |
| `duNN` | 打印槽 NN 段时长（调试） |
| `dbg` | ADC 采样 1s：min/max/avg/edges（调试） |
| `raw` | 捕获 200ms 原始信号并打印（调试） |
| `help` | 显示帮助 |

## 目录结构

- `UserApp/` — 应用逻辑（IR、存储、CLI）
- `Bsp/` — 板级驱动
- `Core/` — CubeMX 生成代码
- `Drivers/` — HAL 驱动
- `MDK-ARM/` — Keil 工程

## 构建

用 Keil MDK 打开 `MDK-ARM/HOPE-Remote.uvprojx`，编译后下载。
