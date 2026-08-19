# HOPE-Remote

基于 STM32F401RE 的遥控码学习/复现设备（红外 IR + 433MHz 射频 RF）。

## 功能

- IR 遥控码学习与回放，槽 0~95 × 2KB Flash 存储（波形）
- 433MHz RF 遥控码学习与回放，槽 100~611 × 8B Flash 存储（码值）
- RF 学习走串口解码模块：灵-R1A-M5（串口版）→ USART2 @ 9600，输出 `LC:xxxxxxxx` 帧
- RF 回放走 PA5 直驱发射模块（远-T2L 类，DATA 基带电平）
- 串口 CLI 调试（USART1 @ 115200）
- `rfloop` 空气回环自测（发射已知码 → 串口回收比对）

## 槽位布局

| 槽号 | 介质 | 存储格式 | 物理位置 |
|---|---|---|---|
| 0~95 | 红外 | 2KB 波形槽（510 段） | 扇区4/5/6 各前 64KB（每扇区 32 槽） |
| 96~99 | 无效（保留） | — | — |
| 100~611 | 射频 | 8B 码值槽（code24 + pulse） | 扇区7 前 4KB（512 槽） |

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
| `fsNNN` | 播放槽 |
| `slots` | 列出槽占用情况 |
| `duNNN` | 打印槽内容（IR 段时长 / RF 码值） |
| `clNNN` | 清除槽数据 |
| `scNNN` | RF 槽直接编程 EV1527 码：`scNNN<hex6\|hex8>` |
| `dbg` | IR ADC 采样 1s：min/max/avg/edges（调试） |
| `raw` | 捕获 3000ms IR 原始波形并打印（调试） |
| `rfmon` | 监听 USART2 RF 码流，x 退出（调试） |
| `rfloop` | RF 空气回环自测（发射位移补偿码 → 模块回收比对） |
| `evtest` | EV1527 编解码往返自测 |
| `help` | 显示帮助 |

> **槽号格式**：`NNN` 支持 2~3 位。IR 槽 0~95（`xx00`/`xx000` 均可）；RF 槽 100~611（3 位）。
> 槽号 96~99 为保留无效区，1 位槽号同样返回 `ERR`。

> **RF 学习**：`xxNNN`（RF 槽）→ 按住原遥控按键，连续收到 3 帧相同码（`LC:` 前缀）
> 即通过去抖，帧间 500ms 无新帧自动保存，15s 无有效帧超时。
> 码值取 hex 前 24 位（hex8 约定与 433_test_arduino/RCSwitch 一致）。

> **RF 回放（sync 收尾时序）**：实测确定正确帧结构 = **数据位先行 + sync 收尾**
> （RCSwitch 风格）：`[24bit 数据 MSB first][sync: 31×pulse 载波 + 1×pulse 空闲]`。
> - sync 在**帧尾**（参考项目 ESP433RF 实际时序），且用长载波比例（1p+31p sync 模块收不到）
> - 回放用**原码**（无需位移补偿——位移补偿是 sync 前置时序下的错误推导）
> - bit：0 = 1p 载波 + 3p 空闲；1 = 3p 载波 + 1p 空闲；pulse 320μs；8 帧重复

## RF 回环自测（rfloop）

无需遥控器，全自动验证 RF 全链路与波形正确性：

```
rfloop → PA5 驱动远-T2L 发 0x55C311（sync 收尾时序，8 帧）
      → 灵-R1A 串口版空中收到 → USART2 回传 LC: 帧
      → MCU 解析比对 → got == 0x55C311 即 [TARGET MATCH] PASS
```

发射与接收模块建议拉开 1~3m（近距可能饱和收不到）。

## 目录结构

- `UserApp/` — 应用逻辑（signal、slot_store 模板 + Traits、receiver、Transmitter 基类 + IR/RF 子类、CLI）
- `Bsp/` — 板级驱动
- `Core/` — CubeMX 生成代码（引脚初始化由 CubeMX 配置生成）
- `Drivers/` — HAL 驱动
- `MDK-ARM/` — Keil 工程

## 构建

用 Keil MDK 打开 `MDK-ARM/HOPE-Remote.uvprojx`，编译后下载。
