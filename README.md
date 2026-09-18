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
| PA6 | RGB 灯珠 ×2（WS2812B 串联，TIM3_CH1 PWM + DMA1_Stream4） |
| PC13 | 用户指示灯 LED（低电平点亮，学习中常亮） |

上电默认灯色：**灯0 奶油粉 `#FFB7C5`，灯1 奶油蓝 `#87CEEB`**（灯0 靠 MCU 侧，两颗互换只需对调 `main.cpp` 里 `bootColor[]` 的顺序）。

## CLI 命令

| 命令 | 说明 |
|---|---|
| `xxNNN` | 学习遥控码到槽（000-095 IR，100-611 RF） |
| `fsNNN` | 播放槽（RF 支持发射参数，见下） |
| `slots` | 列出槽占用情况 |
| `duNNN` | 打印槽内容（IR 段时长 / RF 码值） |
| `clNNN` | 清除槽数据 |
| `scNNN` | RF 槽直接编程 EV1527 码：`scNNN<hex6\|hex8>` |
| `ledRRGGBB` | 设置 RGB 灯珠颜色（两颗同色，每分量 2 位 hex，如 `led00FF00` 绿） |
| `lednRRGGBB` | 只设置第 n 颗灯珠（n=0/1，如 `led1FF0000` 只点灯1 红），用于单独定位 |
| `uled0` / `uled1` | 手动熄灭 / 点亮 PC13 用户指示灯（硬件自检） |
| `help` | 显示帮助 |

调试命令（`rfmon`/`rfloop`/`rfscan`/`rfscan2`/`evtest`/`dbg`/`raw`）见 [docs/rf-uart2-debug.md](docs/rf-uart2-debug.md)。

> **槽号格式**：`NNN` 支持 2-3 位。IR 槽 0-95；RF 槽 100-611（3 位）。96-99 为保留无效区。

> **RF 学习**：按住原遥控按键，连续 2 帧相同码即确认保存（按下瞬间完成），15s 超时。
> 码值取 hex 前 24 位（RCSwitch 约定一致）。

> **RF 回放**：RCSwitch 标准时序（数据先行 24bit MSB + sync 收尾 1p+31p，pulse 320μs）。
> 默认发射参数 **8 帧 × 1 簇 × 帧间隔 5ms × 簇间隔 300ms**，
> 可用 `fsNNN [帧数 簇数 帧间隔ms 簇间隔ms]` 覆盖（如 `fs103 1 1` 单帧、`fs103 8 1 5 300` 显式指定）。

## 目录结构

- `UserApp/` — 应用逻辑（signal、slot_store 模板 + Traits、receiver、Transmitter 基类 + IR/RF 子类、CLI）
- `Bsp/` — 板级驱动（usart 调试口、ws2812b 灯珠、user_led 用户指示灯）
- `Core/` — CubeMX 生成代码（引脚初始化由 CubeMX 配置生成）
- `Drivers/` — HAL 驱动
- `MDK-ARM/` — Keil 工程
- `docs/` — 调试与测试手册

## 用户指示灯（PC13）

电路为 **3.3V → 2kΩ → LED → PC13**，故 PC13 **拉低=亮、拉高=灭**，上电初始化即拉高（灭）。

| 状态 | 灯 |
|---|---|
| 空闲 | 灭 |
| `xxNNN` 学习中（IR 或 RF） | 常亮 |
| 学习结束（OK / TIMEOUT / EMPTY / FLASHERR） | 灭 |

超时不会提前灭灯——学习会话持续到 15s 超时为止，超时那一刻才熄灭。

实现见 `Bsp/user_led/`。`Cli::onLearn()` 入口挂一个作用域守卫 `UserLed::Guard`，
构造即点亮、析构必熄灭，因此 IR 与 RF 两条路径的全部 return 出口
（含 RF 委托的 `onLearnRf`）都自动覆盖，不需要逐个补 `off()`。
其余命令（`fs`/`du`/`raw`/`rfmon`/`dbg`/`rfloop`/`rfscan` 等）不点灯。

> PC13 属备份域引脚，数据手册限定输出速度 ≤2MHz、灌电流 ≤3mA，
> CubeMX 已配 `GPIO_SPEED_FREQ_LOW`，勿改成 `HIGH`。
> 2kΩ 限流下电流 ≈0.65mA（按 Vf≈2.0V 估算），偏暗属正常；要更亮可换 1kΩ（≈1.3mA）。

## RGB 灯珠（WS2812B）

`Bsp/ws2812b/` 驱动移植自 HOPE-Link 工程（原 TIM5_CH3 @PA2），本项目改用 TIM3_CH1 @PA6。
TIM3 与 TIM5 同挂 APB1、定时器时钟同为 84MHz，故时序参数不变：

| 项 | 值 | 说明 |
|---|---|---|
| Prescaler / Period | 1-1 / 105-1 | 84MHz / 105 = 800kHz PWM 载波 |
| T0H / T1H | 35 / 70 拍 | ≈417ns / ≈833ns（规格 400ns / 800ns ±150ns） |
| 一帧长度 | NUM×24 + 50 拍 | 尾部 50 拍低电平 ≈62µs 复位间隔（经典版规格）；NUM=2 时一帧 ≈123µs |
| 灯珠数 | `WS2812B_NUM = 2` | 两颗串联；改此宏即可扩展，须同步确认长度 |
| 复位间隔 | `WS2812B_RESET_TICKS = 50` | 若遇到颜色/级联错位且怀疑复位不足，改成 300（≈375µs）兼容新版规格 |
| 灯珠供电 | 建议 4.3-4.7V | 额定 3.5-5.3V；低于 4.2V 蓝/绿亮度不足，高于 4.7V 则 VIH 超出 3.3V 逻辑能力 |

用法（`SetPixels` 传的是 24bit 值，且线上顺序为 GRB）：

```cpp
uint32_t colors[WS2812B_NUM] = {
    WS2812B::Color(0xFF, 0xB7, 0xC5),   // 灯0 奶油粉
    WS2812B::Color(0x87, 0xCE, 0xEB),   // 灯1 奶油蓝
};
led_.SetPixels(WS2812B_NUM, colors);    // Color() 负责打包成 G<<16 | R<<8 | B
led_.UpdatePixels();
```

DMA 用 `DMA1_Stream4 / Channel 5`（TIM3_CH1/TRIG 的固定映射），Normal 模式、字对齐。
`UpdatePixels()` 内部先 `HAL_TIM_PWM_Stop_DMA` 再启动，以支持反复刷新
（HAL 的 PWM+DMA 是一次性启动，发完后通道状态停在 BUSY，重复启动会被判 HAL_BUSY 静默丢弃）。

## 构建

用 Keil MDK 打开 `MDK-ARM/HOPE-Remote.uvprojx`，编译后下载。

命令行批量编译：`UV4 -b MDK-ARM/HOPE-Remote.uvprojx -j0`。

> 外设改动请走 STM32CubeMX：打开 `HOPE-Remote.ioc` 配置后 Generate Code，
> 不要把自定义代码写进 `Core/` 的生成区（USER CODE 段除外）。新增的 `Bsp/` 源文件
> 需手工加进 Keil 工程（CubeMX 只管理它自己生成的文件）。
