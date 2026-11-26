# CLI 命令参考

HOPE-Remote 的全部功能都通过 USART1 命令行提供。本文是逐条参考；快速上手见 [根目录 README](../README.md)，固件结构见 [firmware/README.md](../firmware/README.md)。

## 1. 通用约定

| 项 | 说明 |
|---|---|
| 串口 | USART1，115200 8N1（板载 CH340N 转 Type-C） |
| 行结尾 | `\r` 或 `\n`，两者等价；空行忽略 |
| 大小写 | 不敏感，板端统一转小写后再匹配 |
| 行长上限 | 63 字符，超出部分丢弃 |
| 槽号 | 2-4 位十进制，前导零可省（`xx00` = `xx000`） |
| 红外槽 | `000`-`095`（96 个） |
| 射频槽 | `100`-`611`（512 个） |
| 保留槽 | `096`-`099`，两类存储都不接受，命令回 `ERR` |

应答均为一行文本，以 `\r\n` 结束。常见状态字：

| 状态字 | 含义 |
|---|---|
| `OK` | 成功 |
| `ERR` | 参数或槽号非法，命令未执行 |
| `EMPTY` | 槽内没有有效数据 |
| `TIMEOUT` | 学习超时（15 s 内没收到可用信号） |
| `FLASHERR` | Flash 写入 / 擦除失败 |

> 所有学习与回放命令都是**阻塞**执行：期间不接受新命令，但 USART1/USART2 中断照常把字节收进缓冲区。

## 2. 学习

### `xxNNN`

把遥控器的一个按键录进槽 NNN。槽号决定走哪条路径，行为完全不同：

| 槽号 | 采集方式 | 结束条件 |
|---|---|---|
| `000`-`095` | 红外接收头（PA0），ADC 采样判边沿 | 首个边沿开始录，空闲 100 ms 判结束；总超时 15 s |
| `100`-`611` | 433 串口模块（USART2），取 `LC:hex` 帧 | 连续 2 帧码值相同即保存；总超时 15 s |

```
xx000
REC 0 start
REC 0 OK (68 seg, 67432 us, 1123ms)
```

```
xx103
REC 103 start
hold remote button until REC OK
REC 103 OK 0x62E7E8 pulse=320
```

返回字段：

- `seg`：录到的段数（载波段 + 空闲段），上限 510
- `us`：所有段时长之和
- `ms`：本次学习耗时
- `0x……`：24 bit EV1527 码值；`pulse`：脉宽，固定 320 µs

> 学习期间 PC13 用户灯常亮，任一出口（OK / TIMEOUT / EMPTY / FLASHERR）立即熄灭——灯亮即表示正在等信号。

**红外注意**：整段录制不做协议解码，按住遥控器只发一帧即可；若 `EMPTY`，先用 `dbg` 确认接收头有边沿。

**射频注意**：模块只在按键按下瞬间输出帧，需在原遥控器上按一下（长按 1 s 内即可完成）。

## 3. 回放

### `fsNNN [f b g bg]`

发射槽 NNN 的内容。红外忽略参数（单帧单发）；射频支持 4 个可选参数，按空格分隔、按顺序生效，缺省用默认值。

| 参数 | 含义 | 范围 | 默认 |
|---|---|---|---|
| `f` | 每簇帧数 | 1-30 | 8 |
| `b` | 簇数 | 1-10 | 1 |
| `g` | 帧间隔（ms） | 0-1000 | 5 |
| `bg` | 簇间隔（ms） | 0-2000 | 300 |

超出范围会被静默截断到上限。

```
fs000              →  FS 0 OK
fs103              →  FS 103 OK (v0 1x8 g5/300)
fs103 1 1          →  FS 103 OK (v0 1x1 g5/300)
fs103 15 1 5 300   →  FS 103 OK (v0 1x15 g5/300)
```

返回里的 `1x8` 是「簇数 × 帧数」，`g5/300` 是「帧间隔 / 簇间隔」。

> 帧间隔与簇数不是「越大越稳」。实测三盏灯的最优参数各不相同，5 ms 紧凑间隔反而更容易被接收端保持同步；簇数为给低功耗接收端唤醒用，目标设备需要时才加。逐台调参见 [rf-uart2-debug.md](rf-uart2-debug.md#3-发射参数试验指南)。

### `fssNNN` / `fslNNN` / `fsaNNN` / `fsbNNN`

回放预设，仅射频槽有效（槽号必须 3 位，命令总长 6 字符）：

| 命令 | 实际行为 | 适用 |
|---|---|---|
| `fssNNN` | 1 簇 × 3 帧，间隔 5/300 ms | 短按类目标（多数灯具 / 插座） |
| `fslNNN` | 1 簇 × 15 帧，间隔 5/300 ms | 长按类目标（需要持续接收） |
| `fsaNNN` | 1 簇 × 8 帧，标准 sync | 等价于 `fsNNN` 默认参数 |
| `fsbNNN` | 1 簇 × 8 帧，**长载波 sync** | 仅调试：让串口模块能回收，目标设备不认 |

> 板端 `help` 里 `fss` / `fsl` 写的「bursts」是早期多簇版本的遗留描述，实际为单簇连发，以上表为准。

## 4. 槽位管理

### `slots`

遍历全部槽并打印已占用项，最后给出总数（红外 96 + 射频 512 = 608）。

```
slots
  000: OK (68 seg)
  103: OK (code 0x62E7E8)
total: 2/608 used
```

### `duNNN`

查看槽内容。红外打印带符号的段序列：正数 = 载波段（mark），负数 = 空闲段（space），单位 µs；射频打印码值与脉宽。

```
du000
DUMP 0 68 seg:
500,-1500,500,-500,...,len=68
```

```
du103
DUMP 103 code=0x62E7E8 pulse=320
```

### `clNNN`

清除槽数据。实现是「擦整扇区 → 回写同扇区其它有效槽」，空槽直接返回成功。

```
cl103  →  CL 103 OK
```

### `scNNN<hex6|hex8>`

免学习，直接把已知 EV1527 码写进射频槽（仅 `100`-`611`）。支持 6 位或 8 位 hex，中间可带一个空格。

```
sc100 62E7E8    →  SC 100 SET 0x62E7E8 pulse=320
sc100 62E7E8AB  →  SC 100 SET 0xE7E8AB pulse=320
```

> 8 位 hex 视作「地址码 6 位 + 按键码 2 位」，实际只取前 24 位（`>> 8`），与 RCSwitch / 433_test_arduino 的约定一致。脉宽固定 320 µs。

## 5. 灯

| 命令 | 作用 | 示例返回 |
|---|---|---|
| `ledRRGGBB` | 两颗灯珠同色，每分量 2 位 hex | `LED 00FF00` |
| `lednRRGGBB` | 只改第 n 颗（n=0/1），其余保持 | `LED1 FF0000` |
| `uled1` / `uled0` | PC13 用户灯亮 / 灭（硬件自检） | `ULED on` / `ULED off` |

上电默认：灯 0 `RGB(2,0,2)` 微亮粉紫，灯 1 `RGB(0,2,2)` 微亮青（亮度值只有 2/255，夜里不刺眼）。改默认色改 `main.cpp` 的 `bootColor[]`。

```
led00FF00     →  两颗绿
led1FF0000    →  只第 1 颗红（定位单颗用）
led000000     →  全灭
```

## 6. 调试与自测

这些命令面向排障与二次开发，日常使用不需要。

### `evtest`

EV1527 编解码往返自测（纯软件，不依赖硬件）。4 组码值编码再解码并比对：

```
EVTEST EV1527 codec self-test
  #0 OK  0x62E7E8 (enc pulse 350 -> dec pulse 350)
  ...
EVTEST PASS
```

### `dbg`

对红外接收头做 1 s ADC 采样统计，判断接收头是否有信号、是否饱和：

```
DBG ch=0(IR/PA0): press IR remote button within 1s...
DBG min=180 max=3980 avg=3560 edges=14
```

`edges` 为超过阈值的跳变次数，按遥控器时明显增加；`max` 一直贴着 4095 说明饱和，`min`/`max` 几乎不变说明接收头没输出。

### `raw`

抓 3000 ms 原始波形并打印段序列，用于看协议结构（格式同 `duNNN`）：

```
RAW ch=0(IR/PA0): capturing 3000ms, press & HOLD IR remote...
RAW 132 seg:
-9000,4500,-560,...
```

### `rfmon`

监听 USART2 码流（串口模块原始输出），逐行打印，输入 `x` + 回车退出：

```
RFMON listening UART2 (RF module), type 'x' + Enter to stop
RF: LC:62E7E8
RFMON stopped
```

用来确认模块是否收到信号、输出格式是否被解析。

### `rfloop`

空气回环自测：PA5 发射 8 帧 `0x55C311`（标准 EV1527 时序），再用串口模块回收比对。

| 返回 | 含义 |
|---|---|
| `RFLOOP PASS 0x55C311 [TARGET MATCH]` | 发射链路通 |
| `RFLOOP MISMATCH got 0x……` | 收到但码值不符 |
| `RFLOOP no rx (normal: module rejects standard sync; verify via target device)` | 模块不认标准 sync，**属正常** |

> 该模块只认长载波 sync（31p+1p），标准 EV1527 是短载波 sync（1p+31p），所以 `no rx` 不代表发射失败。要验证「目标设备认不认」，只能用 `rfscan` 拿真设备试。

### `rfscanNNN`

以槽 NNN 的码值为基础，依次发射 15 种编码变体（sync 位置 / sync 比例 / 位序 / pulse 覆盖），每个变体发 8 帧、监听 1.5 s：

```
RFSCAN code=0x62E7E8 pulse=320, 15 variants - watch target device
#1 FIRST sync31p+1p orig  -> rx 0x62E7E8 [TARGET MATCH]
#11 LAST  sync31p+1p orig  -> no rx
...
RFSCAN done - which # toggled target? module rx above
```

用途：**盯着目标设备**（灯 / 插座）看哪个 `#` 号有反应，那个变体就是正确编码。模块回显只作辅助。

### `rfscan2NNN`

sync-前置比例精扫：mark 28-40p × space 1-3p 共 21 种，每个发 6 帧、监听 1 s，靠模块回收判 `[TARGET MATCH]`：

```
RFSCAN2 sync-FIRST scan code=0x62E7E8 pulse=320, 21 variants
#1 sync31p+1p -> rx 0x62E7E8 [TARGET MATCH]
...
RFSCAN2 done - report MATCH # to me
```

> 模块对连续发射免疫较差，结果不如 `rfscanNNN` 可靠；用于确认 sync 比例，不用于验收。

### `help`

打印板端命令列表。内容与本文可能有细微出入（`fss` / `fsl` 的描述即为一例），以本文为准。

## 7. 典型场景

**学一个红外空调遥控键**

空调码较长，先确认没超容量：学完看 `seg` 数，接近 510 就可能被截断。

```
xx010
（对着板子按一下"开关"）
REC 10 OK (286 seg, 268340 us, 1188ms)
fs010
FS 10 OK
```

**给新买的射频灯配码**

不知道码值，用学习：

```
xx120
REC 120 OK 0x7A31C5 pulse=320
fs120 1 1      ← 先试单帧，灯无反应再加大
fs120 3 1
```

**批量灌码（同型号多台设备）**

一台学到正确码后，直接写进其它槽，不用重复学习：

```
sc300 7A31C5
sc301 7A31C5
```

**怀疑硬件没通**

```
uled1            ← 用户灯亮？
led00FF00        ← 灯珠绿？
dbg              ← 红外接收头有边沿？
evtest           ← 编解码逻辑正常？
```
