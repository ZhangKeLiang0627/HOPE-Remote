# HOPE-Remote Firmware

预编译固件，直接烧录即可使用。

| 项 | 值 |
|---|---|
| 文件 | `HOPE-Remote-firmware.hex`（Intel HEX） |
| 目标芯片 | STM32F401RE（512 KB Flash） |
| 固件大小 | 43,496 字节（42.5 KB），占用 `0x0800_0000` - `0x0800_A9E7` |
| 链接基址 | `0x0800_0000`（从 Flash 起始处执行，无需 bootloader） |
| 编译工具 | Keil MDK 5.35 / ARM Compiler 6.16（AC6） |
| 编译日期 | 2026-11-26 |
| 源码版本 | `c60f817` |
| SHA-256 | `9b8dc91a9b7b982e5512689e63d1e05cff01386b0b8f9672894f44a08ef303cf` |

> [!IMPORTANT]
> 烧录前确认芯片 Flash ≥ 512 KB（`STM32F401RE` / `STM32F401CE`）。`STM32F401RC` 只有 256 KB，槽位区（扇区 6 / 7）会越界，红外与射频存储不可用。

## 烧录后验证

串口 115200 8N1（板载 CH340N，接 Type-C 即出现串口），上电应看到：

```
HOPE-Remote boot
HOPE-Remote ready (help for commands)
```

发 `help` 有命令列表返回即固件正常运行。完整自检流程见 [../hardware/README.md](../hardware/README.md)。

