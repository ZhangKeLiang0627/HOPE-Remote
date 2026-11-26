# HOPE-Remote Firmware

预编译固件，直接烧录即可使用，无需安装 Keil。

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

## 烧录

### STM32CubeProgrammer（图形界面）

1. 板子接 USB，选择 ST-LINK / DAPLink 后点击 Connect
2. `Erasing & Programming` → 选 `HOPE-Remote-firmware.hex`
3. 勾选 `Verify programming`，Start Programming

### STM32CubeProgrammer（命令行）

```bash
STM32_Programmer_CLI -c port=SWD -w HOPE-Remote-firmware.hex -v -rst
```

### OpenOCD（DAPLink / CMSIS-DAP）

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 4000" \
  -c "program HOPE-Remote-firmware.hex verify reset exit"
```

### Keil MDK

Keil 不能直接下载裸 hex，必须打开工程（`firmware/MDK-ARM/HOPE-Remote.uvprojx`）后 `F8`。既然已经装了 Keil，建议直接从源码编译，见下节。

## 烧录后验证

串口 115200 8N1（板载 CH340N，接 Type-C 即出现串口），上电应看到：

```
HOPE-Remote boot
HOPE-Remote ready (help for commands)
```

发 `help` 有命令列表返回即固件正常运行。完整自检流程见 [../hardware/README.md](../hardware/README.md)。

## 与源码构建的关系

本文件与 `firmware/MDK-ARM/HOPE-Remote/HOPE-Remote.hex` 完全相同（同一份产物），只是重命名以便下载识别。

如需自己从源码构建：

```bash
cd ../firmware
./scripts/build.sh          # 或直接用 Keil 打开 MDK-ARM/HOPE-Remote.uvprojx
```

产物会生成在 `firmware/MDK-ARM/HOPE-Remote/HOPE-Remote.hex`。

## 版本说明

**`0x0800_4000` 起始的旧产物已废弃。** 早前工程把 IROM 基址配在 `0x0800_4000`（为 64 KB bootloader 预留扇区 0-3），但仓库里没有对应 bootloader；且 `system_stm32f4xx.c` 未定义 `USER_VECT_TAB_ADDRESS`，`SCB->VTOR` 保持复位值 `0x0000_0000`（BOOT0=0 时别名到 `0x0800_0000`）。两者叠加的结果是：向量表落在 `0x0800_4000`，CPU 却去 `0x0800_0000` 取复位向量，空片烧进去直接 HardFault，串口没有任何输出。

当前工程 IROM 基址已改回 `0x0800_0000`，与本文件一致。这一版是从当前源码重新编译的产物，不是历史文件的拷贝。

历史旧 hex 仅作对照，**不要使用**。
