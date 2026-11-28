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

## 校验下载完整性

```bash
sha256sum HOPE-Remote-firmware.hex
# 9b8dc91a9b7b982e5512689e63d1e05cff01386b0b8f9672894f44a08ef303cf  HOPE-Remote-firmware.hex
```

Windows PowerShell：

```powershell
(Get-FileHash .\HOPE-Remote-firmware.hex -Algorithm SHA256).Hash.ToLower()
```

> 仓库已通过 `.gitattributes` 将 `*.hex` 标记为二进制（`-text`），git 不会改写换行符，因此下载到的字节与上表摘要一致。

## 与源码构建的关系

本文件即 `firmware/MDK-ARM/HOPE-Remote/HOPE-Remote.hex`（Keil 构建产物，本地生成、不入库），只是重命名以便下载识别。

如需自己从源码构建：

```bash
cd ../firmware
./scripts/build.sh          # 或直接用 Keil 打开 MDK-ARM/HOPE-Remote.uvprojx
```

产物会生成在 `firmware/MDK-ARM/HOPE-Remote/HOPE-Remote.hex`。

## 版本说明

旧产物（2026-10-06 构建）链接基址在 `0x0800_4000`，那是早期调试时临时改的配置，仅为测试用，并非最终方案。`0x0800_4000` 意味着跳过扇区 0-3（64 KB）从第二个扇区启动，空片烧录会因 `0x0800_0000` 处没有向量表而不启动。

当前工程 IROM 基址已改回 `0x0800_0000`，本文件即该配置下的产物。这一版是从当前源码重新编译的，不是历史文件的拷贝。

旧的 `0x0800_4000` 版本已废弃，不要使用。
