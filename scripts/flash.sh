#!/bin/bash
# OpenOCD + CMSIS-DAP 烧录 HEX（路径转 Windows 格式）
cd "$(dirname "$0")/.." || exit 1
HEX="$(cygpath -m "$(pwd)/MDK-ARM/HOPE-Remote/HOPE-Remote.hex")"
cd "/d/Application/Edge Download/OpenOCD-20240916-0.12.0/bin" || exit 1
./openocd.exe -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg \
    -c "adapter speed 4000" \
    -c "program \"$HEX\" verify reset exit" 2>&1 | grep -iE "wrote|verified|error|fail|Programming|verify" | head -20
