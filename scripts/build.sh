#!/bin/bash
# 批量编译：UV4 -b，校验对象文件新鲜度 + 0 Error
cd "$(dirname "$0")/.." || exit 1
rm -f MDK-ARM/HOPE-Remote/cli.o MDK-ARM/HOPE-Remote/receiver.o MDK-ARM/HOPE-Remote/main.o
/c/Users/11846/AppData/Local/Keil_v5/UV4/UV4.exe -b MDK-ARM/HOPE-Remote.uvprojx -j0
echo "uv4 exit=$?"
sleep 2
# 校验关键对象是否重新生成（时间戳 > 本分钟）
ls -la --time-style=+%H:%M:%S MDK-ARM/HOPE-Remote/cli.o MDK-ARM/HOPE-Remote/receiver.o MDK-ARM/HOPE-Remote/main.o
if grep -qiE "[0-9]+ Error" MDK-ARM/HOPE-Remote/build_log.htm 2>/dev/null; then grep -iE "error" MDK-ARM/HOPE-Remote/build_log.htm | head; else echo "build log: no errors found (or log absent)"; fi
ls -la --time-style=+%H:%M:%S MDK-ARM/HOPE-Remote/HOPE-Remote.hex
