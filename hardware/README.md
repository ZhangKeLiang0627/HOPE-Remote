# HOPE-Remote Hardware

## 1. 板级概览

```
Type-C ─┬─ VBUS ─┬─ 5 V 升压（PW5100-50）─→ 红外发射管 / 射频模块 / WS2812B
        │        └─ 3.3 V LDO ───────────→ MCU / CH340N / 红外接收头
        └─ D+/D- ─→ CH340N ─→ USART1 (PA9 / PA10)

PA0  ← 红外接收头（VS1838B）        PA1  → 红外发射管（38 kHz 载波）
PA3  ← 射频接收模块（灵-R1A）       PA5  → 射频发射模块（远-T2L）
PA6  → 2 × WS2812B                 PC13 → 用户指示灯
```

## 2. 设计文件

| 文件 | 用途 |
|---|---|
| `ProPrj_HOPE-Remote_2026-11-26.epro2` | 立创EDA 工程 |
| `HOPE-Remote-v1.2-kkl_Gerber_PCB_2026-11-26.zip` | 打样文件 |


## 3. 打样

推荐板厚1mm，无阻抗要求。
