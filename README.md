# AirSense 空气质量检测仪

基于esp32-c5-devkitc-1的多传感器空气质量检测验证机
参考连接:https://docs.espressif.com/projects/esp-dev-kits/zh_CN/latest/esp32c5/esp32-c5-devkitc-1/user_guide.html

## 项目概述

AirSense是一个集成多种空气质量传感器的检测系统，可以实时监测以下参数：
- **CO₂浓度 (NDIR)**: SenseAir S88LP（红外NDIR技术，UART Modbus接口）
- **CO₂浓度 (光声)**: Sensirion SCD41（光声技术，内置温湿度测量）
- **VOC 与 NOx 指数**: Sensirion SGP41（双 MOS 气体传感器，需配合 VOC/NOx 算法库）
- **颗粒物（PM1/PM2.5/PM4/PM10）**: Sensirion SPS30（激光散射粒子传感器）
- **甲醛 (HCHO)浓度**: Prosense WZ-H3-N 燃料电池HCHO传感器（抗酒精干扰，LP_UART硬件串口）
- **气压和温度**: Infineon DPS310（高精度气压计）
- **温湿度**: Sensirion SHT85 数字温湿度传感器（高精度工业级）

## 硬件配置

### MCU
- **型号**: ESP32-C5
- **架构**: RISC-V
- **频率**: 160 MHz

### 传感器列表

| 传感器 | 测量参数 | 接口 | I2C地址 | UART/GPIO |
|--------|----------|------|---------|----------|
| S88LP | CO2 (NDIR红外) | UART1硬件 | - | GPIO6(TX)/GPIO7(RX) |
| SCD41 | CO2/温度/湿度 | I2C | 0x62 | I2C0 |
| SGP41 | VOC/NOx指数 | I2C | 0x59 | I2C0 |
| SPS30 | PM1/PM2.5/PM4/PM10 | I2C | 0x69 | I2C0 |
| WZ-H3-N | HCHO甲醛 (0-5 ppm) | LP_UART硬件 | - | GPIO5(TX)/GPIO4(RX) 固定 |
| DPS310 | 气压/温度 | I2C | 0x77 | I2C0 |
| SHT85 | 温度/湿度 | I2C | 0x44 | I2C0 |

*注: ESP32-C5有3个UART: UART0(日志)、UART1(S88LP)、LP_UART(WZ-H3-N)*

### 引脚连接

```
ESP32-C5 引脚分配:
├── UART1 (硬件串口 - S88LP CO2传感器)
│   ├── TXD: GPIO6 (MCU TX -> 传感器 RX)
│   └── RXD: GPIO7 (MCU RX <- 传感器 TX)
├── LP_UART (硬件串口 - WZ-H3-N HCHO甲醛传感器)
│   ├── TXD: GPIO5 (MCU TX -> 传感器 RX，固定引脚)
│   └── RXD: GPIO4 (MCU RX <- 传感器 TX，固定引脚)
└── I2C0 (所有I2C传感器通过扩展板共用)
    ├── SDA: GPIO2
    └── SCL: GPIO3
    └── 连接传感器: SCD41, SGP41, SPS30, DPS310, SHT85

注1: UART0 用于固件烧录和日志输出，不可占用
注2: LP_UART是ESP32-C5的低功耗UART，引脚固定在GPIO4/GPIO5
注3: MCU的TXD连接到传感器的RXD，MCU的RXD连接到传感器的TXD
```

## 软件架构

### 目录结构

```
AirSense/
├── CMakeLists.txt              # 顶层CMake配置
├── sdkconfig.defaults          # ESP-IDF默认配置
├── README.md                   # 本文档
├── doc/                        # 文档目录
│   ├── mcu/                    # MCU相关文档
│   └── sensor/                 # 传感器数据手册
├── pcb/                        # PCB设计文件
└── main/                       # 主程序源代码
    ├── CMakeLists.txt          # 组件CMake配置
    ├── idf_component.yml       # ESP-IDF组件依赖配置
    ├── main.c                  # 主程序
    └── sensors/                # 传感器驱动
        ├── senseair_s88lp.h/c  # SenseAir S88LP CO2驱动 (Modbus)
        ├── sgp41.h/c           # SGP41 VOC/NOx驱动
        └── prosense_wzh3n.h/c  # Prosense WZ-H3-N HCHO驱动
        # DPS310/SCD41/SPS30/SHT85使用esp-idf-lib库
```

### 传感器驱动特性

#### SenseAir S88LP (CO2红外传感器)
- 通信协议: UART (Modbus)
- 波特率: 9600 bps
- 测量技术: NDIR红外 (非色散红外)

| 参数 | 规格 |
|------|------|
| 测量范围 | 400-2000 ppm (标准) / 400-10000 ppm (扩展) |
| 精度 | ±30 ppm + 3% of reading |
| 分辨率 | 1 ppm |
| 响应时间 | < 2分钟 (T90) |
| 预热时间 | < 2分钟 |
| 工作温度 | 0 to 50°C |
| 工作湿度 | 0-95% RH (非冷凝) |
| 寿命 | > 15年 |

**误差示例**: 读数800ppm时，误差 = ±30 + 800×3% = ±54 ppm，实际范围746-854 ppm

#### SCD41 (CO2/温湿度传感器)
- 通信协议: I2C
- I2C地址: 0x62
- 测量技术: 光声原理 (Photoacoustic)
- **气压补偿**: 使用DPS310实时气压自动补偿

| 参数 | 范围 | 精度/误差 |
|------|------|-----------|
| CO2 | 400-5000 ppm | ±(40 ppm + 5% of reading) |
| 温度 | -10 to 60°C | ±0.8°C (15-35°C), ±1.5°C (全范围) |
| 湿度 | 0-100% RH | ±6% RH (15-35°C), ±9% RH (全范围) |
| 响应时间 | 60秒 (T63) | - |
| 采样间隔 | 5秒 | - |

**误差示例**: CO2读数1000ppm时，误差 = ±40 + 1000×5% = ±90 ppm，实际范围910-1090 ppm

#### SGP41 (VOC/NOx传感器)
- 通信协议: I2C
- I2C地址: 0x59
- 测量技术: 金属氧化物半导体 (MOX)
- 特性: 湿度/温度补偿（使用SHT85数据）

| 参数 | 规格 |
|------|------|
| VOC Raw范围 | 0-65535 (典型室内: 25000-35000) |
| NOx Raw范围 | 0-65535 (典型室内: 14000-18000) |
| Index输出 | 1-500 (相对空气质量指数) |
| 预热时间 | 10秒 conditioning + 45秒算法初始化 |
| 工作温度 | -40 to 85°C |
| 寿命 | > 10年 |

**注意**: SGP41输出的是相对指数，不是绝对浓度值(ppb/ppm)。Index用于监测空气质量变化趋势。

**测量参数说明**:
- **VOC (Volatile Organic Compounds - 挥发性有机化合物)**
  - 包括: 甲醛、苯、甲苯、酒精、油漆挥发物、清洁剂等
  - 来源: 装修材料、家具、清洁剂、香水、化妆品、人体呼吸
  - 健康影响: 头痛、眼睛刺激、过敏反应，长期暴露可能致癌
  - Raw 值范围: 25000-35000 (典型室内)

- **NOx (Nitrogen Oxides - 氮氧化物)**
  - 包括: NO (一氧化氮)、NO₂ (二氧化氮)
  - 来源: 燃气灶、汽车尾气、吸烟、燃烧过程
  - 健康影响: 呼吸道刺激、哮喘恶化、肺功能下降
  - Raw 值范围: 14000-18000 (典型室内)

**空气质量指数 (Index) 解读**:
```
1-100:   优秀 - 空气非常干净，无明显污染
100-200: 良好 - 轻度污染，大部分人可接受
200-300: 中等 - 中度污染，敏感人群可能不适
300-400: 较差 - 重度污染，建议通风
400-500: 很差 - 严重污染，必须立即通风或离开
```

**重要说明**:
- **Index 是相对指标**，用于监测同一环境下的空气质量变化趋势
- **不是绝对浓度值** (ppb/ppm)，SGP41 传感器本身不能直接测量绝对 VOC 浓度
- **不同环境的基线不同**：城市室内、郊区室外的"正常"raw 值可能相差数千
- **传感器个体差异**：同型号传感器在相同环境下的 raw 值也会有差异
- **适用场景**：
  - ✅ 监测开窗通风效果（Index 下降）
  - ✅ 检测烹饪/清洁等活动导致的污染（Index 上升）
  - ✅ 日常室内空气质量趋势跟踪
  - ❌ 不适合作为法规标准依据（需要专业认证设备）
  - ❌ 不适合跨设备/跨环境对比

**算法实现**：
- 本项目使用**固定阈值映射算法**（基于 Sensirion 典型值范围）
- VOC baseline = 27000 (Index 100 边界)
- 与 Sensirion 官方自适应算法不同（官方需要 12-24 小时学习期）
- 优点：立即可用，无需等待学习；缺点：可能不适应所有环境

#### SPS30 (颗粒物传感器)
- 通信协议: I2C
- I2C地址: 0x69
- 测量技术: 激光散射

| 参数 | 范围 | 精度/误差 |
|------|------|-----------|
| PM1.0 | 0-1000 μg/m³ | ±10 μg/m³ (0-100), ±10% (100-1000) |
| PM2.5 | 0-1000 μg/m³ | ±10 μg/m³ (0-100), ±10% (100-1000) |
| PM4.0 | 0-1000 μg/m³ | ±25 μg/m³ (0-100), ±25% (100-1000) |
| PM10 | 0-1000 μg/m³ | ±25 μg/m³ (0-100), ±25% (100-1000) |
| 粒径检测 | 0.3-10 μm | - |
| 采样间隔 | 1秒 | - |
| 寿命 | > 8年 (连续运行) | - |

**PM2.5等级参考** (中国AQI标准):
| 等级 | PM2.5浓度 | 空气质量 |
|------|-----------|----------|
| 优 | 0-35 μg/m³ | 空气质量令人满意 |
| 良 | 35-75 μg/m³ | 可接受，敏感人群注意 |
| 轻度污染 | 75-115 μg/m³ | 敏感人群有症状 |
| 中度污染 | 115-150 μg/m³ | 进一步加剧症状 |
| 重度污染 | 150-250 μg/m³ | 所有人健康影响 |
| 严重污染 | >250 μg/m³ | 健康警报 |

#### Prosense WZ-H3-N (HCHO甲醛传感器)
- 通信协议: LP_UART硬件串口 (GPIO4/GPIO5固定)
- 波特率: 9600 bps (8N1)
- 测量技术: 电化学燃料电池
- 工作模式: 问答模式 (同时输出ug/m³和ppb)

| 参数 | 规格 |
|------|------|
| 测量范围 | 0-1 ppm (0-1250 μg/m³)，过载5ppm |
| 分辨率 | 0.01 ppm (10 ppb) |
| 精度 | ±30 ppb 或 ±10% 取大值 (25±3°C) |
| 响应时间 | < 90秒 (T90) |
| 工作温度 | -40 to 70°C |
| 零点漂移 | < ±20 ppb/年 |
| 寿命 | > 6年 |
| 特性 | 抗酒精干扰、高选择性 |

**精度说明**:
- 低浓度时 (<300 ppb): 误差以±30 ppb为主
- 高浓度时 (>300 ppb): 误差以±10%为主
- 例如: 读数100 ppb时，实际范围70-130 ppb；读数500 ppb时，实际范围450-550 ppb

**甲醛(HCHO)浓度标准与健康影响**:

| 等级 | 浓度范围 | 说明 |
|------|----------|------|
| 优秀 | ≤30 ug/m³ (≤0.03 mg/m³) | 非常安全，适合敏感人群 |
| 良好 | 30-50 ug/m³ (0.03-0.05 mg/m³) | 安全，适合长期居住 |
| 合格 | 50-80 ug/m³ (0.05-0.08 mg/m³) | 符合国标GB/T 18883，可接受 |
| 轻度超标 | 80-100 ug/m³ (0.08-0.10 mg/m³) | 超过国标，建议通风 |
| 中度超标 | 100-200 ug/m³ (0.10-0.20 mg/m³) | 可能引起眼鼻刺激 |
| 严重超标 | >200 ug/m³ (>0.20 mg/m³) | 明显不适，需立即处理 |

**相关标准**:
- **中国 GB/T 18883-2022**: ≤0.08 mg/m³ (80 ug/m³) - 关闭门窗12小时
- **中国 GB 50325-2020**: Ⅰ类建筑≤0.07 mg/m³，Ⅱ类≤0.08 mg/m³
- **WHO世界卫生组织**: 长期≤0.1 mg/m³，短期(30分钟)≤0.2 mg/m³
- **敏感人群建议**: ≤0.05 mg/m³ (50 ug/m³)

**单位换算**: 1 ppm ≈ 1.25 mg/m³ = 1250 ug/m³ (25°C, 1atm)

#### DPS310 (气压/温度传感器)
- 通信协议: I2C/SPI
- I2C地址: 0x77 (SDO=HIGH) / 0x76 (SDO=LOW)
- 测量技术: 电容式MEMS

| 参数 | 范围 | 精度/误差 |
|------|------|-----------|
| 气压 | 300-1200 hPa | ±0.06 hPa (相对), ±1 hPa (绝对) |
| 温度 | -40 to 85°C | ±0.5°C |
| 海拔分辨率 | - | ±0.05 m (相对高度变化) |
| 采样速率 | 1-128 次/秒 | - |
| 噪声 | - | 0.002 hPa (最低噪声模式) |

**应用说明**: 本项目用于SCD41气压补偿和相对高度变化检测

#### SHT85 (温湿度传感器)
- 通信协议: I2C
- I2C地址: 0x44
- 测量技术: CMOSens (电容式)

| 参数 | 范围 | 精度/误差 |
|------|------|-----------|
| 温度 | -40 to 125°C | ±0.1°C (典型), ±0.2°C (最大, 20-60°C) |
| 湿度 | 0-100% RH | ±1.5% RH (典型), ±2% RH (最大, 10-90%) |
| 响应时间 | - | 8秒 (τ63%) |
| 重复性 | - | 温度 0.04°C, 湿度 0.08% RH |
| 漂移 | - | 温度 <0.03°C/年, 湿度 <0.25% RH/年 |

**精度等级**: 工业级高精度传感器，用于SCD41温度补偿和SGP41湿度补偿基准

## 编译和烧录

### 环境要求

- ESP-IDF v5.0或更高版本
- CMake 3.16+
- Python 3.8+

### 编译步骤

```bash
# 1. 设置ESP-IDF环境
. $HOME/esp/esp-idf/export.sh

# 2. 进入项目目录
cd AirSense

# 3. 配置目标芯片
idf.py set-target esp32c5

# 4. 配置项目 (可选)
idf.py menuconfig

# 5. 编译项目
idf.py build

# 6. 烧录到设备
idf.py -p COM3 flash

# 7. 查看串口输出
idf.py -p COM3 monitor
```

### 串口输出示例

```
I (320) AirSense: AirSense Air Quality Monitor
I (325) AirSense: MCU: ESP32-C5
I (330) AirSense: Version: 1.0.0
I (335) AirSense: Initializing sensors...
I (340) SCD41: SCD41 initialized on I2C0 (SDA:2, SCL:3)
I (345) SCD41: Serial: 0x1234567890AB
I (350) SGP41: SGP41 initialized on I2C0 (SDA:2, SCL:3)
I (355) SGP41: Serial: 0x0000000012345678
I (360) SGP41: Self-test passed
I (365) SPS30: SPS30 initialized on I2C0 (SDA:2, SCL:3)
I (370) SPS30: Serial: 12345678901234567890
I (375) WZ-H3-N: Prosense WZ-H3-N initialized on LP_UART (TX:GPIO5, RX:GPIO4, Q&A Mode)
I (380) DPS310: DPS310 initialized on I2C0 (SDA:2, SCL:3)
I (385) DPS310: Product ID: 0x10, Revision ID: 0x01
I (390) SHT85: SHT85 initialized on I2C0 (SDA:2, SCL:3)
I (395) AirSense: Sensor initialization complete
I (400) AirSense: System started successfully
I (5405) AirSense: === Reading Sensors ===
[SCD41] CO2: 412 ppm, Temp: 23.5°C, Humidity: 45.2% RH
[SGP41] VOC raw: 23456, NOx raw: 12345, VOC index: 178, NOx index: 93
[SPS30] PM1.0: 5.2 μg/m³, PM2.5: 8.7 μg/m³, PM4.0: 10.3 μg/m³, PM10: 11.5 μg/m³
[WZ-H3-N] HCHO: 8 ug/m³ (0.008 mg/m³) / 6 ppb (0.006 ppm)
[DPS310] Pressure: 1013.25 hPa, Temp: 23.4°C
[SHT85] Temp: 23.6°C, Humidity: 45.0% RH

```

## 使用说明

### 当前功能 (验证机版本)

1. **传感器初始化**: 系统启动时自动初始化所有传感器
2. **SCD41自动温度补偿**: 启动时自动对比SHT85和SCD41温度,设置温度偏移补偿(补偿SCD41自热效应)
3. **数据采集**: 每3秒读取一次所有传感器数据
4. **串口输出**: 实时通过串口输出传感器读数
5. **命令行交互**: 通过串口终端发送命令进行配置

### 可用命令

系统通过串口监视器提供交互式命令行界面，支持以下命令：

#### `help`
显示所有可用命令的帮助信息

```bash
help
```

输出示例：
```
=== AirSense Available Commands ===

--- SCD41 CO2 Sensor ---
  calibrate_co2 <ppm>      - Forced CO2 calibration (400-2000)
  factory_reset            - Restore SCD41 to factory defaults
  set_temp_offset <celsius>- Manual temp offset (auto at startup)
  Note: Pressure & temperature compensation are automatic

--- SGP41 VOC/NOx Sensor ---
  set_voc_baseline <value> - Set VOC baseline (10000-60000)
  set_nox_baseline <value> - Set NOx baseline (5000-40000)
  get_baseline             - Show current baseline values

--- DPS310 Pressure Sensor ---
  set_alt_ref <altitude>   - Set altitude reference (meters)

--- Wi-Fi ---
  set_wifi <ssid> <password> - Update Wi-Fi credentials and reconnect
  get_ip                     - Show current STA IP (if connected)

--- General ---
  help                     - Show this help message

Note: SCD41 auto-compensated by DPS310 pressure
```

#### `set_alt_ref <altitude>`
设置海拔高度参考值（单位：米），用于 DPS310 气压传感器计算海拔高度

**参数**:
- `<altitude>`: 当前位置的海拔高度（米）

**示例**:
```bash
# 设置海拔为 100 米
set_alt_ref 100

# 设置海拔为海平面（0 米）
set_alt_ref 0

# 设置海拔为 1500 米（高原地区）
set_alt_ref 1500
```

**说明**:
- DPS310 传感器测量的是绝对气压值
- 通过设置当前位置的真实海拔，可以让传感器计算相对海拔变化
- 海拔公式: `h = 44330 * (1 - (P/P0)^0.1903)`，其中 P0 为参考气压
- 此命令仅在编译时启用 DPS310 传感器（`ENABLE_DPS310 1`）时可用

**注意事项**:
- 首次使用建议在已知海拔位置设置参考值
- 设置后，传感器会根据当前气压计算相对海拔高度
- 气压会随天气变化，海拔计算值会有±5米的正常波动

#### `set_voc_baseline <value>`
设置 SGP41 传感器的 VOC（挥发性有机化合物）基线值

**参数**:
- `<value>`: VOC 原始信号基线值（有效范围：10000-60000，默认：27000）

**示例**:
```bash
# 设置 VOC 基线为 30000（适用于城市室内环境）
set_voc_baseline 30000

# 设置 VOC 基线为 25000（适用于郊区/农村环境）
set_voc_baseline 25000

# 根据当前环境的实际 raw 值设置
set_voc_baseline 28500
```

**说明**:
- 基线值是 Index = 100 的边界，代表"优秀"空气质量的上限
- 设置后，Index 的阈值会自动调整：
  - Index 1-100（优秀）: raw ≤ baseline
  - Index 100-200（良好）: baseline < raw ≤ baseline+5000
  - Index 200-300（中等）: baseline+5000 < raw ≤ baseline+10000
  - Index 300-400（较差）: baseline+10000 < raw ≤ baseline+15000
  - Index 400-500（很差）: raw > baseline+15000
- 此命令仅在编译时启用 SGP41 传感器时可用

**如何确定合适的基线值**:
1. 在通风良好的干净环境中运行设备 30 分钟
2. 观察串口输出的 `VOC raw` 值（例如：`[SGP41] VOC raw: 29500`）
3. 取多次测量的平均值作为基线（例如：29000-30000 的平均值 = 29500）
4. 使用命令设置：`set_voc_baseline 29500`

**注意事项**:
- 不同环境的"正常"raw 值差异很大（城市室内 vs 郊区室外可能相差数千）
- 建议在目标使用环境中校准基线
- 基线设置立即生效，无需重启设备
- 设置不会持久化存储，重启后恢复默认值

#### `set_nox_baseline <value>`
设置 SGP41 传感器的 NOx（氮氧化物）基线值

**参数**:
- `<value>`: NOx 原始信号基线值（有效范围：5000-40000，默认：15000）

**示例**:
```bash
# 设置 NOx 基线为 16000（适用于城市环境）
set_nox_baseline 16000

# 设置 NOx 基线为 14000（适用于农村环境）
set_nox_baseline 14000

# 根据当前环境的实际 raw 值设置
set_nox_baseline 15500
```

**说明**:
- 基线值是 Index = 100 的边界，代表"优秀"空气质量的上限
- 设置后，Index 的阈值会自动调整：
  - Index 1-100（优秀）: raw ≤ baseline
  - Index 100-200（良好）: baseline < raw ≤ baseline+3000
  - Index 200-300（中等）: baseline+3000 < raw ≤ baseline+7000
  - Index 300-400（较差）: baseline+7000 < raw ≤ baseline+12000
  - Index 400-500（很差）: raw > baseline+12000
- 此命令仅在编译时启用 SGP41 传感器时可用

**如何确定合适的基线值**:
1. 在无燃气、无燃烧、无吸烟的干净环境中运行设备 30 分钟
2. 观察串口输出的 `NOx raw` 值（例如：`[SGP41] NOx raw: 15800`）
3. 取多次测量的平均值作为基线
4. 使用命令设置：`set_nox_baseline 15800`

**注意事项**:
- NOx 主要来源：燃气灶、汽车尾气、吸烟
- 城市环境的基线通常高于农村环境
- 基线设置立即生效，无需重启设备
- 设置不会持久化存储，重启后恢复默认值

#### `get_baseline`
显示当前 SGP41 传感器的 VOC 和 NOx 基线值

```bash
get_baseline
```

输出示例：
```
Current baselines:
  VOC: 30000 (default: 27000)
  NOx: 16000 (default: 15000)
```

**说明**:
- 显示当前使用的基线值
- 如果未手动设置，显示默认值
- 用于确认基线设置是否生效

#### `calibrate_co2 <ppm>`
对 SCD41 传感器执行强制 CO2 校准（FRC - Forced Recalibration）

**参数**:
- `<ppm>`: 目标 CO2 浓度（有效范围：400-2000 ppm）

**示例**:
```bash
# 在室外新鲜空气中校准到 400 ppm
calibrate_co2 400

# 在已知浓度环境中校准（如使用标准气体）
calibrate_co2 1000
```

**使用步骤**:
1. **准备环境**：将设备放置在目标CO2浓度环境中
   - 室外新鲜空气：约400 ppm
   - 标准气体：使用已知浓度的标准气体
2. **稳定运行**：让设备在该环境中运行**至少3分钟**，确保读数稳定
3. **执行校准**：输入命令 `calibrate_co2 400`（或实际CO2浓度）
4. **等待完成**：校准过程约需400ms，完成后会显示FRC修正值
5. **验证结果**：观察后续CO2读数是否接近目标值

**输出示例**:
```
calibrate_co2 400

Starting SCD41 forced recalibration to 400 ppm...
WARNING: Sensor must be in stable conditions for 3+ minutes
Calibration successful!
FRC correction: -230 ppm
Restarting measurement...
```

**说明**:
- **FRC correction**: 显示校准修正值（当前读数与目标值的差值）
  - 正值：传感器读数偏高，需要减去修正值
  - 负值：传感器读数偏低，需要加上修正值
- 校准值**会持久化存储**在SCD41内部，重启后仍然有效
- 此命令仅在编译时启用 SCD41 传感器时可用

**注意事项**:
- ⚠️ **环境稳定性至关重要**：
  - 必须在CO2浓度稳定的环境中进行
  - 建议在室外无风、无人流的地方校准
  - 室内校准需确保通风良好且人员活动停止3分钟以上
- ⚠️ **校准频率**：
  - 不建议频繁校准（每月1次即可）
  - 过度校准可能降低精度
- ⚠️ **校准失败原因**：
  - 传感器运行时间不足3分钟
  - 环境CO2浓度不稳定
  - 目标值与实际浓度差异过大（>100 ppm）
- 💡 **推荐做法**：
  - 新传感器首次使用时在室外校准一次
  - 之后依赖SCD41的自动自校准（ASC）功能
  - 仅在发现读数明显偏差时手动校准

**与S88LP读数对比**:
- 校准后，SCD41读数应更接近S88LP（NDIR技术，更准确）
- 如果校准后两者仍有差异，以S88LP为准

#### `factory_reset`
恢复 SCD41 传感器到出厂默认设置

```bash
factory_reset
```

**使用场景**:
- 校准失败后需要恢复传感器状态
- 传感器读数异常（如显示0 ppm）
- 需要清除所有自定义校准数据

**输出示例**:
```
Performing SCD41 factory reset...
Factory reset successful!
Restarting measurement...
```

**说明**:
- 此命令会清除所有用户校准数据，恢复出厂默认值
- 执行后需要等待约1.2秒让传感器重新初始化
- 恢复后，温度偏移和海拔补偿需要重新设置（系统会在下次重启时自动设置）

**注意事项**:
- ⚠️ 如果校准命令返回错误码 `0x8000`（-32768），说明校准失败，应执行此命令恢复
- 恢复后建议等待传感器运行3分钟以上再进行新的校准

### 引脚配置修改

如需修改引脚配置，请编辑 [main.c](main/main.c) 文件中的引脚定义：

```c
// UART1 - S88LP CO2传感器 (硬件UART)
#define S88LP_TXD_PIN           GPIO_NUM_6
#define S88LP_RXD_PIN           GPIO_NUM_7

// LP_UART - Prosense WZ-H3-N HCHO传感器 (硬件UART，引脚固定)
#define PROSENSE_TXD_PIN        GPIO_NUM_5
#define PROSENSE_RXD_PIN        GPIO_NUM_4

// I2C0 - 所有I2C传感器 (SCD41, SGP41, SPS30, DPS310, SHT85)
#define I2C0_SDA_PIN            GPIO_NUM_2
#define I2C0_SCL_PIN            GPIO_NUM_3
```

### 测量间隔调整

修改 [main.c](main/main.c) 中的 `MEASURE_INTERVAL_MS` 宏定义：

```c
#define MEASURE_INTERVAL_MS     5000  // 5秒
```

## 注意事项

### I2C总线配置

本项目所有I2C传感器通过I2C总线扩展板连接到同一个I2C总线：
- **I2C0**: 连接所有I2C传感器 (SCD41, SGP41, SPS30, DPS310, SHT85)
- **扩展板**: 使用I2C总线扩展板统一管理多个传感器

所有传感器使用不同的I2C地址（0x62, 0x59, 0x69, 0x77, 0x44），不会产生地址冲突。

### SCD41自动补偿

系统会自动对SCD41执行两种补偿：

#### 1. 温度偏移补偿（启动时执行一次）

**原理**:
- SCD41内部有自热效应,传感器温度比环境温度高约1.5-4°C
- 自热会影响CO2读数准确性
- 使用外部高精度温度传感器(SHT85)测量真实环境温度
- 自动计算偏移量并设置到SCD41

**校准过程**:
1. 系统启动后等待10秒,让传感器稳定
2. 读取SHT85和SCD41的温度
3. 计算温度偏移 = SCD41温度 - SHT85温度
4. 如果偏移在0.5~5°C范围内,自动设置到SCD41

#### 2. 气压动态补偿（每次测量时执行）

**原理**:
- SCD41的CO2测量基于光声技术，气压变化会影响测量结果
- 传统方法是设置静态海拔高度，由传感器内部估算气压
- 本项目使用DPS310实时气压进行动态补偿，更加精确

**工作方式**:
- 每次传感器采样周期（3秒），读取DPS310的实时气压
- 将气压值（hPa）直接发送给SCD41进行补偿
- 气压范围：700-1200 hPa

**优势**:
- 比静态海拔补偿更精确（海拔补偿只是用来估算气压）
- 能够响应天气变化导致的气压波动
- 无需用户手动设置海拔

**日志输出示例**:
```
I (12345) AirSense: Starting SCD41 automatic temperature offset calibration...
I (12346) AirSense: Waiting for sensors to stabilize (10 seconds)...
I (22456) AirSense: Temperature offset detected: 2.56°C (SCD41=24.04°C, SHT85=21.48°C)
I (22567) AirSense: Normal self-heating detected, applying temperature offset compensation
I (22678) AirSense: SCD41 temperature offset set to 2.56°C
I (23123) AirSense: SCD41 periodic measurement restarted with calibration
```

**注意事项**:
- 温度补偿仅在同时启用SCD41和SHT85时生效
- 气压补偿仅在同时启用SCD41和DPS310时生效
- 温度偏移必须在0.5~5°C范围内才会设置(避免异常值)
- 校准失败时会使用默认设置继续运行
- 温度补偿不会持久化存储,每次重启都会重新校准
- 气压补偿是实时动态的，无需持久化

### 传感器预热时间

- **SCD41**: 首次测量需要约60秒，建议预热5分钟后读数更稳定
- **SGP41**:
  - Conditioning: 10秒（初始化时自动执行，Sensirion 官方要求）
  - 算法预热: 45秒（返回有效 Index 前的等待时间）
  - 总计: 约55秒后开始输出稳定数据
- **SPS30**: 启动后约10秒开始输出稳定数据，建议预热30秒
- **Prosense WZ-H3-N**: 固态电解质传感器需要预热约90秒 (T90)
- **DPS310**: 立即可用，无需预热
- **SHT85**: 上电后立即可用，建议等待2秒

### 校准说明

- **SCD41 (CO2传感器)**: 支持FRC（强制校准）和ASC（自动自校准），气压由DPS310自动补偿
- **SGP41 (VOC/NOx传感器)**:
  - 使用固定阈值映射算法，无需校准
  - 自动执行 conditioning 步骤（初始化时）
  - Index 用于相对变化监测，不代表绝对浓度
- **SPS30 (颗粒物传感器)**: 出厂已校准，支持定期自动清洁功能延长寿命
- **WZ-H3-N (HCHO传感器)**: 出厂已校准，建议根据温湿度进行软件补偿
- **DPS310 (气压传感器)**: 出厂已校准，无需用户校准
- **SHT85 (温湿度传感器)**: 出厂已校准，精度极高无需用户校准

## 后续开发计划

- [ ] 添加显示屏支持 (OLED/LCD)
- [ ] 实现数据存储功能 (SD卡/Flash)
- [ ] 添加WiFi连接和云端上传
- [ ] 实现低功耗模式
- [ ] 添加警报功能（CO2/VOC/PM2.5/甲醛超标报警）
- [ ] 开发移动端APP
- [ ] 增加数据可视化图表和历史数据分析
- [ ] 支持更多空气质量传感器 (臭氧、氡气等)

## 故障排查

### 传感器无响应

1. 检查引脚连接是否正确
2. 检查传感器供电是否正常 (3.3V或5V)
3. 使用逻辑分析仪检查通信波形
4. 查看串口日志中的错误信息

### I2C通信失败

1. 检查SDA/SCL上拉电阻 (通常4.7kΩ)
2. 确认I2C地址是否正确
3. 降低I2C时钟频率尝试
4. 检查是否有地址冲突

### UART通信失败

1. 检查TX/RX引脚是否交叉连接
2. 确认波特率设置正确
3. 检查UART参数 (数据位、停止位、校验位)
4. 使用示波器检查信号电平

## 许可证

本项目采用 MIT 许可证

## 联系方式

如有问题或建议，请提交Issue或联系项目维护者。
