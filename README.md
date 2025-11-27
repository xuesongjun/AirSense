# AirSense 空气质量检测仪

基于ESP32-C5的多传感器空气质量检测验证机

## 项目概述

AirSense是一个集成多种空气质量传感器的检测系统，可以实时监测以下参数：
- **CO₂浓度 (NDIR)**: SenseAir S88LP（红外NDIR技术，UART Modbus接口）
- **CO₂浓度 (光声)**: Sensirion SCD41（光声技术，内置温湿度测量）
- **VOC 与 NOx 指数**: Sensirion SGP41（双 MOS 气体传感器，需配合 VOC/NOx 算法库）
- **颗粒物（PM1/PM2.5/PM4/PM10）**: Sensirion SPS30（激光散射粒子传感器）
- **甲醛 (HCHO)浓度**: Dart WZ-H3-N 电化学HCHO传感器，需要温度/湿度补偿
- **气压和温度**: Infineon DPS310（高精度气压计）
- **温湿度**: Sensirion SHT85 数字温湿度传感器（高精度工业级）

## 硬件配置

### MCU
- **型号**: ESP32-C5
- **架构**: RISC-V
- **频率**: 160 MHz

### 传感器列表

| 传感器 | 测量参数 | 接口 | I2C地址 | UART/端口 |
|--------|----------|------|---------|----------|
| S88LP | CO2 (NDIR红外) | UART | - | UART1 |
| SCD41 | CO2/温度/湿度 | I2C | 0x62 | I2C0 |
| SGP41 | VOC/NOx指数 | I2C | 0x59 | I2C0 |
| SPS30 | PM1/PM2.5/PM4/PM10 | I2C | 0x69 | I2C0 |
| WZ-H3-N | HCHO (0-5 ppm) | UART | - | UART1* |
| DPS310 | 气压/温度 | I2C | 0x77 | I2C0 |
| SHT85 | 温度/湿度 | I2C | 0x44 | I2C0 |

*注: S88LP和WZ-H3-N共用UART1，不能同时启用（ESP32-C5仅有UART1可用于外设）

### 引脚连接

```
ESP32-C5 引脚分配:
├── UART1 (S88LP CO2传感器 或 WZ-H3-N HCHO传感器，二选一)
│   ├── TXD: GPIO5
│   └── RXD: GPIO4
└── I2C0 (所有I2C传感器通过扩展板共用)
    ├── SDA: GPIO2
    └── SCL: GPIO3
    └── 连接传感器: SCD41, SGP41, SPS30, DPS310, SHT85

注: UART0 (TXD0/RXD0) 用于固件烧录和日志输出，不可占用
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
        ├── senseair_s8.h/c     # SenseAir S88LP/S8 CO2驱动 (Modbus)
        ├── sgp41.h/c           # SGP41 VOC/NOx驱动
        └── dart_wzh3n.h/c      # Dart WZ-H3-N HCHO驱动
        # DPS310/SCD41/SPS30/SHT85使用esp-idf-lib库
```

### 传感器驱动特性

#### SenseAir S88LP (CO2红外传感器)
- 通信协议: UART (Modbus)
- 波特率: 9600 bps
- 测量技术: NDIR红外
- CO2测量范围: 400-2000 ppm (标准) / 400-10000 ppm (扩展)
- CO2精度: ±40 ppm + 3% of reading
- 响应时间: < 2分钟 (90%)
- 预热时间: < 2分钟
- 寿命: > 15年

#### SCD41 (CO2/温湿度传感器)
- 通信协议: I2C
- I2C地址: 0x62
- CO2测量范围: 400-5000 ppm
- CO2精度: ±(40 ppm + 5% of reading)
- 温度范围: -10 to 60°C, 精度: ±0.8°C
- 湿度范围: 0-100% RH, 精度: ±6% RH
- 响应时间: 60秒
- 采样间隔: 5秒

#### SGP41 (VOC/NOx传感器)
- 通信协议: I2C
- I2C地址: 0x59
- 输出: 原始信号值 + 指数值(0-500)
- 预热时间: 10秒
- 特性: 湿度/温度补偿
- 自适应算法: 24小时学习周期

#### SPS30 (颗粒物传感器)
- 通信协议: I2C
- I2C地址: 0x69
- 测量参数: PM1.0, PM2.5, PM4.0, PM10
- 测量范围: 0-1000 μg/m³
- 精度: ±10 μg/m³ (0-100 μg/m³)
- 粒径范围: 0.3-10 μm
- 采样间隔: 1秒

#### Dart WZ-H3-N (HCHO电化学传感器)
- 通信协议: UART
- 波特率: 9600 bps
- 工作模式: 主动上传/问答模式
- 测量范围: 0-5 ppm (甲醛)
- 分辨率: 0.01 ppm
- 响应时间: < 30秒
- 特性: 需要温度/湿度补偿

#### DPS310 (气压/温度传感器)
- 通信协议: I2C/SPI
- I2C地址: 0x77 (SDO=HIGH) / 0x76 (SDO=LOW)
- 气压范围: 300-1200 hPa
- 温度范围: -40 to 85°C
- 精度: ±0.005 hPa (±0.05 m)
- 采样速率: 可配置 1-128 次/秒

#### SHT85 (温湿度传感器)
- 通信协议: I2C
- I2C地址: 0x44
- 温度范围: -40 to 125°C
- 温度精度: ±0.1°C (典型值)
- 湿度范围: 0-100% RH
- 湿度精度: ±1.5% RH (典型值)
- 响应时间: 8秒 (τ63%)

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
I (375) Dart_WZH3N: Dart WZ-H3-N initialized on UART1 (TXD:5, RXD:4, Baud:9600)
I (380) DPS310: DPS310 initialized on I2C0 (SDA:2, SCL:3)
I (385) DPS310: Product ID: 0x10, Revision ID: 0x01
I (390) SHT85: SHT85 initialized on I2C0 (SDA:2, SCL:3)
I (395) AirSense: Sensor initialization complete
I (400) AirSense: System started successfully
I (5405) AirSense: === Reading Sensors ===
[SCD41] CO2: 412 ppm, Temp: 23.5°C, Humidity: 45.2% RH
[SGP41] VOC raw: 23456, NOx raw: 12345, VOC index: 178, NOx index: 93
[SPS30] PM1.0: 5.2 μg/m³, PM2.5: 8.7 μg/m³, PM4.0: 10.3 μg/m³, PM10: 11.5 μg/m³
[Dart] HCHO: 0.02 ppm
[DPS310] Pressure: 1013.25 hPa, Temp: 23.4°C
[SHT85] Temp: 23.6°C, Humidity: 45.0% RH

```

## 使用说明

### 当前功能 (验证机版本)

1. **传感器初始化**: 系统启动时自动初始化所有传感器
2. **数据采集**: 每5秒读取一次所有传感器数据
3. **串口输出**: 实时通过串口输出传感器读数

### 引脚配置修改

如需修改引脚配置，请编辑 [main.c](main/main.c) 文件中的引脚定义：

```c
// UART1 - S88LP CO2传感器 或 WZ-H3-N HCHO传感器 (二选一)
#define S88LP_TXD_PIN           GPIO_NUM_5
#define S88LP_RXD_PIN           GPIO_NUM_4
#define DART_TXD_PIN            GPIO_NUM_5
#define DART_RXD_PIN            GPIO_NUM_4

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

### 传感器预热时间

- **SCD41**: 首次测量需要约60秒，建议预热5分钟后读数更稳定
- **SGP41**: 需要10秒预热，前24小时数据可能不稳定（自适应学习阶段）
- **SPS30**: 启动后约10秒开始输出稳定数据，建议预热30秒
- **Dart WZ-H3-N**: 电化学传感器需要预热约1-2分钟
- **DPS310**: 立即可用，无需预热
- **SHT85**: 上电后立即可用，建议等待2秒

### 校准说明

- **SCD41 (CO2传感器)**: 支持FRC（强制校准）和ASC（自动自校准），ASC需在新鲜空气中定期运行
- **SGP41 (VOC传感器)**: 自适应算法，24小时后达到最佳精度，无需手动校准
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
