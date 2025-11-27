# AirSense 空气质量检测仪

基于ESP32-C5的多传感器空气质量检测验证机

## 项目概述

AirSense是一个集成多种空气质量传感器的检测系统，可以实时监测以下参数：
- **CO2浓度**: SenseAir S8传感器
- **VOC和NOx**: Sensirion SGP41传感器
- **NH3浓度**: Dart WZ-H3-N电化学传感器
- **气压和温度**: Infineon DPS310传感器

## 硬件配置

### MCU
- **型号**: ESP32-C5
- **架构**: RISC-V
- **频率**: 160 MHz

### 传感器列表

| 传感器 | 测量参数 | 接口 | I2C地址 | UART/端口 |
|--------|----------|------|---------|----------|
| SenseAir S8 | CO2 (0-2000 ppm) | UART | - | UART1 |
| SGP41 | VOC/NOx指数 | I2C | 0x59 | I2C0 |
| Dart WZ-H3-N | NH3 (0-100 ppm) | UART | - | UART2 |
| DPS310 | 气压/温度 | I2C | 0x77 | I2C1 |

### 引脚连接

```
ESP32-C5 引脚分配:
├── UART1 (SenseAir S8)
│   ├── TX: GPIO17
│   └── RX: GPIO18
├── UART2 (Dart WZ-H3-N)
│   ├── TX: GPIO21
│   └── RX: GPIO22
├── I2C0 (SGP41)
│   ├── SDA: GPIO6
│   └── SCL: GPIO7
└── I2C1 (DPS310)
    ├── SDA: GPIO4
    └── SCL: GPIO5
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
└── src/                        # 源代码
    ├── CMakeLists.txt          # 组件CMake配置
    ├── main.c                  # 主程序
    └── sensors/                # 传感器驱动
        ├── senseair_s8.h/c     # SenseAir S8驱动
        ├── sgp41.h/c           # SGP41驱动
        ├── dart_wzh3n.h/c      # Dart WZ-H3-N驱动
        └── dps310.h/c          # DPS310驱动
```

### 传感器驱动特性

#### SenseAir S8 (CO2传感器)
- 通信协议: Modbus RTU
- 波特率: 9600 bps
- 测量范围: 0-2000 ppm (可扩展到10000 ppm)
- 精度: ±40 ppm + 3% of reading
- 响应时间: 20秒 (90%)

#### SGP41 (VOC/NOx传感器)
- 通信协议: I2C
- I2C地址: 0x59
- 输出: 原始信号值 + 指数值(0-500)
- 预热时间: 10秒
- 特性: 湿度/温度补偿

#### Dart WZ-H3-N (NH3电化学传感器)
- 通信协议: UART
- 工作模式: 主动上传/问答模式
- 测量范围: 0-100 ppm
- 分辨率: 0.01 ppm
- 响应时间: < 30秒

#### DPS310 (气压传感器)
- 通信协议: I2C/SPI
- I2C地址: 0x77 (SDO=HIGH) / 0x76 (SDO=LOW)
- 气压范围: 300-1200 hPa
- 温度范围: -40 to 85°C
- 精度: ±0.005 hPa (±0.05 m)

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
I (340) SenseAir_S8: SenseAir S8 initialized on UART1 (TX:17, RX:18, Baud:9600)
I (350) SGP41: SGP41 initialized on I2C0 (SDA:6, SCL:7)
I (360) SGP41: Serial: 0000000012345678
I (365) SGP41: Self-test passed
I (370) Dart_WZH3N: Dart WZ-H3-N initialized on UART2 (TX:21, RX:22, Mode:Q&A)
I (380) AirSense: Sensor initialization complete
I (385) AirSense: System started successfully
I (5390) AirSense: === Reading Sensors ===
[S8] CO2: 412 ppm, Status: 0x00
[SGP41] VOC raw: 23456, NOx raw: 12345, VOC index: 178, NOx index: 93
[Dart] NH3: 0.12 ppm, Full Range: 100 ppm

```

## 使用说明

### 当前功能 (验证机版本)

1. **传感器初始化**: 系统启动时自动初始化所有传感器
2. **数据采集**: 每5秒读取一次所有传感器数据
3. **串口输出**: 实时通过串口输出传感器读数

### 引脚配置修改

如需修改引脚配置，请编辑 [main.c](src/main.c) 文件中的引脚定义：

```c
// UART1 - SenseAir S8
#define S8_TX_PIN               GPIO_NUM_17
#define S8_RX_PIN               GPIO_NUM_18

// UART2 - Dart WZ-H3-N
#define DART_TX_PIN             GPIO_NUM_21
#define DART_RX_PIN             GPIO_NUM_22

// I2C0 - SGP41
#define I2C0_SDA_PIN            GPIO_NUM_6
#define I2C0_SCL_PIN            GPIO_NUM_7

// I2C1 - DPS310
#define I2C1_SDA_PIN            GPIO_NUM_4
#define I2C1_SCL_PIN            GPIO_NUM_5
```

### 测量间隔调整

修改 [main.c](src/main.c) 中的 `MEASURE_INTERVAL_MS` 宏定义：

```c
#define MEASURE_INTERVAL_MS     5000  // 5秒
```

## 注意事项

### I2C总线配置

本项目使用两个独立的I2C总线避免设备冲突：
- **I2C0**: 连接SGP41传感器
- **I2C1**: 连接DPS310传感器

这样可以避免I2C设备地址冲突和时钟频率不匹配的问题。

### 传感器预热时间

- **SGP41**: 需要10秒预热，前30分钟数据可能不稳定
- **SenseAir S8**: 需要几分钟稳定时间
- **Dart WZ-H3-N**: 电化学传感器需要预热约1分钟

### 校准说明

- **CO2传感器**: 支持ABC自动校准，需在新鲜空气中运行7天
- **VOC传感器**: 自适应算法，24小时后达到最佳精度
- **气压传感器**: 出厂已校准，无需用户校准

## 后续开发计划

- [ ] 添加显示屏支持 (OLED/LCD)
- [ ] 实现数据存储功能 (SD卡/Flash)
- [ ] 添加WiFi连接和云端上传
- [ ] 实现低功耗模式
- [ ] 添加警报功能
- [ ] 开发移动端APP
- [ ] 增加数据可视化图表
- [ ] 支持更多空气质量传感器 (PM2.5, 甲醛等)

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
