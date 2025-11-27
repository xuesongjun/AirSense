# AirSense 项目摘要

## 项目信息

- **项目名称**: AirSense 空气质量检测仪
- **版本**: 1.0.0 (验证机)
- **MCU**: ESP32-C5 (RISC-V)
- **开发框架**: ESP-IDF v5.x
- **创建日期**: 2025-11-26

## 核心功能

本项目是一个多传感器空气质量检测系统，实时监测以下空气参数：

1. **CO2浓度** - SenseAir S8传感器 (UART)
2. **VOC/NOx指数** - Sensirion SGP41传感器 (I2C)
3. **NH3浓度** - Dart WZ-H3-N电化学传感器 (UART)
4. **气压/温度** - Infineon DPS310传感器 (I2C/SPI)

## 文件结构概览

```
AirSense/
│
├── 配置文件
│   ├── CMakeLists.txt                 # 顶层CMake构建配置
│   ├── sdkconfig.defaults             # ESP-IDF默认配置
│   └── .gitignore                     # Git忽略规则
│
├── 文档
│   ├── README.md                      # 完整项目说明
│   ├── QUICKSTART.md                  # 5分钟快速开始
│   ├── PROJECT_SUMMARY.md             # 本文档
│   └── doc/
│       ├── build_guide.md            # 详细编译和烧录指南
│       └── hardware_setup.md         # 硬件连接详细说明
│
├── 源代码
│   └── src/
│       ├── CMakeLists.txt            # 源码CMake配置
│       ├── main.c                    # 主程序入口
│       └── sensors/                  # 传感器驱动模块
│           ├── senseair_s8.h/c      # SenseAir S8 CO2传感器驱动
│           ├── sgp41.h/c            # SGP41 VOC/NOx传感器驱动
│           ├── dart_wzh3n.h/c       # Dart WZ-H3-N NH3传感器驱动
│           └── dps310.h/c           # DPS310气压传感器驱动
│
└── 其他
    ├── doc/                          # 数据手册和文档存储
    │   ├── mcu/                     # MCU相关文档
    │   └── sensor/                  # 传感器数据手册
    └── pcb/                         # PCB设计文件
```

## 技术规格

### 硬件接口

| 接口类型 | 传感器 | GPIO引脚 | 说明 |
|---------|--------|---------|------|
| UART1 | SenseAir S8 | TX: GPIO17, RX: GPIO18 | Modbus RTU, 9600bps |
| UART2 | Dart WZ-H3-N | TX: GPIO21, RX: GPIO22 | 问答模式, 9600bps |
| I2C0 | SGP41 | SDA: GPIO6, SCL: GPIO7 | 地址0x59, 100kHz |
| I2C0 | DPS310 | SDA: GPIO6, SCL: GPIO7 | 地址0x77, 400kHz |

### 软件架构

```
main.c
├── 初始化
│   ├── NVS Flash初始化
│   ├── 传感器驱动初始化
│   │   ├── senseair_s8_init()
│   │   ├── sgp41_init()
│   │   ├── dart_wzh3n_init()
│   │   └── dps310_init()
│   └── 创建任务
│       └── sensor_task (采集任务)
│
└── sensor_task (周期执行)
    ├── 读取SenseAir S8 CO2数据
    ├── 读取SGP41 VOC/NOx数据
    ├── 读取Dart WZ-H3-N NH3数据
    ├── 读取DPS310气压/温度数据
    └── 通过串口输出结果
```

## 传感器驱动特性

### 1. SenseAir S8 驱动 (senseair_s8.c/h)

**功能**:
- Modbus RTU协议实现
- CO2浓度读取 (0-2000 ppm)
- CRC16校验
- 支持ABC自动校准

**关键函数**:
- `senseair_s8_init()` - 初始化UART
- `senseair_s8_read_co2()` - 读取CO2数据
- `senseair_s8_calibrate_abc()` - ABC校准

### 2. SGP41 驱动 (sgp41.c/h)

**功能**:
- I2C通信
- VOC和NOx原始信号读取
- 温湿度补偿
- CRC8校验
- 自检功能
- 序列号读取

**关键函数**:
- `sgp41_init()` - 初始化I2C
- `sgp41_measure_raw()` - 测量VOC/NOx
- `sgp41_self_test()` - 自检
- `sgp41_get_serial_number()` - 读取序列号

### 3. Dart WZ-H3-N 驱动 (dart_wzh3n.c/h)

**功能**:
- UART通信
- 问答模式/主动上传模式
- NH3浓度读取 (0-100 ppm)
- 校验和验证
- 小数位自动处理

**关键函数**:
- `dart_wzh3n_init()` - 初始化UART
- `dart_wzh3n_read_concentration()` - 问答模式读取
- `dart_wzh3n_read_active()` - 主动上传模式读取

### 4. DPS310 驱动 (dps310.c/h)

**功能**:
- I2C/SPI双接口支持
- 气压测量 (300-1200 hPa)
- 温度测量 (-40 to 85°C)
- 海拔高度计算
- 校准系数读取
- 连续测量模式

**关键函数**:
- `dps310_init()` - 初始化I2C或SPI
- `dps310_read_data()` - 读取气压和温度
- `dps310_soft_reset()` - 软复位

## 编译和烧录

### 快速命令

```bash
# 设置目标
idf.py set-target esp32c5

# 编译
idf.py build

# 烧录并监视
idf.py -p COM3 flash monitor
```

详细说明请参考 [编译指南](doc/build_guide.md)

## 输出示例

```
I (320) AirSense: AirSense Air Quality Monitor
I (325) AirSense: MCU: ESP32-C5
I (330) AirSense: Version: 1.0.0
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

## 配置选项

### 引脚配置 (src/main.c)

```c
#define S8_TX_PIN               GPIO_NUM_17
#define S8_RX_PIN               GPIO_NUM_18
#define DART_TX_PIN             GPIO_NUM_21
#define DART_RX_PIN             GPIO_NUM_22
#define I2C_SDA_PIN             GPIO_NUM_6
#define I2C_SCL_PIN             GPIO_NUM_7
```

### 采样间隔 (src/main.c)

```c
#define MEASURE_INTERVAL_MS     5000  // 5秒
```

### ESP-IDF配置 (sdkconfig.defaults)

- 串口波特率: 115200
- 主任务栈大小: 4096字节
- 看门狗超时: 10秒

## 已知问题和限制

### 1. I2C总线共享

**问题**: SGP41和DPS310共享I2C0，当前驱动不支持总线共享
**状态**: DPS310初始化在main.c中被注释
**解决方案**:
- 修改驱动支持I2C总线共享
- 使用不同的I2C端口
- 使用DPS310的SPI接口

### 2. DPS310校准系数

**问题**: 校准系数解析未完全实现
**影响**: 气压和温度读数可能不准确
**解决方案**: 参考DPS310数据手册完善校准算法

### 3. SGP41指数计算

**问题**: VOC/NOx指数计算使用简化算法
**影响**: 指数值不是标准算法结果
**解决方案**: 集成Sensirion官方算法库

## 未来开发计划

### 短期 (1-2周)
- [ ] 修复I2C总线共享问题
- [ ] 完善DPS310驱动
- [ ] 添加错误重试机制
- [ ] 实现数据滤波算法

### 中期 (1-2月)
- [ ] 添加OLED/LCD显示屏
- [ ] 实现数据存储 (SD卡或Flash)
- [ ] 添加WiFi连接
- [ ] 实现OTA升级
- [ ] 设计外壳和PCB

### 长期 (3-6月)
- [ ] 云端数据上传
- [ ] 移动端APP开发
- [ ] 低功耗模式优化
- [ ] 警报和通知功能
- [ ] 多设备组网

## 相关资源

### 文档
- [快速开始指南](QUICKSTART.md)
- [完整README](README.md)
- [硬件连接指南](doc/hardware_setup.md)
- [编译指南](doc/build_guide.md)

### 数据手册
- SenseAir S8: `doc/sensor/`
- SGP41: `doc/sensor/`
- Dart WZ-H3-N: `doc/sensor/`
- DPS310: `doc/sensor/`
- ESP32-C5: `doc/mcu/`

### 在线资源
- ESP-IDF文档: https://docs.espressif.com/projects/esp-idf/
- ESP32-C5技术手册: https://www.espressif.com/
- Sensirion官方: https://www.sensirion.com/
- SenseAir官网: https://senseair.com/

## 贡献指南

欢迎提交Issue和Pull Request！

### 报告Bug
1. 描述问题现象
2. 提供串口日志
3. 说明硬件连接
4. 列出编译环境

### 提交代码
1. Fork本仓库
2. 创建功能分支
3. 编写测试
4. 提交Pull Request

## 许可证

MIT License

---

**最后更新**: 2025-11-26
**维护者**: AirSense Team
