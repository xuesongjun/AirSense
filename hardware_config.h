/**
 * @file hardware_config.h
 * @brief AirSense硬件引脚配置文件
 *
 * 在这个文件中集中定义所有硬件引脚配置，便于适配不同的硬件版本
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "driver/gpio.h"

// ============================================================================
// ESP32-C5 引脚配置
// ============================================================================

// ----------------------------------------------------------------------------
// UART接口配置
// ----------------------------------------------------------------------------

// UART1 - SenseAir S8 CO2传感器
#define S8_UART_NUM             UART_NUM_1
#define S8_TX_PIN               GPIO_NUM_17
#define S8_RX_PIN               GPIO_NUM_18
#define S8_BAUD_RATE            9600

// UART2 - Dart WZ-H3-N NH3传感器
#define DART_UART_NUM           UART_NUM_2
#define DART_TX_PIN             GPIO_NUM_21
#define DART_RX_PIN             GPIO_NUM_22
#define DART_BAUD_RATE          9600

// ----------------------------------------------------------------------------
// I2C接口配置
// ----------------------------------------------------------------------------

// I2C0 - SGP41 VOC/NOx传感器
#define SGP41_I2C_NUM           I2C_NUM_0
#define SGP41_SDA_PIN           GPIO_NUM_6
#define SGP41_SCL_PIN           GPIO_NUM_7
#define SGP41_I2C_FREQ          100000      // 100kHz
#define SGP41_I2C_ADDR          0x59

// I2C1 - DPS310 气压/温度传感器
#define DPS310_I2C_NUM          I2C_NUM_1
#define DPS310_SDA_PIN          GPIO_NUM_4
#define DPS310_SCL_PIN          GPIO_NUM_5
#define DPS310_I2C_FREQ         400000      // 400kHz
#define DPS310_I2C_ADDR         0x77        // SDO连接到VDD

// ----------------------------------------------------------------------------
// SPI接口配置 (可选，DPS310可使用SPI)
// ----------------------------------------------------------------------------

// SPI2 - 备用DPS310 SPI接口
#define DPS310_SPI_HOST         SPI2_HOST
#define DPS310_SPI_MOSI         GPIO_NUM_11
#define DPS310_SPI_MISO         GPIO_NUM_13
#define DPS310_SPI_SCLK         GPIO_NUM_12
#define DPS310_SPI_CS           GPIO_NUM_10
#define DPS310_SPI_FREQ         1000000     // 1MHz

// ----------------------------------------------------------------------------
// 其他GPIO配置
// ----------------------------------------------------------------------------

// LED指示灯 (可选)
#define LED_STATUS_PIN          GPIO_NUM_2
#define LED_ERROR_PIN           GPIO_NUM_3

// 按键 (可选)
#define BUTTON_CALIBRATE_PIN    GPIO_NUM_0  // 校准按键
#define BUTTON_RESET_PIN        GPIO_NUM_1  // 复位按键

// 蜂鸣器 (可选)
#define BUZZER_PIN              GPIO_NUM_14

// ----------------------------------------------------------------------------
// 采样和定时配置
// ----------------------------------------------------------------------------

// 传感器采样间隔 (ms)
#define MEASURE_INTERVAL_MS     5000        // 5秒

// 传感器预热时间 (ms)
#define S8_WARMUP_TIME_MS       30000       // 30秒
#define SGP41_WARMUP_TIME_MS    10000       // 10秒
#define DART_WARMUP_TIME_MS     60000       // 60秒
#define DPS310_WARMUP_TIME_MS   50          // 50毫秒

// ----------------------------------------------------------------------------
// 传感器参数配置
// ----------------------------------------------------------------------------

// SenseAir S8 Modbus地址
#define S8_MODBUS_ADDR          0xFE

// SGP41温湿度补偿默认值
#define SGP41_DEFAULT_RH        50          // 相对湿度 50%
#define SGP41_DEFAULT_TEMP      25          // 温度 25°C

// Dart WZ-H3-N工作模式
#define DART_ACTIVE_UPLOAD      false       // false=问答模式, true=主动上传模式

// DPS310接口选择
#define DPS310_USE_SPI          false       // false=使用I2C, true=使用SPI

// ============================================================================
// 硬件版本配置
// ============================================================================

// 当前硬件版本
#define HARDWARE_VERSION        "v1.0"

// 支持的传感器 (0=禁用, 1=启用)
#define ENABLE_SENSOR_S8        1           // SenseAir S8
#define ENABLE_SENSOR_SGP41     1           // SGP41
#define ENABLE_SENSOR_DART      1           // Dart WZ-H3-N
#define ENABLE_SENSOR_DPS310    1           // DPS310

// 可选功能 (0=禁用, 1=启用)
#define ENABLE_LED              0           // LED指示灯
#define ENABLE_BUTTON           0           // 按键
#define ENABLE_BUZZER           0           // 蜂鸣器
#define ENABLE_DISPLAY          0           // 显示屏 (未实现)
#define ENABLE_WIFI             0           // WiFi功能 (未实现)
#define ENABLE_BLE              0           // 蓝牙功能 (未实现)

// ============================================================================
// 调试配置
// ============================================================================

// 日志级别
#define LOG_LEVEL_DEFAULT       ESP_LOG_INFO
#define LOG_LEVEL_SENSORS       ESP_LOG_DEBUG

// 串口输出格式
#define USE_SIMPLE_OUTPUT       1           // 1=简单格式, 0=详细格式

// ============================================================================
// 电源管理配置
// ============================================================================

// 低功耗模式
#define ENABLE_POWER_SAVE       0           // 低功耗模式
#define SLEEP_DURATION_MS       60000       // 深度睡眠时长 (ms)

// ============================================================================
// 数据存储配置
// ============================================================================

// 数据记录
#define ENABLE_DATA_LOGGING     0           // 数据记录功能
#define LOG_BUFFER_SIZE         100         // 记录缓冲区大小

#endif // HARDWARE_CONFIG_H
