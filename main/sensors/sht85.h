/**
 * @file sht85.h
 * @brief Sensirion SHT85 温湿度传感器驱动 (I2C接口)
 *
 * SHT85是Sensirion的高精度温湿度传感器
 * I2C地址: 0x44 (ADDR引脚接地) 或 0x45 (ADDR引脚接VDD)
 * 测量范围: 温度 -40~125°C, 湿度 0~100%RH
 * 精度: ±0.1°C, ±1.5%RH
 */

#ifndef SHT85_H
#define SHT85_H

#include <stdint.h>
#include <stdbool.h>
#if defined(__has_include)
#if __has_include("hal/i2c_types.h")
#include "hal/i2c_types.h"
#else
#include "driver/i2c.h"
#endif
#else
#include "driver/i2c.h"
#endif
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// SHT85 I2C配置
#define SHT85_I2C_ADDRESS_LOW       0x44  // ADDR引脚接地
#define SHT85_I2C_ADDRESS_HIGH      0x45  // ADDR引脚接VDD
#define SHT85_I2C_FREQ_HZ           100000  // 100kHz

// 命令定义 (高重复性测量)
#define SHT85_CMD_MEAS_HIGH_CS      0x2C06  // 时钟拉伸模式
#define SHT85_CMD_MEAS_HIGH_NOCS    0x2400  // 无时钟拉伸模式
#define SHT85_CMD_MEAS_MED_NOCS     0x240B  // 中等重复性
#define SHT85_CMD_MEAS_LOW_NOCS     0x2416  // 低重复性

// 其他命令
#define SHT85_CMD_READ_STATUS       0xF32D  // 读取状态寄存器
#define SHT85_CMD_CLEAR_STATUS      0x3041  // 清除状态寄存器
#define SHT85_CMD_SOFT_RESET        0x30A2  // 软复位
#define SHT85_CMD_HEATER_ENABLE     0x306D  // 使能加热器
#define SHT85_CMD_HEATER_DISABLE    0x3066  // 禁用加热器
#define SHT85_CMD_GET_SERIAL        0x3780  // 获取序列号

/**
 * @brief SHT85 配置结构体
 */
typedef struct {
    i2c_port_t i2c_num;         // I2C端口号
    uint8_t i2c_addr;           // I2C地址 (0x44 或 0x45)
    int sda_pin;                // SDA引脚
    int scl_pin;                // SCL引脚
    uint32_t i2c_freq_hz;       // I2C频率
} sht85_config_t;

/**
 * @brief SHT85 数据结构体
 */
typedef struct {
    float temperature;          // 温度 (°C)
    float humidity;             // 相对湿度 (%)
    bool valid;                 // 数据有效标志
} sht85_data_t;

/**
 * @brief 测量重复性级别
 */
typedef enum {
    SHT85_REPEATABILITY_HIGH,   // 高重复性 (最高精度)
    SHT85_REPEATABILITY_MEDIUM, // 中等重复性
    SHT85_REPEATABILITY_LOW     // 低重复性 (最快速度)
} sht85_repeatability_t;

/**
 * @brief 初始化SHT85传感器
 *
 * @param config 配置参数
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sht85_init(const sht85_config_t *config);

/**
 * @brief 读取温湿度数据
 *
 * @param repeatability 测量重复性级别
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sht85_read_measurement(sht85_repeatability_t repeatability, sht85_data_t *data);

/**
 * @brief 软复位传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sht85_soft_reset(void);

/**
 * @brief 读取状态寄存器
 *
 * @param status 输出状态值指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sht85_read_status(uint16_t *status);

/**
 * @brief 清除状态寄存器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sht85_clear_status(void);

/**
 * @brief 使能/禁用内部加热器
 *
 * @param enable true=使能, false=禁用
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sht85_set_heater(bool enable);

/**
 * @brief 获取传感器序列号
 *
 * @param serial 32位序列号输出缓冲区
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sht85_get_serial_number(uint32_t *serial);

/**
 * @brief 反初始化SHT85传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sht85_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SHT85_H
