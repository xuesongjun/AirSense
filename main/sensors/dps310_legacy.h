/**
 * @file dps310_legacy.h
 * @brief Infineon DPS310 气压传感器驱动 (使用旧版 I2C API)
 *
 * DPS310 是 Infineon 的高精度气压和温度传感器
 * I2C地址: 0x77 (SDO接VDD) 或 0x76 (SDO接GND)
 * 测量范围: 气压 300-1200 hPa, 温度 -40~85°C
 * 精度: ±0.002 hPa (128x过采样)
 */

#ifndef DPS310_LEGACY_H
#define DPS310_LEGACY_H

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

// DPS310 I2C地址
#define DPS310_I2C_ADDR_0       0x76  // SDO接GND
#define DPS310_I2C_ADDR_1       0x77  // SDO接VDD

// DPS310 寄存器地址
#define DPS310_REG_PSR_B2       0x00  // 气压数据 MSB
#define DPS310_REG_PSR_B1       0x01
#define DPS310_REG_PSR_B0       0x02  // 气压数据 LSB
#define DPS310_REG_TMP_B2       0x03  // 温度数据 MSB
#define DPS310_REG_TMP_B1       0x04
#define DPS310_REG_TMP_B0       0x05  // 温度数据 LSB
#define DPS310_REG_PRS_CFG      0x06  // 气压配置
#define DPS310_REG_TMP_CFG      0x07  // 温度配置
#define DPS310_REG_MEAS_CFG     0x08  // 测量配置
#define DPS310_REG_CFG_REG      0x09  // 其他配置
#define DPS310_REG_RESET        0x0C  // 复位
#define DPS310_REG_PRODUCT_ID   0x0D  // 产品ID
#define DPS310_REG_COEF_BASE    0x10  // 校准系数起始地址
#define DPS310_REG_COEF_SRCE    0x28  // 温度系数源寄存器

// 产品ID
#define DPS310_PRODUCT_ID       0x10

// 过采样率
typedef enum {
    DPS310_OVERSAMPLE_1 = 0,   // 1次, 测量时间 3.6ms
    DPS310_OVERSAMPLE_2,       // 2次, 测量时间 5.2ms
    DPS310_OVERSAMPLE_4,       // 4次, 测量时间 8.4ms
    DPS310_OVERSAMPLE_8,       // 8次, 测量时间 14.8ms
    DPS310_OVERSAMPLE_16,      // 16次, 测量时间 27.6ms
    DPS310_OVERSAMPLE_32,      // 32次, 测量时间 53.2ms
    DPS310_OVERSAMPLE_64,      // 64次, 测量时间 104.4ms
    DPS310_OVERSAMPLE_128      // 128次, 测量时间 206.8ms (最高精度)
} dps310_oversample_t;

// 测量速率
typedef enum {
    DPS310_RATE_1 = 0,         // 1 次/秒
    DPS310_RATE_2,             // 2 次/秒
    DPS310_RATE_4,             // 4 次/秒
    DPS310_RATE_8,             // 8 次/秒
    DPS310_RATE_16,            // 16 次/秒
    DPS310_RATE_32,            // 32 次/秒
    DPS310_RATE_64,            // 64 次/秒
    DPS310_RATE_128            // 128 次/秒
} dps310_rate_t;

// 温度源
typedef enum {
    DPS310_TMP_SRC_INTERNAL = 0,  // 内部 ASIC 温度
    DPS310_TMP_SRC_EXTERNAL = 1   // 外部 MEMS 温度 (推荐)
} dps310_temp_src_t;

// 测量模式
typedef enum {
    DPS310_MODE_IDLE = 0,              // 空闲模式
    DPS310_MODE_ONE_PRESSURE = 1,      // 单次气压测量
    DPS310_MODE_ONE_TEMPERATURE = 2,   // 单次温度测量
    DPS310_MODE_CONTINUOUS_PRESSURE = 5,    // 连续气压测量
    DPS310_MODE_CONTINUOUS_TEMPERATURE = 6, // 连续温度测量
    DPS310_MODE_CONTINUOUS_BOTH = 7         // 连续气压+温度测量
} dps310_mode_t;

/**
 * @brief DPS310 配置结构体
 */
typedef struct {
    i2c_port_t i2c_num;              // I2C端口号
    uint8_t i2c_addr;                // I2C地址 (0x76 或 0x77)
    int sda_pin;                     // SDA引脚
    int scl_pin;                     // SCL引脚
    uint32_t i2c_freq_hz;            // I2C频率
    dps310_oversample_t prs_osr;     // 气压过采样率
    dps310_oversample_t tmp_osr;     // 温度过采样率
    dps310_rate_t prs_rate;          // 气压测量速率
    dps310_rate_t tmp_rate;          // 温度测量速率
    dps310_temp_src_t tmp_src;       // 温度源
} dps310_config_t;

/**
 * @brief DPS310 数据结构体
 */
typedef struct {
    float pressure;      // 气压 (Pa)
    float temperature;   // 温度 (°C)
    float altitude;      // 海拔高度 (m, 基于标准大气压计算)
    bool valid;          // 数据有效标志
} dps310_data_t;

/**
 * @brief 初始化DPS310传感器
 *
 * @param config 配置参数
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dps310_legacy_init(const dps310_config_t *config);

/**
 * @brief 启动连续测量模式
 *
 * @param mode 测量模式
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dps310_legacy_start_continuous(dps310_mode_t mode);

/**
 * @brief 停止测量进入空闲模式
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dps310_legacy_stop(void);

/**
 * @brief 读取温度和气压数据
 *
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dps310_legacy_read_data(dps310_data_t *data);

/**
 * @brief 设置海拔参考值（用于相对高度测量）
 *
 * @param altitude_m 当前海拔高度(米)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dps310_legacy_set_altitude_ref(float altitude_m);

/**
 * @brief 软复位传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dps310_legacy_soft_reset(void);

/**
 * @brief 反初始化DPS310传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dps310_legacy_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // DPS310_LEGACY_H
