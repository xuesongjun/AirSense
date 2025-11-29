/**
 * @file sgp41.h
 * @brief Sensirion SGP41 VOC传感器驱动 (I2C接口)
 *
 * SGP41是Sensirion的VOC和NOx气体传感器
 * I2C地址: 0x59
 * 测量参数: VOC指数, NOx指数
 */

#ifndef SGP41_H
#define SGP41_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// SGP41 I2C配置
#define SGP41_I2C_ADDRESS           0x59
#define SGP41_I2C_FREQ_HZ           100000  // 100kHz

// 命令定义
#define SGP41_CMD_MEASURE_RAW       0x2619  // 测量原始信号
#define SGP41_CMD_EXECUTE_SELF_TEST 0x280E  // 自检
#define SGP41_CMD_HEATER_OFF        0x3615  // 关闭加热器
#define SGP41_CMD_GET_SERIAL        0x3682  // 获取序列号

/**
 * @brief SGP41 配置结构体
 */
typedef struct {
    i2c_port_t i2c_num;         // I2C端口号
    int sda_pin;                // SDA引脚
    int scl_pin;                // SCL引脚
    uint32_t i2c_freq_hz;       // I2C频率
} sgp41_config_t;

/**
 * @brief SGP41 数据结构体
 */
typedef struct {
    uint16_t sraw_voc;          // VOC原始信号
    uint16_t sraw_nox;          // NOx原始信号
    int32_t voc_index;          // VOC指数 (0-500)
    int32_t nox_index;          // NOx指数 (0-500)
    bool valid;                 // 数据有效标志
} sgp41_data_t;

/**
 * @brief 初始化SGP41传感器
 *
 * @param config 配置参数
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sgp41_init(const sgp41_config_t *config);

/**
 * @brief 读取VOC和NOx原始数据 (浮点温湿度补偿)
 *
 * @param rh_percent 相对湿度 (%)，用于补偿
 * @param temp_celsius 温度 (°C)，用于补偿
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sgp41_measure_raw_f(float rh_percent, float temp_celsius, sgp41_data_t *data);

/**
 * @brief 读取VOC和NOx原始数据 (整数温湿度补偿，兼容旧接口)
 *
 * @param rh_percent 相对湿度 (%)，用于补偿
 * @param temp_celsius 温度 (°C)，用于补偿
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sgp41_measure_raw(uint16_t rh_percent, uint16_t temp_celsius, sgp41_data_t *data);

/**
 * @brief 执行自检
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sgp41_self_test(void);

/**
 * @brief 获取传感器序列号
 *
 * @param serial 48位序列号输出缓冲区
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sgp41_get_serial_number(uint8_t serial[6]);

/**
 * @brief 关闭加热器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sgp41_heater_off(void);

/**
 * @brief 反初始化SGP41传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t sgp41_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SGP41_H
