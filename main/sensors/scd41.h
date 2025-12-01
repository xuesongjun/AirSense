/**
 * @file scd41.h
 * @brief Sensirion SCD41 CO2传感器驱动 (I2C接口)
 *
 * SCD41是Sensirion的光声CO2传感器
 * I2C地址: 0x62
 * 测量范围: 400-5000 ppm
 * 精度: ±(40ppm + 5%读数)
 */

#ifndef SCD41_H
#define SCD41_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// SCD41 I2C配置
#define SCD41_I2C_ADDRESS           0x62
#define SCD41_I2C_FREQ_HZ           100000  // 100kHz

// 命令定义
#define SCD41_CMD_START_PERIODIC    0x21B1  // 开始周期性测量
#define SCD41_CMD_READ_MEASUREMENT  0xEC05  // 读取测量数据
#define SCD41_CMD_STOP_PERIODIC     0x3F86  // 停止周期性测量
#define SCD41_CMD_GET_SERIAL        0x3682  // 获取序列号
#define SCD41_CMD_SELF_TEST         0x3639  // 执行自检
#define SCD41_CMD_FACTORY_RESET     0x3632  // 恢复出厂设置
#define SCD41_CMD_GET_DATA_READY    0xE4B8  // 获取数据就绪状态

// 校准相关命令
#define SCD41_CMD_SET_TEMP_OFFSET   0x241D  // 设置温度补偿
#define SCD41_CMD_GET_TEMP_OFFSET   0x2318  // 获取温度补偿
#define SCD41_CMD_SET_SENSOR_ALT    0x2427  // 设置海拔补偿
#define SCD41_CMD_GET_SENSOR_ALT    0x2322  // 获取海拔补偿
#define SCD41_CMD_SET_PRESSURE      0xE000  // 设置环境气压补偿
#define SCD41_CMD_FORCE_CALIBRATION 0x362F  // 强制校准

/**
 * @brief SCD41 配置结构体
 */
typedef struct {
    i2c_port_t i2c_num;         // I2C端口号
    int sda_pin;                // SDA引脚
    int scl_pin;                // SCL引脚
    uint32_t i2c_freq_hz;       // I2C频率
    uint16_t altitude_m;        // 海拔高度 (m)，用于气压补偿
    float temp_offset;          // 温度补偿偏移 (°C)
} scd41_config_t;

/**
 * @brief SCD41 数据结构体
 */
typedef struct {
    uint16_t co2_ppm;           // CO2浓度 (ppm)
    float temperature;          // 温度 (°C)
    float humidity;             // 相对湿度 (%)
    bool valid;                 // 数据有效标志
} scd41_data_t;

/**
 * @brief 初始化SCD41传感器
 *
 * @param config 配置参数
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_init(const scd41_config_t *config);

/**
 * @brief 开始周期性测量
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_start_periodic_measurement(void);

/**
 * @brief 停止周期性测量
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_stop_periodic_measurement(void);

/**
 * @brief 读取CO2、温度和湿度数据
 *
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_read_measurement(scd41_data_t *data);

/**
 * @brief 检查数据是否就绪
 *
 * @param ready 输出就绪状态指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_get_data_ready_status(bool *ready);

/**
 * @brief 执行自检
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_self_test(void);

/**
 * @brief 获取传感器序列号
 *
 * @param serial 48位序列号输出缓冲区
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_get_serial_number(uint8_t serial[6]);

/**
 * @brief 设置温度补偿偏移
 *
 * @param temp_offset 温度偏移 (°C)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_set_temperature_offset(float temp_offset);

/**
 * @brief 设置海拔补偿
 *
 * @param altitude_m 海拔高度 (m)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_set_sensor_altitude(uint16_t altitude_m);

/**
 * @brief 设置环境气压补偿 (动态补偿，测量期间可用)
 *
 * 注意: 此命令可在测量期间调用，用于实时气压补偿
 * 如果设置了气压，则海拔设置会被忽略
 *
 * @param pressure_hpa 环境气压 (hPa), 范围 700-1200
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_set_ambient_pressure(uint16_t pressure_hpa);

/**
 * @brief 强制校准CO2到指定浓度
 *
 * @param target_co2 目标CO2浓度 (ppm)，通常为400ppm
 * @param frc_correction 输出校准修正值
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_perform_forced_calibration(uint16_t target_co2, int16_t *frc_correction);

/**
 * @brief 恢复出厂设置
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_factory_reset(void);

/**
 * @brief 反初始化SCD41传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t scd41_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SCD41_H
