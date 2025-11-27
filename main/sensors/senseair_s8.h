/**
 * @file senseair_s8.h
 * @brief SenseAir S8 CO2传感器驱动 (UART接口)
 *
 * SenseAir S8是一款低功耗NDIR CO2传感器
 * 通信协议: Modbus RTU
 * 测量范围: 0-2000 ppm (标准) / 0-10000 ppm (可选)
 */

#ifndef SENSEAIR_S8_H
#define SENSEAIR_S8_H

#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// SenseAir S8 默认配置
#define SENSEAIR_S8_UART_NUM        UART_NUM_1
#define SENSEAIR_S8_BAUD_RATE       9600
#define SENSEAIR_S8_DEFAULT_ADDR    0xFE  // 默认Modbus地址

// 寄存器地址
#define S8_REG_CO2_HIGH             0x03  // CO2读数高字节
#define S8_REG_CO2_LOW              0x04  // CO2读数低字节
#define S8_REG_STATUS               0x01  // 状态寄存器

/**
 * @brief SenseAir S8 配置结构体
 */
typedef struct {
    uart_port_t uart_num;       // UART端口号
    int tx_pin;                 // TX引脚
    int rx_pin;                 // RX引脚
    uint32_t baud_rate;         // 波特率
    uint8_t modbus_addr;        // Modbus地址
} senseair_s8_config_t;

/**
 * @brief SenseAir S8 数据结构体
 */
typedef struct {
    uint16_t co2_ppm;           // CO2浓度 (ppm)
    uint8_t status;             // 传感器状态
    bool valid;                 // 数据有效标志
} senseair_s8_data_t;

/**
 * @brief 初始化SenseAir S8传感器
 *
 * @param config 配置参数
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t senseair_s8_init(const senseair_s8_config_t *config);

/**
 * @brief 读取CO2浓度
 *
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t senseair_s8_read_co2(senseair_s8_data_t *data);

/**
 * @brief 执行ABC自动校准
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t senseair_s8_calibrate_abc(void);

/**
 * @brief 反初始化SenseAir S8传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t senseair_s8_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SENSEAIR_S8_H
