/**
 * @file dart_wzh3n.h
 * @brief Dart WZ-H3-N 电化学传感器驱动 (UART接口)
 *
 * Dart WZ-H3-N是氨气(NH3)电化学传感器
 * 通信协议: UART (主动上传模式或问答模式)
 * 测量范围: 0-100 ppm NH3
 */

#ifndef DART_WZH3N_H
#define DART_WZH3N_H

#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Dart WZ-H3-N 默认配置
#define DART_WZH3N_UART_NUM         UART_NUM_1  // ESP32-C5 only has UART0/1
#define DART_WZH3N_BAUD_RATE        9600

// 命令定义 (问答模式)
#define DART_CMD_READ_CONCENTRATION 0x86  // 读取浓度
#define DART_CMD_READ_FULL_RANGE    0x87  // 读取满量程

/**
 * @brief Dart WZ-H3-N 配置结构体
 */
typedef struct {
    uart_port_t uart_num;       // UART端口号
    int tx_pin;                 // TX引脚
    int rx_pin;                 // RX引脚
    uint32_t baud_rate;         // 波特率
    bool active_upload;         // true: 主动上传模式, false: 问答模式
} dart_wzh3n_config_t;

/**
 * @brief Dart WZ-H3-N 数据结构体
 */
typedef struct {
    float nh3_ppm;              // NH3浓度 (ppm)
    uint16_t full_range;        // 满量程值 (ppm)
    uint8_t unit;               // 单位 (1=ppm, 2=ppb, 3=mg/m³, etc.)
    int16_t decimal_places;     // 小数位数
    bool valid;                 // 数据有效标志
} dart_wzh3n_data_t;

/**
 * @brief 初始化Dart WZ-H3-N传感器
 *
 * @param config 配置参数
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dart_wzh3n_init(const dart_wzh3n_config_t *config);

/**
 * @brief 读取NH3浓度 (问答模式)
 *
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dart_wzh3n_read_concentration(dart_wzh3n_data_t *data);

/**
 * @brief 主动上传模式读取数据
 *
 * @param data 输出数据结构体指针
 * @param timeout_ms 超时时间(ms)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dart_wzh3n_read_active(dart_wzh3n_data_t *data, uint32_t timeout_ms);

/**
 * @brief 反初始化Dart WZ-H3-N传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t dart_wzh3n_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // DART_WZH3N_H
