/**
 * @file prosense_wzh3n.h
 * @brief Prosense WZ-H3-N 甲醛传感器驱动 (LP_UART硬件串口)
 *
 * WZ-H3-N是深圳普晟(Prosense)自主研发的固态电解质甲醛(HCHO)传感器
 * 特点: 抗酒精干扰、寿命长、数据稳定
 * 通信协议: UART 9600 8N1 (使用LP_UART硬件串口,主动上传模式或问答模式)
 * 测量范围: 0-1 ppm HCHO (最大过载5ppm)
 *
 * 使用ESP32-C5的LP_UART硬件串口 (GPIO4/GPIO5)
 */

#ifndef PROSENSE_WZH3N_H
#define PROSENSE_WZH3N_H

#include <stdint.h>
#if defined(__has_include)
#if __has_include("hal/gpio_types.h")
#include "hal/gpio_types.h"
#elif __has_include("driver/gpio_types.h")
#include "driver/gpio_types.h"
#else
#include "driver/gpio.h"
#endif
#else
#include "driver/gpio.h"
#endif
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Prosense WZ-H3-N 默认配置
#define PROSENSE_WZH3N_BAUD_RATE        9600

// 命令定义 (问答模式)
#define PROSENSE_CMD_READ_CONCENTRATION 0x86  // 读取浓度
#define PROSENSE_CMD_READ_FULL_RANGE    0x87  // 读取满量程
#define PROSENSE_CMD_SWITCH_QA          0x78  // 切换到问答模式 (参数0x41)
#define PROSENSE_CMD_SWITCH_ACTIVE      0x78  // 切换到主动上传 (参数0x40)

// 气体标识
#define PROSENSE_GAS_HCHO               0x11  // WZ-H3-N实际返回的HCHO标识
#define PROSENSE_GAS_CH2O               0x17  // 数据手册标注的CH2O标识

/**
 * @brief Prosense WZ-H3-N 配置结构体
 */
typedef struct {
    gpio_num_t tx_pin;          // TX引脚 (LP_UART: GPIO5)
    gpio_num_t rx_pin;          // RX引脚 (LP_UART: GPIO4)
    uint32_t baud_rate;         // 波特率 (默认9600)
    bool active_upload;         // true: 主动上传模式, false: 问答模式
} prosense_wzh3n_config_t;

/**
 * @brief Prosense WZ-H3-N 数据结构体
 */
typedef struct {
    uint16_t hcho_ugm3;         // HCHO甲醛浓度 (ug/m³) - 问答模式
    uint16_t hcho_ppb;          // HCHO甲醛浓度 (ppb) - 主动上传/问答模式
    float hcho_ppm;             // HCHO甲醛浓度 (ppm) - 由ppb转换
    float hcho_mgm3;            // HCHO甲醛浓度 (mg/m³) - 由ug/m³转换
    uint16_t full_range;        // 满量程值 (ppb) - 主动上传模式
    uint8_t gas_name;           // 气体名称 (0x11=HCHO) - 主动上传模式
    uint8_t unit;               // 单位 (0x04=ppb, 0x10=ug/m³) - 主动上传模式
    uint8_t decimal_places;     // 小数位数 - 主动上传模式
    bool valid;                 // 数据有效标志
} prosense_wzh3n_data_t;

/**
 * @brief 初始化Prosense WZ-H3-N传感器
 *
 * @param config 配置参数
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t prosense_wzh3n_init(const prosense_wzh3n_config_t *config);

/**
 * @brief 读取HCHO浓度 (问答模式)
 *
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t prosense_wzh3n_read_concentration(prosense_wzh3n_data_t *data);

/**
 * @brief 主动上传模式读取数据
 *
 * @param data 输出数据结构体指针
 * @param timeout_ms 超时时间(ms)
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t prosense_wzh3n_read_active(prosense_wzh3n_data_t *data, uint32_t timeout_ms);

/**
 * @brief 反初始化Prosense WZ-H3-N传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t prosense_wzh3n_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // PROSENSE_WZH3N_H
