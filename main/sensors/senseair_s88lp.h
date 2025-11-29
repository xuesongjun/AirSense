/**
 * @file senseair_s88lp.h
 * @brief SenseAir S88LP CO2传感器驱动 (UART Modbus接口)
 *
 * SenseAir S88LP是一款低功耗NDIR红外CO2传感器
 * 通信协议: Modbus RTU
 * 测量范围: 400-2000 ppm (标准) / 400-10000 ppm (扩展)
 *
 * 注: S88LP与S8系列使用相同的Modbus协议，驱动兼容
 */

#ifndef SENSEAIR_S88LP_H
#define SENSEAIR_S88LP_H

#include <stdint.h>
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// SenseAir S88LP 默认配置
#define S88LP_UART_NUM          UART_NUM_1
#define S88LP_BAUD_RATE         9600
#define S88LP_DEFAULT_ADDR      0xFE  // 默认Modbus地址 (广播地址)

// Modbus寄存器地址 (Input Registers, 功能码0x04)
#define S88LP_REG_STATUS        0x0000  // IR1: MeterStatus 状态寄存器
#define S88LP_REG_ALARM_STATUS  0x0001  // IR2: AlarmStatus
#define S88LP_REG_OUTPUT_STATUS 0x0002  // IR3: OutputStatus
#define S88LP_REG_CO2           0x0003  // IR4: Space CO2 浓度寄存器 (ppm)
#define S88LP_REG_TEMP          0x0004  // IR5: Space Temp 传感器温度

// Modbus寄存器地址 (Holding Registers, 功能码0x03/0x06)
#define S88LP_HREG_ACK          0x0000  // HR1: Acknowledgement register
#define S88LP_HREG_COMMAND      0x0001  // HR2: Special Command Register
#define S88LP_HREG_CAL_TARGET   0x0002  // HR3: Calibration Target
#define S88LP_HREG_ABC_PERIOD   0x001F  // HR32: ABC Period (hours), 0=disable

/**
 * @brief SenseAir S88LP 配置结构体
 */
typedef struct {
    uart_port_t uart_num;       // UART端口号
    int tx_pin;                 // TX引脚 (ESP32 -> S88LP RXD)
    int rx_pin;                 // RX引脚 (ESP32 <- S88LP TXD)
    uint32_t baud_rate;         // 波特率 (默认9600)
    uint8_t modbus_addr;        // Modbus地址 (默认0xFE)
} s88lp_config_t;

/**
 * @brief SenseAir S88LP 数据结构体
 */
typedef struct {
    uint16_t co2_ppm;           // CO2浓度 (ppm)
    uint8_t status;             // 传感器状态
    bool valid;                 // 数据有效标志
} s88lp_data_t;

/**
 * @brief 初始化SenseAir S88LP传感器
 *
 * @param config 配置参数
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t s88lp_init(const s88lp_config_t *config);

/**
 * @brief 读取CO2浓度
 *
 * @param data 输出数据结构体指针
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t s88lp_read_co2(s88lp_data_t *data);

/**
 * @brief 执行ABC自动背景校准
 *
 * @param enable true启用ABC, false禁用ABC
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t s88lp_set_abc(bool enable);

/**
 * @brief 执行手动校准 (在400ppm新鲜空气中)
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t s88lp_calibrate_zero(void);

/**
 * @brief 反初始化SenseAir S88LP传感器
 *
 * @return esp_err_t ESP_OK成功，其他失败
 */
esp_err_t s88lp_deinit(void);

// 兼容旧API (senseair_s8.h)
typedef s88lp_config_t senseair_s8_config_t;
typedef s88lp_data_t senseair_s8_data_t;
#define SENSEAIR_S8_DEFAULT_ADDR S88LP_DEFAULT_ADDR
#define senseair_s8_init(cfg) s88lp_init(cfg)
#define senseair_s8_read_co2(data) s88lp_read_co2(data)
#define senseair_s8_deinit() s88lp_deinit()

#ifdef __cplusplus
}
#endif

#endif // SENSEAIR_S88LP_H
