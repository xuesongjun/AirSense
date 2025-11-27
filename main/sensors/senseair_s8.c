/**
 * @file senseair_s8.c
 * @brief SenseAir S8 CO2传感器驱动实现
 */

#include "senseair_s8.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SenseAir_S8";

#define UART_BUF_SIZE           256
#define MODBUS_TIMEOUT_MS       1000

static uart_port_t s_uart_num = UART_NUM_1;
static uint8_t s_modbus_addr = SENSEAIR_S8_DEFAULT_ADDR;

/**
 * @brief 计算Modbus CRC16
 */
static uint16_t modbus_crc16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 发送Modbus RTU命令
 */
static esp_err_t modbus_send_command(uint8_t func, uint16_t addr, uint16_t len, uint8_t *response, size_t *resp_len) {
    uint8_t cmd[8];
    cmd[0] = s_modbus_addr;     // 设备地址
    cmd[1] = func;              // 功能码
    cmd[2] = (addr >> 8) & 0xFF;    // 起始地址高字节
    cmd[3] = addr & 0xFF;           // 起始地址低字节
    cmd[4] = (len >> 8) & 0xFF;     // 读取长度高字节
    cmd[5] = len & 0xFF;            // 读取长度低字节

    uint16_t crc = modbus_crc16(cmd, 6);
    cmd[6] = crc & 0xFF;        // CRC低字节
    cmd[7] = (crc >> 8) & 0xFF; // CRC高字节

    // 清空接收缓冲区
    uart_flush_input(s_uart_num);

    // 发送命令
    int sent = uart_write_bytes(s_uart_num, cmd, 8);
    if (sent != 8) {
        ESP_LOGE(TAG, "Failed to send Modbus command");
        return ESP_FAIL;
    }

    // 等待响应
    vTaskDelay(pdMS_TO_TICKS(100));

    // 读取响应
    int read_len = uart_read_bytes(s_uart_num, response, UART_BUF_SIZE, pdMS_TO_TICKS(MODBUS_TIMEOUT_MS));
    if (read_len < 0) {
        ESP_LOGE(TAG, "UART read error");
        return ESP_FAIL;
    }

    *resp_len = read_len;

    if (read_len < 5) {
        ESP_LOGE(TAG, "Response too short: %d bytes", read_len);
        return ESP_FAIL;
    }

    // 验证CRC
    uint16_t recv_crc = response[read_len - 2] | (response[read_len - 1] << 8);
    uint16_t calc_crc = modbus_crc16(response, read_len - 2);

    if (recv_crc != calc_crc) {
        ESP_LOGE(TAG, "CRC error: recv=0x%04X, calc=0x%04X", recv_crc, calc_crc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t senseair_s8_init(const senseair_s8_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_uart_num = config->uart_num;
    s_modbus_addr = config->modbus_addr;

    // 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(s_uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(s_uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(s_uart_num, config->tx_pin, config->rx_pin,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "SenseAir S8 initialized on UART%d (TX:%d, RX:%d, Baud:%d)",
             s_uart_num, config->tx_pin, config->rx_pin, config->baud_rate);

    return ESP_OK;
}

esp_err_t senseair_s8_read_co2(senseair_s8_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    data->valid = false;

    uint8_t response[UART_BUF_SIZE];
    size_t resp_len = 0;

    // 读取CO2寄存器 (功能码0x04: Read Input Registers)
    // 起始地址0x0003, 读取2个寄存器
    esp_err_t ret = modbus_send_command(0x04, 0x0003, 0x0002, response, &resp_len);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CO2 data");
        return ret;
    }

    // 解析响应
    // 响应格式: [地址][功能码][字节数][数据高字节][数据低字节][CRC低][CRC高]
    if (resp_len >= 7 && response[0] == s_modbus_addr && response[1] == 0x04) {
        data->co2_ppm = (response[3] << 8) | response[4];
        data->status = response[5];
        data->valid = true;

        ESP_LOGD(TAG, "CO2: %d ppm, Status: 0x%02X", data->co2_ppm, data->status);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Invalid response format");
    return ESP_FAIL;
}

esp_err_t senseair_s8_calibrate_abc(void) {
    // ABC校准通常需要发送特殊命令到特定寄存器
    // 这里是示例实现，具体命令需参考数据手册
    ESP_LOGI(TAG, "ABC calibration not implemented yet");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t senseair_s8_deinit(void) {
    uart_driver_delete(s_uart_num);
    ESP_LOGI(TAG, "SenseAir S8 deinitialized");
    return ESP_OK;
}
