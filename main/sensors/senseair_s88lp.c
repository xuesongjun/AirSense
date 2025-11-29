/**
 * @file senseair_s88lp.c
 * @brief SenseAir S88LP CO2传感器驱动实现
 *
 * 使用Modbus RTU协议通信
 */

#include "senseair_s88lp.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "S88LP";

#define UART_BUF_SIZE           256
#define MODBUS_TIMEOUT_MS       1000

static uart_port_t s_uart_num = UART_NUM_1;
static uint8_t s_modbus_addr = S88LP_DEFAULT_ADDR;
static bool s_initialized = false;

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
 * @brief 写入单个Holding Register (功能码0x06)
 */
static esp_err_t modbus_write_register(uint16_t addr, uint16_t value) {
    uint8_t cmd[8];
    cmd[0] = s_modbus_addr;         // 设备地址
    cmd[1] = 0x06;                  // 功能码 (0x06: Write Single Register)
    cmd[2] = (addr >> 8) & 0xFF;    // 寄存器地址高字节
    cmd[3] = addr & 0xFF;           // 寄存器地址低字节
    cmd[4] = (value >> 8) & 0xFF;   // 寄存器值高字节
    cmd[5] = value & 0xFF;          // 寄存器值低字节

    uint16_t crc = modbus_crc16(cmd, 6);
    cmd[6] = crc & 0xFF;            // CRC低字节
    cmd[7] = (crc >> 8) & 0xFF;     // CRC高字节

    // 清空接收缓冲区
    uart_flush_input(s_uart_num);

    // 发送命令
    int sent = uart_write_bytes(s_uart_num, cmd, 8);
    if (sent != 8) {
        ESP_LOGE(TAG, "Failed to send Modbus write command");
        return ESP_FAIL;
    }

    // 等待传感器响应
    vTaskDelay(pdMS_TO_TICKS(100));

    // 读取响应 (回显请求)
    uint8_t response[UART_BUF_SIZE];
    int read_len = uart_read_bytes(s_uart_num, response, UART_BUF_SIZE,
                                    pdMS_TO_TICKS(MODBUS_TIMEOUT_MS));
    if (read_len < 0) {
        ESP_LOGE(TAG, "UART read error");
        return ESP_FAIL;
    }

    if (read_len < 8) {
        ESP_LOGE(TAG, "Write response too short: %d bytes", read_len);
        return ESP_ERR_TIMEOUT;
    }

    // 验证CRC
    uint16_t recv_crc = response[read_len - 2] | (response[read_len - 1] << 8);
    uint16_t calc_crc = modbus_crc16(response, read_len - 2);

    if (recv_crc != calc_crc) {
        ESP_LOGE(TAG, "Write CRC error: recv=0x%04X, calc=0x%04X", recv_crc, calc_crc);
        return ESP_ERR_INVALID_CRC;
    }

    // 检查是否为错误响应
    if (response[1] & 0x80) {
        ESP_LOGE(TAG, "Modbus write error response: 0x%02X", response[2]);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief 发送Modbus RTU命令并读取响应
 */
static esp_err_t modbus_read_registers(uint8_t func, uint16_t addr, uint16_t count,
                                        uint8_t *response, size_t *resp_len) {
    uint8_t cmd[8];
    cmd[0] = s_modbus_addr;         // 设备地址
    cmd[1] = func;                  // 功能码 (0x04: Read Input Registers)
    cmd[2] = (addr >> 8) & 0xFF;    // 起始地址高字节
    cmd[3] = addr & 0xFF;           // 起始地址低字节
    cmd[4] = (count >> 8) & 0xFF;   // 寄存器数量高字节
    cmd[5] = count & 0xFF;          // 寄存器数量低字节

    uint16_t crc = modbus_crc16(cmd, 6);
    cmd[6] = crc & 0xFF;            // CRC低字节
    cmd[7] = (crc >> 8) & 0xFF;     // CRC高字节

    // 清空接收缓冲区
    uart_flush_input(s_uart_num);

    // 发送命令
    int sent = uart_write_bytes(s_uart_num, cmd, 8);
    if (sent != 8) {
        ESP_LOGE(TAG, "Failed to send Modbus command");
        return ESP_FAIL;
    }

    // 等待传感器响应
    vTaskDelay(pdMS_TO_TICKS(100));

    // 读取响应
    int read_len = uart_read_bytes(s_uart_num, response, UART_BUF_SIZE,
                                    pdMS_TO_TICKS(MODBUS_TIMEOUT_MS));
    if (read_len < 0) {
        ESP_LOGE(TAG, "UART read error");
        return ESP_FAIL;
    }

    *resp_len = read_len;

    if (read_len < 5) {
        ESP_LOGE(TAG, "Response too short: %d bytes", read_len);
        return ESP_ERR_TIMEOUT;
    }

    // 验证CRC
    uint16_t recv_crc = response[read_len - 2] | (response[read_len - 1] << 8);
    uint16_t calc_crc = modbus_crc16(response, read_len - 2);

    if (recv_crc != calc_crc) {
        ESP_LOGE(TAG, "CRC error: recv=0x%04X, calc=0x%04X", recv_crc, calc_crc);
        return ESP_ERR_INVALID_CRC;
    }

    // 检查是否为错误响应
    if (response[1] & 0x80) {
        ESP_LOGE(TAG, "Modbus error response: 0x%02X", response[2]);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t s88lp_init(const s88lp_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized, deinitializing first");
        s88lp_deinit();
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

    esp_err_t ret = uart_driver_install(s_uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(s_uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        uart_driver_delete(s_uart_num);
        return ret;
    }

    ret = uart_set_pin(s_uart_num, config->tx_pin, config->rx_pin,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(s_uart_num);
        return ret;
    }

    s_initialized = true;

    ESP_LOGI(TAG, "S88LP initialized on UART%d (TXD:%d, RXD:%d, Baud:%lu, Addr:0x%02X)",
             s_uart_num, config->tx_pin, config->rx_pin,
             (unsigned long)config->baud_rate, config->modbus_addr);

    return ESP_OK;
}

esp_err_t s88lp_read_co2(s88lp_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "S88LP not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    data->valid = false;

    uint8_t response[UART_BUF_SIZE];
    size_t resp_len = 0;

    // 读取CO2寄存器 (功能码0x04: Read Input Registers)
    // 起始地址0x0003, 读取1个寄存器 (2字节)
    esp_err_t ret = modbus_read_registers(0x04, S88LP_REG_CO2, 0x0001, response, &resp_len);

    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to read CO2: %s", esp_err_to_name(ret));
        return ret;
    }

    // 解析响应
    // 响应格式: [地址][功能码][字节数][数据高字节][数据低字节][CRC低][CRC高]
    if (resp_len >= 7 && response[0] == s_modbus_addr && response[1] == 0x04) {
        uint8_t byte_count = response[2];
        if (byte_count >= 2) {
            data->co2_ppm = (response[3] << 8) | response[4];
            data->status = 0;  // S88LP可能没有状态字节
            data->valid = true;

            ESP_LOGD(TAG, "CO2: %d ppm", data->co2_ppm);
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "Invalid response format (len=%d)", resp_len);
    return ESP_FAIL;
}

esp_err_t s88lp_set_abc(bool enable) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "S88LP not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // ABC Period: 0 = 禁用, 180 = 7.5天 (默认值)
    // 根据数据手册，ABC Period 以小时为单位存储在 HR32 (地址0x001F)
    uint16_t abc_period = enable ? 180 : 0;  // 180小时 = 7.5天

    esp_err_t ret = modbus_write_register(S88LP_HREG_ABC_PERIOD, abc_period);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ABC %s (period: %d hours)", enable ? "enabled" : "disabled", abc_period);
    } else {
        ESP_LOGE(TAG, "Failed to set ABC: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t s88lp_calibrate_zero(void) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "S88LP not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 根据数据手册，背景校准流程：
    // 1. 清除确认寄存器 HR1 (地址0x0000) = 0
    // 2. 写入命令 0x7C06 到 HR2 (地址0x0001) - 背景校准命令
    // 3. 等待至少2秒
    // 4. 读取 HR1 确认校准是否成功

    esp_err_t ret;

    // 步骤1: 清除确认寄存器
    ret = modbus_write_register(S88LP_HREG_ACK, 0x0000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear ACK register");
        return ret;
    }

    // 步骤2: 发送背景校准命令 (0x7C 命令, 参数 0x06 = 背景校准)
    ret = modbus_write_register(S88LP_HREG_COMMAND, 0x7C06);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send calibration command");
        return ret;
    }

    // 步骤3: 等待校准完成
    ESP_LOGI(TAG, "Zero calibration in progress, waiting 2.5s...");
    vTaskDelay(pdMS_TO_TICKS(2500));

    // 步骤4: 检查确认寄存器 (可选，需要读取 HR1 检查 bit6)
    ESP_LOGI(TAG, "Zero calibration command sent successfully");
    ESP_LOGI(TAG, "Note: Ensure sensor is in fresh air (~400ppm) for accurate calibration");

    return ESP_OK;
}

esp_err_t s88lp_deinit(void) {
    if (!s_initialized) {
        return ESP_OK;
    }

    uart_driver_delete(s_uart_num);
    s_initialized = false;

    ESP_LOGI(TAG, "S88LP deinitialized");
    return ESP_OK;
}
