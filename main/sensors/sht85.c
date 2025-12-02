/**
 * @file sht85.c
 * @brief Sensirion SHT85 温湿度传感器驱动实现
 */

#ifndef CONFIG_I2C_SUPPRESS_DEPRECATE_WARN
#define CONFIG_I2C_SUPPRESS_DEPRECATE_WARN 1
#endif
#include "driver/i2c.h"
#include "sht85.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SHT85";

static i2c_port_t s_i2c_num = I2C_NUM_0;
static uint8_t s_i2c_addr = SHT85_I2C_ADDRESS_LOW;

/**
 * @brief 计算Sensirion CRC8
 */
static uint8_t sensirion_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 写命令到SHT85
 */
static esp_err_t sht85_write_command(uint16_t command) {
    uint8_t buffer[2];
    buffer[0] = (command >> 8) & 0xFF;
    buffer[1] = command & 0xFF;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buffer, 2, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write command 0x%04X failed: %s", command, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 从SHT85读取数据
 */
static esp_err_t sht85_read_data(uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_i2c_addr << 1) | I2C_MASTER_READ, true);

    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t sht85_init(const sht85_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_num = config->i2c_num;
    s_i2c_addr = config->i2c_addr;

    // 注意: I2C总线已经由main.c初始化,不需要重复初始化
    ESP_LOGI(TAG, "SHT85 initialized on I2C%d (Addr:0x%02X, using existing bus)",
             s_i2c_num, s_i2c_addr);

    // 等待传感器上电启动
    vTaskDelay(pdMS_TO_TICKS(50));

    // 发送软复位,确保传感器处于已知状态
    esp_err_t ret = sht85_write_command(SHT85_CMD_SOFT_RESET);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Soft reset sent successfully");
        vTaskDelay(pdMS_TO_TICKS(20));  // 复位后等待(数据手册要求最多1.5ms,这里用20ms确保稳定)
    } else {
        ESP_LOGW(TAG, "Soft reset failed: %s, continuing anyway", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "SHT85 basic initialization complete");
    return ESP_OK;
}

esp_err_t sht85_read_measurement(sht85_repeatability_t repeatability, sht85_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    data->valid = false;

    // 选择测量命令
    uint16_t command;
    uint16_t delay_ms;

    switch (repeatability) {
        case SHT85_REPEATABILITY_HIGH:
            command = SHT85_CMD_MEAS_HIGH_NOCS;
            delay_ms = 20;  // 高重复性需要15ms,增加余量到20ms
            break;
        case SHT85_REPEATABILITY_MEDIUM:
            command = SHT85_CMD_MEAS_MED_NOCS;
            delay_ms = 10;  // 中等重复性需要6ms,增加余量到10ms
            break;
        case SHT85_REPEATABILITY_LOW:
            command = SHT85_CMD_MEAS_LOW_NOCS;
            delay_ms = 8;   // 低重复性需要4ms,增加余量到8ms
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    // 发送测量命令
    esp_err_t ret = sht85_write_command(command);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command 0x%04X: %s", command, esp_err_to_name(ret));
        return ret;
    }

    // 等待测量完成
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    // 读取6字节: Temp(2B+CRC) + RH(2B+CRC)
    uint8_t response[6];
    ret = sht85_read_data(response, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data: %s", esp_err_to_name(ret));
        return ret;
    }

    // 验证CRC
    uint8_t crc_temp = sensirion_crc8(&response[0], 2);
    uint8_t crc_rh = sensirion_crc8(&response[3], 2);

    if (crc_temp != response[2] || crc_rh != response[5]) {
        ESP_LOGE(TAG, "Measurement CRC error");
        return ESP_FAIL;
    }

    // 解析温度 (°C) = -45 + 175 * (value / 65535)
    uint16_t temp_raw = (response[0] << 8) | response[1];
    data->temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);

    // 解析湿度 (%) = 100 * (value / 65535)
    uint16_t rh_raw = (response[3] << 8) | response[4];
    data->humidity = 100.0f * ((float)rh_raw / 65535.0f);

    // 限制湿度范围 0-100%
    if (data->humidity < 0.0f) data->humidity = 0.0f;
    if (data->humidity > 100.0f) data->humidity = 100.0f;

    data->valid = true;

    ESP_LOGD(TAG, "Temp: %.2f°C, RH: %.2f%%", data->temperature, data->humidity);

    return ESP_OK;
}

esp_err_t sht85_soft_reset(void) {
    esp_err_t ret = sht85_write_command(SHT85_CMD_SOFT_RESET);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Soft reset executed");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ret;
}

esp_err_t sht85_read_status(uint16_t *status) {
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = sht85_write_command(SHT85_CMD_READ_STATUS);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t response[3];  // 2字节状态 + 1字节CRC
    ret = sht85_read_data(response, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    // 验证CRC
    uint8_t crc = sensirion_crc8(response, 2);
    if (crc != response[2]) {
        ESP_LOGE(TAG, "Status CRC error");
        return ESP_FAIL;
    }

    *status = (response[0] << 8) | response[1];

    return ESP_OK;
}

esp_err_t sht85_clear_status(void) {
    return sht85_write_command(SHT85_CMD_CLEAR_STATUS);
}

esp_err_t sht85_set_heater(bool enable) {
    uint16_t command = enable ? SHT85_CMD_HEATER_ENABLE : SHT85_CMD_HEATER_DISABLE;
    esp_err_t ret = sht85_write_command(command);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Heater %s", enable ? "enabled" : "disabled");
    }
    return ret;
}

esp_err_t sht85_get_serial_number(uint32_t *serial) {
    if (serial == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = sht85_write_command(SHT85_CMD_GET_SERIAL);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t response[6];  // 4字节序列号 + 2字节CRC
    ret = sht85_read_data(response, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    // 验证CRC (两组数据,每组2字节+1字节CRC)
    uint8_t crc1 = sensirion_crc8(&response[0], 2);
    uint8_t crc2 = sensirion_crc8(&response[3], 2);

    if (crc1 != response[2] || crc2 != response[5]) {
        ESP_LOGE(TAG, "Serial number CRC error");
        return ESP_FAIL;
    }

    *serial = ((uint32_t)response[0] << 24) | ((uint32_t)response[1] << 16) |
              ((uint32_t)response[3] << 8) | response[4];

    ESP_LOGI(TAG, "Serial: 0x%08lX", (unsigned long)*serial);

    return ESP_OK;
}

esp_err_t sht85_deinit(void) {
    sht85_set_heater(false);
    ESP_LOGI(TAG, "SHT85 deinitialized");
    return ESP_OK;
}
