/**
 * @file sgp41.c
 * @brief Sensirion SGP41 VOC传感器驱动实现
 */

#include "sgp41.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SGP41";

static i2c_port_t s_i2c_num = I2C_NUM_0;

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
 * @brief 写命令到SGP41
 */
static esp_err_t sgp41_write_command(uint16_t command, const uint8_t *params, uint8_t params_len) {
    uint8_t buffer[32];
    uint8_t idx = 0;

    // 命令高字节和低字节
    buffer[idx++] = (command >> 8) & 0xFF;
    buffer[idx++] = command & 0xFF;

    // 添加参数和CRC
    for (uint8_t i = 0; i < params_len; i += 2) {
        buffer[idx++] = params[i];
        buffer[idx++] = params[i + 1];
        buffer[idx++] = sensirion_crc8(&params[i], 2);
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP41_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buffer, idx, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 从SGP41读取数据
 */
static esp_err_t sgp41_read_data(uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SGP41_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);

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

esp_err_t sgp41_init(const sgp41_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_num = config->i2c_num;

    // 配置I2C参数
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config->i2c_freq_hz,
    };

    ESP_ERROR_CHECK(i2c_param_config(s_i2c_num, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(s_i2c_num, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "SGP41 initialized on I2C%d (SDA:%d, SCL:%d)",
             s_i2c_num, config->sda_pin, config->scl_pin);

    // 等待传感器启动
    vTaskDelay(pdMS_TO_TICKS(50));

    return ESP_OK;
}

esp_err_t sgp41_measure_raw(uint16_t rh_percent, uint16_t temp_celsius, sgp41_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    data->valid = false;

    // 转换温湿度为SGP41格式
    // RH: ticks = %RH * 65535 / 100
    // Temp: ticks = (°C + 45) * 65535 / 175
    uint16_t rh_ticks = (rh_percent * 65535) / 100;
    uint16_t temp_ticks = ((temp_celsius + 45) * 65535) / 175;

    // 准备参数 (湿度和温度)
    uint8_t params[4];
    params[0] = (rh_ticks >> 8) & 0xFF;
    params[1] = rh_ticks & 0xFF;
    params[2] = (temp_ticks >> 8) & 0xFF;
    params[3] = temp_ticks & 0xFF;

    // 发送测量命令
    esp_err_t ret = sgp41_write_command(SGP41_CMD_MEASURE_RAW, params, 4);
    if (ret != ESP_OK) {
        return ret;
    }

    // 等待测量完成 (50ms)
    vTaskDelay(pdMS_TO_TICKS(50));

    // 读取结果 (VOC: 2字节数据+1字节CRC, NOx: 2字节数据+1字节CRC)
    uint8_t response[6];
    ret = sgp41_read_data(response, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    // 验证CRC
    uint8_t crc_voc = sensirion_crc8(response, 2);
    uint8_t crc_nox = sensirion_crc8(&response[3], 2);

    if (crc_voc != response[2] || crc_nox != response[5]) {
        ESP_LOGE(TAG, "CRC error");
        return ESP_FAIL;
    }

    // 解析数据
    data->sraw_voc = (response[0] << 8) | response[1];
    data->sraw_nox = (response[3] << 8) | response[4];

    // VOC和NOx指数需要通过算法计算，这里简化处理
    // 实际应用中应使用Sensirion的VOC和NOx算法库
    data->voc_index = (data->sraw_voc * 500) / 65535;  // 简化映射到0-500
    data->nox_index = (data->sraw_nox * 500) / 65535;  // 简化映射到0-500

    data->valid = true;

    ESP_LOGD(TAG, "VOC raw: %d, NOx raw: %d", data->sraw_voc, data->sraw_nox);

    return ESP_OK;
}

esp_err_t sgp41_self_test(void) {
    esp_err_t ret = sgp41_write_command(SGP41_CMD_EXECUTE_SELF_TEST, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    // 等待自检完成 (320ms)
    vTaskDelay(pdMS_TO_TICKS(320));

    uint8_t response[3];
    ret = sgp41_read_data(response, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    // 验证CRC
    uint8_t crc = sensirion_crc8(response, 2);
    if (crc != response[2]) {
        ESP_LOGE(TAG, "Self-test CRC error");
        return ESP_FAIL;
    }

    uint16_t result = (response[0] << 8) | response[1];
    if (result != 0xD400) {
        ESP_LOGE(TAG, "Self-test failed: 0x%04X", result);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Self-test passed");
    return ESP_OK;
}

esp_err_t sgp41_get_serial_number(uint8_t serial[6]) {
    if (serial == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = sgp41_write_command(SGP41_CMD_GET_SERIAL, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t response[9];  // 6字节数据 + 3字节CRC
    ret = sgp41_read_data(response, 9);
    if (ret != ESP_OK) {
        return ret;
    }

    // 验证CRC
    for (int i = 0; i < 3; i++) {
        uint8_t crc = sensirion_crc8(&response[i * 3], 2);
        if (crc != response[i * 3 + 2]) {
            ESP_LOGE(TAG, "Serial number CRC error");
            return ESP_FAIL;
        }
        serial[i * 2] = response[i * 3];
        serial[i * 2 + 1] = response[i * 3 + 1];
    }

    ESP_LOGI(TAG, "Serial: %02X%02X%02X%02X%02X%02X",
             serial[0], serial[1], serial[2], serial[3], serial[4], serial[5]);

    return ESP_OK;
}

esp_err_t sgp41_heater_off(void) {
    return sgp41_write_command(SGP41_CMD_HEATER_OFF, NULL, 0);
}

esp_err_t sgp41_deinit(void) {
    sgp41_heater_off();
    i2c_driver_delete(s_i2c_num);
    ESP_LOGI(TAG, "SGP41 deinitialized");
    return ESP_OK;
}
