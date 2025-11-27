/**
 * @file scd41.c
 * @brief Sensirion SCD41 CO2传感器驱动实现
 */

#include "scd41.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SCD41";

static i2c_port_t s_i2c_num = I2C_NUM_0;
static bool s_measurement_started = false;

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
 * @brief 写命令到SCD41
 */
static esp_err_t scd41_write_command(uint16_t command, const uint8_t *params, uint8_t params_len) {
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
    i2c_master_write_byte(cmd, (SCD41_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buffer, idx, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write command 0x%04X failed: %s", command, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 从SCD41读取数据
 */
static esp_err_t scd41_read_data(uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD41_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);

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

esp_err_t scd41_init(const scd41_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_num = config->i2c_num;

    // 注意: I2C总线已经由main.c初始化,不需要重复初始化
    ESP_LOGI(TAG, "SCD41 initialized on I2C%d (using existing bus)", s_i2c_num);

    // 等待传感器上电启动(SCD41需要较长时间)
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 尝试停止可能正在运行的测量(传感器可能处于未知状态)
    scd41_write_command(SCD41_CMD_STOP_PERIODIC, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "SCD41 basic initialization complete");
    return ESP_OK;
}

esp_err_t scd41_start_periodic_measurement(void) {
    esp_err_t ret = scd41_write_command(SCD41_CMD_START_PERIODIC, NULL, 0);
    if (ret == ESP_OK) {
        s_measurement_started = true;
        ESP_LOGI(TAG, "Periodic measurement started (5s interval)");
    }
    return ret;
}

esp_err_t scd41_stop_periodic_measurement(void) {
    esp_err_t ret = scd41_write_command(SCD41_CMD_STOP_PERIODIC, NULL, 0);
    if (ret == ESP_OK) {
        s_measurement_started = false;
        ESP_LOGI(TAG, "Periodic measurement stopped");
    }
    return ret;
}

esp_err_t scd41_get_data_ready_status(bool *ready) {
    if (ready == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = scd41_write_command(SCD41_CMD_GET_DATA_READY, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t response[3];  // 2字节数据 + 1字节CRC
    ret = scd41_read_data(response, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    // 验证CRC
    uint8_t crc = sensirion_crc8(response, 2);
    if (crc != response[2]) {
        ESP_LOGE(TAG, "Data ready CRC error");
        return ESP_FAIL;
    }

    uint16_t status = (response[0] << 8) | response[1];
    *ready = ((status & 0x07FF) != 0);  // 低11位非0表示数据就绪

    return ESP_OK;
}

esp_err_t scd41_read_measurement(scd41_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    data->valid = false;

    if (!s_measurement_started) {
        ESP_LOGW(TAG, "Measurement not started, attempting to start...");
        esp_err_t ret = scd41_start_periodic_measurement();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start measurement");
            return ESP_FAIL;
        }
        // 刚启动，数据还未就绪
        return ESP_ERR_NOT_FOUND;
    }

    // 检查数据是否就绪
    bool ready = false;
    esp_err_t ret = scd41_get_data_ready_status(&ready);
    if (ret != ESP_OK || !ready) {
        ESP_LOGD(TAG, "Data not ready");
        return ESP_ERR_NOT_FOUND;
    }

    // 发送读取命令
    ret = scd41_write_command(SCD41_CMD_READ_MEASUREMENT, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    // 读取9字节: CO2(2B+CRC) + Temp(2B+CRC) + RH(2B+CRC)
    uint8_t response[9];
    ret = scd41_read_data(response, 9);
    if (ret != ESP_OK) {
        return ret;
    }

    // 验证CRC
    uint8_t crc_co2 = sensirion_crc8(&response[0], 2);
    uint8_t crc_temp = sensirion_crc8(&response[3], 2);
    uint8_t crc_rh = sensirion_crc8(&response[6], 2);

    if (crc_co2 != response[2] || crc_temp != response[5] || crc_rh != response[8]) {
        ESP_LOGE(TAG, "Measurement CRC error");
        return ESP_FAIL;
    }

    // 解析CO2 (ppm)
    data->co2_ppm = (response[0] << 8) | response[1];

    // 解析温度 (°C) = -45 + 175 * (value / 65536)
    uint16_t temp_raw = (response[3] << 8) | response[4];
    data->temperature = -45.0f + 175.0f * ((float)temp_raw / 65536.0f);

    // 解析湿度 (%) = 100 * (value / 65536)
    uint16_t rh_raw = (response[6] << 8) | response[7];
    data->humidity = 100.0f * ((float)rh_raw / 65536.0f);

    data->valid = true;

    ESP_LOGD(TAG, "CO2: %d ppm, Temp: %.2f°C, RH: %.2f%%",
             data->co2_ppm, data->temperature, data->humidity);

    return ESP_OK;
}

esp_err_t scd41_self_test(void) {
    // 停止测量
    esp_err_t ret = scd41_stop_periodic_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // 执行自检
    ret = scd41_write_command(SCD41_CMD_SELF_TEST, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    // 等待自检完成 (10秒)
    ESP_LOGI(TAG, "Self-test in progress (10s)...");
    vTaskDelay(pdMS_TO_TICKS(10000));

    uint8_t response[3];
    ret = scd41_read_data(response, 3);
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
    if (result != 0x0000) {
        ESP_LOGE(TAG, "Self-test failed: 0x%04X", result);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Self-test passed");
    return ESP_OK;
}

esp_err_t scd41_get_serial_number(uint8_t serial[6]) {
    if (serial == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 停止测量
    esp_err_t ret = scd41_stop_periodic_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ret = scd41_write_command(SCD41_CMD_GET_SERIAL, NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t response[9];  // 6字节数据 + 3字节CRC
    ret = scd41_read_data(response, 9);
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

esp_err_t scd41_set_temperature_offset(float temp_offset) {
    // 停止测量
    esp_err_t ret = scd41_stop_periodic_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    // 转换温度偏移: ticks = offset * 65536 / 175
    uint16_t offset_ticks = (uint16_t)(temp_offset * 65536.0f / 175.0f);

    uint8_t params[2];
    params[0] = (offset_ticks >> 8) & 0xFF;
    params[1] = offset_ticks & 0xFF;

    ret = scd41_write_command(SCD41_CMD_SET_TEMP_OFFSET, params, 2);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Temperature offset set to %.2f°C", temp_offset);
    }

    return ret;
}

esp_err_t scd41_set_sensor_altitude(uint16_t altitude_m) {
    // 停止测量
    esp_err_t ret = scd41_stop_periodic_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t params[2];
    params[0] = (altitude_m >> 8) & 0xFF;
    params[1] = altitude_m & 0xFF;

    ret = scd41_write_command(SCD41_CMD_SET_SENSOR_ALT, params, 2);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sensor altitude set to %d m", altitude_m);
    }

    return ret;
}

esp_err_t scd41_perform_forced_calibration(uint16_t target_co2, int16_t *frc_correction) {
    if (frc_correction == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 停止测量
    esp_err_t ret = scd41_stop_periodic_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t params[2];
    params[0] = (target_co2 >> 8) & 0xFF;
    params[1] = target_co2 & 0xFF;

    ret = scd41_write_command(SCD41_CMD_FORCE_CALIBRATION, params, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    // 等待校准完成 (400ms)
    vTaskDelay(pdMS_TO_TICKS(400));

    uint8_t response[3];
    ret = scd41_read_data(response, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    // 验证CRC
    uint8_t crc = sensirion_crc8(response, 2);
    if (crc != response[2]) {
        ESP_LOGE(TAG, "FRC CRC error");
        return ESP_FAIL;
    }

    *frc_correction = (int16_t)((response[0] << 8) | response[1]);
    if (*frc_correction == (int16_t)0xFFFF) {  // -1 in signed int16_t
        ESP_LOGE(TAG, "Forced calibration failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Forced calibration successful, correction: %d ppm", *frc_correction);
    return ESP_OK;
}

esp_err_t scd41_factory_reset(void) {
    // 停止测量
    esp_err_t ret = scd41_stop_periodic_measurement();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ret = scd41_write_command(SCD41_CMD_FACTORY_RESET, NULL, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Factory reset executed");
        vTaskDelay(pdMS_TO_TICKS(1200));
    }

    return ret;
}

esp_err_t scd41_deinit(void) {
    scd41_stop_periodic_measurement();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "SCD41 deinitialized");
    return ESP_OK;
}
