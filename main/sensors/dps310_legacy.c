/**
 * @file dps310_legacy.c
 * @brief Infineon DPS310 气压传感器驱动实现 (使用旧版 I2C API)
 */

#include "dps310_legacy.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DPS310";

static i2c_port_t s_i2c_num = I2C_NUM_0;
static uint8_t s_i2c_addr = DPS310_I2C_ADDR_1;

// 校准系数
static int32_t s_c0, s_c1;
static int32_t s_c00, s_c10, s_c01, s_c11, s_c20, s_c21, s_c30;

// 缩放因子
static float s_kp = 0;  // 气压缩放因子
static float s_kt = 0;  // 温度缩放因子

// 海拔参考气压
static float s_ref_pressure = 101325.0f;  // 标准大气压 (Pa)

/**
 * @brief 写寄存器
 */
static esp_err_t dps310_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buffer, 2, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 读寄存器
 */
static esp_err_t dps310_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
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
        ESP_LOGE(TAG, "Read reg 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 获取缩放因子
 */
static float get_scale_factor(dps310_oversample_t osr) {
    switch (osr) {
        case DPS310_OVERSAMPLE_1:   return 524288.0f;
        case DPS310_OVERSAMPLE_2:   return 1572864.0f;
        case DPS310_OVERSAMPLE_4:   return 3670016.0f;
        case DPS310_OVERSAMPLE_8:   return 7864320.0f;
        case DPS310_OVERSAMPLE_16:  return 253952.0f;
        case DPS310_OVERSAMPLE_32:  return 516096.0f;
        case DPS310_OVERSAMPLE_64:  return 1040384.0f;
        case DPS310_OVERSAMPLE_128: return 2088960.0f;
        default: return 524288.0f;
    }
}

/**
 * @brief 读取校准系数
 */
static esp_err_t dps310_read_calibration_coeffs(void) {
    uint8_t coef[18];
    esp_err_t ret = dps310_read_reg(DPS310_REG_COEF_BASE, coef, 18);
    if (ret != ESP_OK) {
        return ret;
    }

    // 解析校准系数
    s_c0 = ((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F);
    if (s_c0 & 0x0800) s_c0 -= 4096;  // 符号扩展

    s_c1 = (((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2];
    if (s_c1 & 0x0800) s_c1 -= 4096;

    s_c00 = ((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F);
    if (s_c00 & 0x080000) s_c00 -= 1048576;

    s_c10 = (((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7];
    if (s_c10 & 0x080000) s_c10 -= 1048576;

    s_c01 = ((uint32_t)coef[8] << 8) | (uint32_t)coef[9];
    if (s_c01 & 0x8000) s_c01 -= 65536;

    s_c11 = ((uint32_t)coef[10] << 8) | (uint32_t)coef[11];
    if (s_c11 & 0x8000) s_c11 -= 65536;

    s_c20 = ((uint32_t)coef[12] << 8) | (uint32_t)coef[13];
    if (s_c20 & 0x8000) s_c20 -= 65536;

    s_c21 = ((uint32_t)coef[14] << 8) | (uint32_t)coef[15];
    if (s_c21 & 0x8000) s_c21 -= 65536;

    s_c30 = ((uint32_t)coef[16] << 8) | (uint32_t)coef[17];
    if (s_c30 & 0x8000) s_c30 -= 65536;

    ESP_LOGI(TAG, "Calibration coefficients loaded successfully");
    return ESP_OK;
}

esp_err_t dps310_legacy_init(const dps310_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_i2c_num = config->i2c_num;
    s_i2c_addr = config->i2c_addr;

    ESP_LOGI(TAG, "DPS310 initialized on I2C%d (Addr:0x%02X, using existing bus)",
             s_i2c_num, s_i2c_addr);

    // 等待传感器启动
    vTaskDelay(pdMS_TO_TICKS(50));

    // 读取产品ID
    uint8_t product_id;
    esp_err_t ret = dps310_read_reg(DPS310_REG_PRODUCT_ID, &product_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read product ID");
        return ret;
    }

    if (product_id != DPS310_PRODUCT_ID) {
        ESP_LOGE(TAG, "Invalid product ID: 0x%02X (expected 0x10)", product_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Product ID verified: 0x%02X", product_id);

    // 软复位
    ret = dps310_legacy_soft_reset();
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // 读取校准系数
    ret = dps310_read_calibration_coeffs();
    if (ret != ESP_OK) {
        return ret;
    }

    // 计算缩放因子
    s_kp = get_scale_factor(config->prs_osr);
    s_kt = get_scale_factor(config->tmp_osr);

    // 配置气压测量
    uint8_t prs_cfg = (config->prs_rate << 4) | config->prs_osr;
    ret = dps310_write_reg(DPS310_REG_PRS_CFG, prs_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    // 配置温度测量
    uint8_t tmp_cfg = (config->tmp_rate << 4) | config->tmp_osr | (config->tmp_src << 7);
    ret = dps310_write_reg(DPS310_REG_TMP_CFG, tmp_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    // 配置结果移位（用于高过采样率）
    uint8_t cfg_reg = 0;
    if (config->prs_osr > DPS310_OVERSAMPLE_8) {
        cfg_reg |= 0x04;  // P_SHIFT
    }
    if (config->tmp_osr > DPS310_OVERSAMPLE_8) {
        cfg_reg |= 0x08;  // T_SHIFT
    }
    ret = dps310_write_reg(DPS310_REG_CFG_REG, cfg_reg);
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(TAG, "DPS310 initialization complete");
    return ESP_OK;
}

esp_err_t dps310_legacy_start_continuous(dps310_mode_t mode) {
    uint8_t meas_cfg = mode;
    esp_err_t ret = dps310_write_reg(DPS310_REG_MEAS_CFG, meas_cfg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Continuous measurement started (mode: %d)", mode);
    }
    return ret;
}

esp_err_t dps310_legacy_stop(void) {
    return dps310_write_reg(DPS310_REG_MEAS_CFG, DPS310_MODE_IDLE);
}

esp_err_t dps310_legacy_read_data(dps310_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    data->valid = false;

    // 检查数据就绪状态 (MEAS_CFG寄存器: bit6=PRS_RDY, bit5=TMP_RDY)
    uint8_t meas_cfg;
    esp_err_t ret = dps310_read_reg(DPS310_REG_MEAS_CFG, &meas_cfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MEAS_CFG register");
        return ret;
    }

    // 检查气压和温度数据是否都就绪
    bool prs_ready = (meas_cfg & 0x10) != 0;  // bit4: PRS_RDY
    bool tmp_ready = (meas_cfg & 0x20) != 0;  // bit5: TMP_RDY

    if (!prs_ready || !tmp_ready) {
        ESP_LOGD(TAG, "Data not ready (MEAS_CFG=0x%02X, PRS=%d, TMP=%d)",
                 meas_cfg, prs_ready, tmp_ready);
        return ESP_ERR_NOT_FINISHED;
    }

    // 读取原始数据
    uint8_t prs_data[3];
    uint8_t tmp_data[3];

    ret = dps310_read_reg(DPS310_REG_PSR_B2, prs_data, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = dps310_read_reg(DPS310_REG_TMP_B2, tmp_data, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    // 解析原始值
    int32_t prs_raw = ((int32_t)prs_data[0] << 16) | ((int32_t)prs_data[1] << 8) | prs_data[2];
    if (prs_raw & 0x800000) prs_raw -= 16777216;  // 24位符号扩展

    int32_t tmp_raw = ((int32_t)tmp_data[0] << 16) | ((int32_t)tmp_data[1] << 8) | tmp_data[2];
    if (tmp_raw & 0x800000) tmp_raw -= 16777216;

    // 计算温度 (°C)
    float tmp_scaled = (float)tmp_raw / s_kt;
    data->temperature = s_c0 * 0.5f + s_c1 * tmp_scaled;

    // 计算气压 (Pa)
    float prs_scaled = (float)prs_raw / s_kp;
    data->pressure = s_c00 + prs_scaled * (s_c10 + prs_scaled * (s_c20 + prs_scaled * s_c30)) +
                     tmp_scaled * s_c01 + tmp_scaled * prs_scaled * (s_c11 + prs_scaled * s_c21);

    // 计算海拔 (m) - 使用气压高度公式
    data->altitude = 44330.0f * (1.0f - powf(data->pressure / s_ref_pressure, 0.1903f));

    data->valid = true;

    ESP_LOGD(TAG, "P: %.2f Pa, T: %.2f°C, Alt: %.2f m",
             data->pressure, data->temperature, data->altitude);

    return ESP_OK;
}

esp_err_t dps310_legacy_set_altitude_ref(float altitude_m) {
    // 根据当前海拔计算参考气压
    s_ref_pressure = 101325.0f * powf(1.0f - altitude_m / 44330.0f, 5.255f);
    ESP_LOGI(TAG, "Altitude reference set to %.2f m (ref pressure: %.2f Pa)",
             altitude_m, s_ref_pressure);
    return ESP_OK;
}

esp_err_t dps310_legacy_soft_reset(void) {
    esp_err_t ret = dps310_write_reg(DPS310_REG_RESET, 0x89);  // 软复位命令
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Soft reset executed");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ret;
}

esp_err_t dps310_legacy_deinit(void) {
    dps310_legacy_stop();
    ESP_LOGI(TAG, "DPS310 deinitialized");
    return ESP_OK;
}
