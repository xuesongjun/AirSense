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
#include "freertos/semphr.h"

static const char *TAG = "DPS310";

static i2c_port_t s_i2c_num = I2C_NUM_0;
static uint8_t s_i2c_addr = DPS310_I2C_ADDR_1;
static SemaphoreHandle_t s_i2c_mutex = NULL;

#define DPS310_ALT_REF_RETRY_COUNT   20
#define DPS310_ALT_REF_RETRY_DELAY_MS 50

static inline void dps310_lock(void) {
    if (s_i2c_mutex) {
        // 始终阻塞等待，避免上层偶发拿锁失败
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    }
}

static inline void dps310_unlock(void) {
    if (s_i2c_mutex) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

// 校准系数
static int32_t s_c0, s_c1;
static int32_t s_c00, s_c10, s_c01, s_c11, s_c20, s_c21, s_c30;

// 缩放因子
static float s_kp = 0;  // 气压缩放因子
static float s_kt = 0;  // 温度缩放因子

// 海拔参考气压和海拔值
static float s_ref_pressure = 101325.0f;  // 标准大气压 (Pa)
static float s_ref_altitude = 0.0f;       // 参考海拔 (m)

/**
 * @brief 写寄存器
 */
static esp_err_t dps310_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};

    dps310_lock();
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

    dps310_unlock();
    return ret;
}

/**
 * @brief 读寄存器
 */
static esp_err_t dps310_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    dps310_lock();
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

    dps310_unlock();
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

    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        if (s_i2c_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create DPS310 mutex");
            return ESP_ERR_NO_MEM;
        }
    }

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

    // 读取 COEF_SRCE 寄存器判断校准系数是针对哪个温度源的
    // COEF_SRCE bit7 是只读的，指示 OTP 中存储的系数适用于哪个温度传感器
    // 我们必须使用与系数匹配的温度源，否则补偿会有系统偏差
    uint8_t coef_srce;
    bool use_external_sensor = false;  // 默认使用内部
    ret = dps310_read_reg(DPS310_REG_COEF_SRCE, &coef_srce, 1);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read COEF_SRCE register, using internal temp source");
    } else {
        // COEF_SRCE bit7: 0=系数针对内部ASIC温度, 1=系数针对外部MEMS温度
        use_external_sensor = (coef_srce & 0x80) != 0;
        ESP_LOGI(TAG, "COEF_SRCE: 0x%02X (calibration for %s temperature sensor)",
                 coef_srce, use_external_sensor ? "external MEMS" : "internal ASIC");

        // 如果用户配置与芯片校准系数不匹配，给出警告并使用芯片推荐的配置
        if (config->tmp_src == DPS310_TMP_SRC_EXTERNAL && !use_external_sensor) {
            ESP_LOGW(TAG, "User requested external temp source, but chip calibrated for internal");
            ESP_LOGW(TAG, "Using internal source to match calibration coefficients");
        } else if (config->tmp_src == DPS310_TMP_SRC_INTERNAL && use_external_sensor) {
            ESP_LOGW(TAG, "User requested internal temp source, but chip calibrated for external");
            ESP_LOGW(TAG, "Using external source to match calibration coefficients");
        }
    }

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

    // 配置温度测量 - 使用与校准系数匹配的温度源
    uint8_t tmp_cfg = (config->tmp_rate << 4) | config->tmp_osr | (use_external_sensor ? 0x80 : 0x00);
    ret = dps310_write_reg(DPS310_REG_TMP_CFG, tmp_cfg);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "Temperature source configured: %s", use_external_sensor ? "external MEMS" : "internal ASIC");

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
    // 公式计算的是相对于海平面的高度，加上参考海拔得到绝对高度
    float relative_alt = 44330.0f * (1.0f - powf(data->pressure / s_ref_pressure, 0.1903f));
    data->altitude = relative_alt + s_ref_altitude;

    data->valid = true;

    ESP_LOGD(TAG, "P: %.2f Pa, T: %.2f°C, Alt: %.2f m",
             data->pressure, data->temperature, data->altitude);

    return ESP_OK;
}

esp_err_t dps310_legacy_set_altitude_ref(float altitude_m) {
    // 读取当前气压，如果数据未就绪则等待重试
    dps310_data_t current_data;
    esp_err_t ret;

    for (int retry = 0; retry < DPS310_ALT_REF_RETRY_COUNT; retry++) {
        ret = dps310_legacy_read_data(&current_data);
        if (ret == ESP_OK && current_data.valid) {
            break;  // 数据就绪
        }
        if (ret != ESP_ERR_NOT_FINISHED) {
            // 发生了非"未就绪"的错误
            ESP_LOGE(TAG, "Failed to read pressure: %s", esp_err_to_name(ret));
            return ret;
        }
        // 数据未就绪，等待后重试
        vTaskDelay(pdMS_TO_TICKS(DPS310_ALT_REF_RETRY_DELAY_MS));
    }

    if (ret != ESP_OK || !current_data.valid) {
        ESP_LOGE(TAG, "Failed to read current pressure for altitude calibration (timeout)");
        return ESP_FAIL;
    }

    // 存储用户设置的海拔值
    s_ref_altitude = altitude_m;

    // 根据当前气压和用户指定的海拔，反推海平面气压
    // 气压高度公式: altitude = 44330 * (1 - (P / P0)^0.1903)
    // 反推: P0 = P / (1 - altitude / 44330)^(1/0.1903)
    // 其中 1/0.1903 ≈ 5.255
    float ratio = 1.0f - altitude_m / 44330.0f;
    if (ratio <= 0.0f) {
        ESP_LOGE(TAG, "Invalid altitude: %.2f m (too high)", altitude_m);
        return ESP_ERR_INVALID_ARG;
    }
    s_ref_pressure = current_data.pressure / powf(ratio, 5.255f);

    ESP_LOGI(TAG, "Altitude reference set to %.2f m (current pressure: %.2f Pa, sea-level ref: %.2f Pa / %.2f hPa)",
             altitude_m, current_data.pressure, s_ref_pressure, s_ref_pressure / 100.0f);
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
