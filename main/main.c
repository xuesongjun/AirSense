/**
 * @file main.c
 * @brief AirSense 空气质量检测仪主程序
 *
 * ESP32-C5验证机程序
 * 集成传感器:
 * - SenseAir S88LP: CO2红外传感器 (UART1 Modbus)
 * - Sensirion SCD41: CO2传感器 (I2C0)
 * - Sensirion SGP41: VOC/NOx传感器 (I2C0)
 * - Sensirion SPS30: PM颗粒物传感器 (I2C0)
 * - Prosense WZ-H3-N: HCHO甲醛电化学传感器 (LP_UART GPIO4/5)
 * - Infineon DPS310: 气压/温度传感器 (I2C0)
 * - Sensirion SHT85: 温湿度传感器 (I2C0)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

// 传感器驱动头文件
#include "sensors/senseair_s88lp.h" // S88LP CO2传感器 (UART Modbus)
#include "sensors/scd41.h"          // SCD41 CO2传感器
#include "sensors/sgp41.h"          // SGP41 VOC/NOx传感器
#include "sensors/sht85.h"          // SHT85温湿度传感器
#include "sensors/prosense_wzh3n.h" // WZ-H3-N HCHO传感器 (Prosense普晟)
#include "sensors/dps310_legacy.h"  // DPS310气压传感器 (旧版I2C API)

static const char *TAG = "AirSense";

// 传感器使能开关
#define ENABLE_S88LP            1  // S88LP CO2传感器 (UART1 硬件串口)
#define ENABLE_SCD41            1  // SCD41 CO2传感器 (I2C0)
#define ENABLE_SGP41            1  // SGP41 VOC/NOx传感器 (I2C0)
#define ENABLE_SHT85            1  // SHT85温湿度传感器 (I2C0)
#define ENABLE_PROSENSE_WZH3N   1  // WZ-H3-N HCHO甲醛传感器 (LP_UART GPIO4/GPIO5)
#define ENABLE_DPS310           1  // DPS310气压传感器 (I2C0) - 使用旧版I2C API

// 引脚定义 (ESP32-C5-DevKitC-1)
// UART1 - S88LP CO2传感器 (硬件UART)
#define S88LP_TXD_PIN           GPIO_NUM_6
#define S88LP_RXD_PIN           GPIO_NUM_7

// LP_UART - Prosense WZ-H3-N HCHO甲醛传感器 (硬件UART)
// LP_UART引脚固定在LP_GPIO0~5, 对应GPIO0~5
#define PROSENSE_TXD_PIN        GPIO_NUM_5
#define PROSENSE_RXD_PIN        GPIO_NUM_4

// I2C0 - SGP41和DPS310共用 (ESP32-C5开发板)
#define I2C0_SDA_PIN            GPIO_NUM_2
#define I2C0_SCL_PIN            GPIO_NUM_3

// 注: 所有I2C传感器通过扩展板共用I2C0
// 注: UART0用于固件烧录和日志输出，不可占用
// 注: WZ-H3-N使用LP_UART(GPIO4/GPIO5)硬件串口

// 测量间隔 (ms) - 128次过采样测量时间~344ms
#define MEASURE_INTERVAL_MS     3000  // 3秒间隔,给128次过采样足够余量

// DPS310状态（全局变量）
#if ENABLE_DPS310
static bool altitude_calibrated = false;  // 海拔是否已校准
static float reference_pressure = 0;      // 参考气压(开机时记录)

// IIR低通滤波器配置
#define IIR_FILTER_ALPHA 0.2f
static float filtered_pressure = 0;
static bool filter_initialized = false;
#endif

/**
 * @brief IIR低通滤波器 - 平滑气压数据
 *
 * 根据DPS310文档建议实现IIR滤波,用于抑制门窗开关、风扇等短期气流扰动
 * 公式: y[n] = α*x[n] + (1-α)*y[n-1]
 *
 * @param new_value 新的气压读数(Pa)
 * @return 滤波后的气压值(Pa)
 */
#if ENABLE_DPS310
static float iir_lowpass_filter(float new_value) {
    if (!filter_initialized) {
        // 首次使用,直接使用当前值初始化
        filtered_pressure = new_value;
        filter_initialized = true;
        return filtered_pressure;
    }

    // 一阶IIR低通滤波: y[n] = α*x[n] + (1-α)*y[n-1]
    // α=0.2: 80%来自历史平滑值, 20%来自新测量值
    filtered_pressure = IIR_FILTER_ALPHA * new_value + (1.0f - IIR_FILTER_ALPHA) * filtered_pressure;
    return filtered_pressure;
}
#endif

/**
 * @brief 初始化所有传感器
 */
static void init_all_sensors(void) {
    ESP_LOGI(TAG, "Initializing sensors...");
    esp_err_t ret = ESP_OK;

    // 初始化I2C总线 (旧版驱动,用于SCD41/SGP41/SHT85)
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C0_SDA_PIN,
        .scl_io_num = I2C0_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,  // 100kHz
    };
    ret = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (ret == ESP_OK) {
        ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C bus initialized on I2C_NUM_0 (SDA:%d, SCL:%d, 100kHz)",
                     I2C0_SDA_PIN, I2C0_SCL_PIN);

            // I2C设备扫描 (调试用,已确认: 0x44=SHT85, 0x62=SCD41, 0x77=DPS310)
            #if 0  // 设置为1启用扫描
            ESP_LOGI(TAG, "Scanning I2C bus...");
            for (uint8_t addr = 1; addr < 127; addr++) {
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
                i2c_master_stop(cmd);
                esp_err_t scan_ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
                i2c_cmd_link_delete(cmd);
                if (scan_ret == ESP_OK) {
                    ESP_LOGI(TAG, "  Found I2C device at 0x%02X", addr);
                }
            }
            ESP_LOGI(TAG, "I2C scan complete");
            #endif
        } else {
            ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
    }

#if ENABLE_S88LP
    // 初始化SenseAir S88LP CO2传感器 (UART1 Modbus)
    senseair_s8_config_t s88lp_config = {
        .uart_num = UART_NUM_1,
        .tx_pin = S88LP_TXD_PIN,
        .rx_pin = S88LP_RXD_PIN,
        .baud_rate = 9600,
        .modbus_addr = SENSEAIR_S8_DEFAULT_ADDR,
    };
    ret = senseair_s8_init(&s88lp_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SenseAir S88LP CO2 sensor initialized");
    } else {
        ESP_LOGE(TAG, "Failed to initialize S88LP: %s", esp_err_to_name(ret));
    }
#endif

#if ENABLE_SCD41
    // 初始化SCD41 CO2传感器 (I2C0)
    scd41_config_t scd41_config = {
        .i2c_num = I2C_NUM_0,
        .sda_pin = I2C0_SDA_PIN,
        .scl_pin = I2C0_SCL_PIN,
        .i2c_freq_hz = SCD41_I2C_FREQ_HZ,
        .altitude_m = 10,           // 设置海拔(南京浦口约10m)
        .temp_offset = 0.0f,        // 温度补偿偏移
    };
    ret = scd41_init(&scd41_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SCD41 CO2 sensor initialized");

        // 注意: 获取序列号需要先停止测量,为简化初始化流程,暂时跳过
        // 可以在启动后手动调用 scd41_get_serial_number()

        // 启动周期性测量
        if (scd41_start_periodic_measurement() == ESP_OK) {
            ESP_LOGI(TAG, "SCD41 periodic measurement started (5s interval)");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize SCD41: %s", esp_err_to_name(ret));
    }
#endif

#if ENABLE_SGP41
    // 初始化SGP41 VOC/NOx传感器 (I2C0)
    sgp41_config_t sgp41_config = {
        .i2c_num = I2C_NUM_0,
        .sda_pin = I2C0_SDA_PIN,
        .scl_pin = I2C0_SCL_PIN,
        .i2c_freq_hz = SGP41_I2C_FREQ_HZ,
    };
    ret = sgp41_init(&sgp41_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SGP41 VOC/NOx sensor initialized");

        // 注意: 为简化初始化流程,跳过序列号读取和自检
        // 可以在启动后手动调用这些功能测试函数
    } else {
        ESP_LOGE(TAG, "Failed to initialize SGP41: %s", esp_err_to_name(ret));
    }
#endif

#if ENABLE_SHT85
    // 初始化SHT85温湿度传感器 (I2C0)
    sht85_config_t sht85_config = {
        .i2c_num = I2C_NUM_0,
        .i2c_addr = SHT85_I2C_ADDRESS_LOW,  // 0x44 (ADDR引脚接地)
        .sda_pin = I2C0_SDA_PIN,
        .scl_pin = I2C0_SCL_PIN,
        .i2c_freq_hz = SHT85_I2C_FREQ_HZ,
    };
    ret = sht85_init(&sht85_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SHT85 temperature/humidity sensor initialized");

        // 注意: 为简化初始化流程,跳过序列号读取
        // 可以在启动后手动调用 sht85_get_serial_number()
    } else {
        ESP_LOGE(TAG, "Failed to initialize SHT85: %s", esp_err_to_name(ret));
    }
#endif

#if ENABLE_PROSENSE_WZH3N
    // 初始化Prosense WZ-H3-N HCHO甲醛传感器 (LP_UART GPIO4/GPIO5)
    // 使用问答模式，可直接获取ug/m³和ppb两种单位数据
    prosense_wzh3n_config_t prosense_config = {
        .tx_pin = PROSENSE_TXD_PIN,
        .rx_pin = PROSENSE_RXD_PIN,
        .baud_rate = PROSENSE_WZH3N_BAUD_RATE,
        .active_upload = false,  // 问答模式
    };
    ret = prosense_wzh3n_init(&prosense_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Prosense WZ-H3-N initialized (LP_UART on GPIO%d/GPIO%d, Q&A Mode)", PROSENSE_TXD_PIN, PROSENSE_RXD_PIN);
    } else {
        ESP_LOGE(TAG, "Failed to initialize Prosense WZ-H3-N: %s", esp_err_to_name(ret));
    }
#endif

#if ENABLE_DPS310
    // 初始化DPS310气压传感器 (I2C0, 使用旧版I2C API)
    dps310_config_t dps310_config = {
        .i2c_num = I2C_NUM_0,
        .i2c_addr = DPS310_I2C_ADDR_1,  // 0x77 (SDO接VDD)
        .sda_pin = I2C0_SDA_PIN,
        .scl_pin = I2C0_SCL_PIN,
        .i2c_freq_hz = 100000,
        .prs_osr = DPS310_OVERSAMPLE_128,  // 气压128次过采样
        .tmp_osr = DPS310_OVERSAMPLE_128,  // 温度128次过采样
        .prs_rate = DPS310_RATE_1,         // 1Hz
        .tmp_rate = DPS310_RATE_1,
        .tmp_src = DPS310_TMP_SRC_EXTERNAL // 使用外部MEMS温度
    };
    ret = dps310_legacy_init(&dps310_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DPS310 pressure sensor initialized");

        // 启动连续测量
        ret = dps310_legacy_start_continuous(DPS310_MODE_CONTINUOUS_BOTH);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "DPS310 continuous measurement started");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize DPS310: %s", esp_err_to_name(ret));
    }
#endif

    ESP_LOGI(TAG, "Sensor initialization complete");

#if ENABLE_SCD41 && ENABLE_SHT85
    // SCD41自动温度偏移补偿
    // 等待传感器稳定并获取第一次测量值
    ESP_LOGI(TAG, "Starting SCD41 automatic temperature offset calibration...");
    ESP_LOGI(TAG, "Waiting for sensors to stabilize (10 seconds)...");
    vTaskDelay(pdMS_TO_TICKS(10000));  // 等待10秒,确保SCD41完成至少一个完整的5秒周期

    sht85_data_t sht_cal;
    scd41_data_t scd_cal;
    bool calibration_success = false;

    // 尝试读取5次,确保获取有效数据 (每次间隔1秒,最多等待5秒)
    for (int i = 0; i < 5; i++) {
        esp_err_t sht_ret = sht85_read_measurement(SHT85_REPEATABILITY_HIGH, &sht_cal);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_err_t scd_ret = scd41_read_measurement(&scd_cal);

        if (sht_ret == ESP_OK && sht_cal.valid && scd_ret == ESP_OK && scd_cal.valid) {
            // 计算温度偏移 (SCD41内部温度 - 真实环境温度)
            float temp_offset = scd_cal.temperature - sht_cal.temperature;

            ESP_LOGI(TAG, "Temperature offset detected: %.2f°C (SCD41=%.2f°C, SHT85=%.2f°C)",
                     temp_offset, scd_cal.temperature, sht_cal.temperature);

            // 根据温度偏移判断是否需要补偿
            // 正常自热: temp_offset > 0 (SCD41温度更高)
            // 异常情况: temp_offset < 0 (SCD41温度更低,可能是气流差异)
            if (temp_offset >= 0.5f && temp_offset <= 5.0f) {
                // 典型自热效应: SCD41比环境温度高0.5~5°C
                ESP_LOGI(TAG, "Normal self-heating detected, applying temperature offset compensation");
                ret = scd41_set_temperature_offset(temp_offset);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "SCD41 temperature offset set to %.2f°C", temp_offset);

                    // 设置海拔补偿 (南京浦口区海拔约10m)
                    ret = scd41_set_sensor_altitude(10);
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "SCD41 sensor altitude set to 10m");
                    }

                    // 重新启动测量
                    vTaskDelay(pdMS_TO_TICKS(500));
                    ret = scd41_start_periodic_measurement();
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "SCD41 periodic measurement restarted with calibration");
                        calibration_success = true;
                    }
                }
            } else if (temp_offset < -0.5f && temp_offset >= -5.0f) {
                // 反向偏移: SCD41比环境温度低(可能是传感器位置或气流差异)
                ESP_LOGW(TAG, "Reverse temperature offset detected (SCD41 cooler than SHT85)");
                ESP_LOGW(TAG, "This may indicate: 1) Different sensor locations, 2) Better airflow at SCD41");
                ESP_LOGW(TAG, "Skipping temperature offset compensation - review sensor placement");

                // 仅设置海拔补偿,不设置温度偏移
                ret = scd41_set_sensor_altitude(10);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "SCD41 sensor altitude set to 10m (no temp offset)");
                    vTaskDelay(pdMS_TO_TICKS(500));
                    scd41_start_periodic_measurement();
                }
            } else {
                ESP_LOGW(TAG, "Temperature offset %.2f°C out of valid range, skipping calibration", temp_offset);
                ESP_LOGW(TAG, "Expected range: -5.0°C to +5.0°C, |offset| >= 0.5°C");
            }
            break;
        }

        // 如果数据未就绪,等待1秒后重试 (给SCD41时间产生下一个数据)
        if (i < 4) {  // 最后一次不需要等待
            ESP_LOGD(TAG, "Temperature calibration attempt %d/5 failed, retrying in 1 second...", i + 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (!calibration_success) {
        ESP_LOGW(TAG, "SCD41 automatic temperature calibration failed after 5 attempts");
        ESP_LOGW(TAG, "This may indicate sensor communication issues");
        ESP_LOGW(TAG, "Continuing with default configuration");
    }
#endif
}

/**
 * @brief 传感器数据采集任务
 */
static void sensor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor task started");

    // 等待传感器稳定(128次过采样需要更长的稳定时间)
    ESP_LOGI(TAG, "Waiting for sensors to stabilize (128x oversampling)...");
    vTaskDelay(pdMS_TO_TICKS(3000));  // 3秒,确保传感器完全稳定

    while (1) {
        ESP_LOGI(TAG, "=== Reading Sensors ===");

        // 用于存储温湿度数据(供SGP41补偿使用)
        float ambient_temp = 25.0f;
        float ambient_rh = 50.0f;

#if ENABLE_SHT85
        // 读取SHT85 - 温湿度 (优先读取,供其他传感器补偿使用)
        sht85_data_t sht85_data;
        if (sht85_read_measurement(SHT85_REPEATABILITY_HIGH, &sht85_data) == ESP_OK && sht85_data.valid) {
            ambient_temp = sht85_data.temperature;
            ambient_rh = sht85_data.humidity;
            printf("[SHT85] Temp: %.2f°C, RH: %.1f%%\n",
                   sht85_data.temperature, sht85_data.humidity);
        } else {
            printf("[SHT85] Failed to read temperature/humidity\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

#if ENABLE_SCD41
        // 读取SCD41 - CO2/温度/湿度
        scd41_data_t scd41_data;
        if (scd41_read_measurement(&scd41_data) == ESP_OK && scd41_data.valid) {
            printf("[SCD41] CO2: %d ppm, Temp: %.2f°C, RH: %.1f%%\n",
                   scd41_data.co2_ppm, scd41_data.temperature, scd41_data.humidity);
        } else {
            // 数据未就绪是正常的(SCD41每5秒更新一次)
            ESP_LOGD(TAG, "SCD41 data not ready");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

#if ENABLE_S88LP
        // 读取SenseAir S88LP - CO2
        senseair_s8_data_t s88lp_data;
        if (senseair_s8_read_co2(&s88lp_data) == ESP_OK && s88lp_data.valid) {
            printf("[S88LP] CO2: %d ppm, Status: 0x%02X\n", s88lp_data.co2_ppm, s88lp_data.status);
        } else {
            printf("[S88LP] Failed to read CO2\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

#if ENABLE_SGP41
        // 读取SGP41 - VOC/NOx (使用SHT85的温湿度补偿，浮点精度)
        sgp41_data_t sgp41_data;

        if (sgp41_measure_raw_f(ambient_rh, ambient_temp, &sgp41_data) == ESP_OK && sgp41_data.valid) {
            printf("[SGP41] VOC raw: %d, NOx raw: %d, VOC index: %ld, NOx index: %ld\n",
                   sgp41_data.sraw_voc, sgp41_data.sraw_nox,
                   (long)sgp41_data.voc_index, (long)sgp41_data.nox_index);
        } else {
            printf("[SGP41] Failed to read VOC/NOx\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

#if ENABLE_PROSENSE_WZH3N
        // 读取Prosense WZ-H3-N - HCHO甲醛 (问答模式)
        prosense_wzh3n_data_t prosense_data;
        if (prosense_wzh3n_read_concentration(&prosense_data) == ESP_OK && prosense_data.valid) {
            printf("[WZ-H3-N] HCHO: %d ug/m³ (%.3f mg/m³) / %d ppb (%.3f ppm)\n",
                   prosense_data.hcho_ugm3, prosense_data.hcho_mgm3,
                   prosense_data.hcho_ppb, prosense_data.hcho_ppm);
        } else {
            printf("[WZ-H3-N] Failed to read HCHO\n");
        }
#endif

#if ENABLE_DPS310
        // 读取DPS310 - 气压/温度/海拔
        dps310_data_t dps310_data;
        if (dps310_legacy_read_data(&dps310_data) == ESP_OK && dps310_data.valid) {
            // 应用IIR低通滤波器平滑气压数据
            float raw_pressure = dps310_data.pressure;
            dps310_data.pressure = iir_lowpass_filter(dps310_data.pressure);

            // 记录开机时的参考气压(用于相对高度计算)
            if (reference_pressure == 0) {
                reference_pressure = dps310_data.pressure;
                ESP_LOGI(TAG, "Reference pressure set to %.2f hPa at startup", reference_pressure / 100.0f);
            }

            // 计算相对高度变化(相对于开机位置)
            float height_change = (reference_pressure - dps310_data.pressure) / 12.0f;

            printf("[DPS310] Temp: %.2f°C, P_raw: %.2f, P_filt: %.2f hPa, Alt: %.2f m, Δh: %+.2f m%s\n",
                   dps310_data.temperature,
                   raw_pressure / 100.0f,
                   dps310_data.pressure / 100.0f,
                   dps310_data.altitude,
                   height_change,
                   altitude_calibrated ? "" : " (uncalib)");

#if ENABLE_SCD41
            // 用DPS310气压动态补偿SCD41 (比海拔补偿更精确)
            // 气压单位: DPS310返回Pa, SCD41需要hPa
            uint16_t pressure_hpa = (uint16_t)(dps310_data.pressure / 100.0f + 0.5f);
            if (pressure_hpa >= 700 && pressure_hpa <= 1200) {
                scd41_set_ambient_pressure(pressure_hpa);
            }
#endif
        } else {
            printf("[DPS310] Failed to read data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

        printf("\n");

        // 等待下次测量
        vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
    }
}

/**
 * @brief 串口命令处理任务
 */
static void console_task(void *pvParameters) {
    char line[128];
    int line_pos = 0;

    ESP_LOGI(TAG, "Console task started");
    ESP_LOGI(TAG, "Commands:");
    ESP_LOGI(TAG, "  set_alt_ref <altitude_m> - Set altitude reference (e.g., set_alt_ref 7.3)");

    while (1) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (c == '\r' || c == '\n') {
            if (line_pos > 0) {
                line[line_pos] = '\0';

                // 处理命令
                if (strncmp(line, "set_alt_ref ", 12) == 0) {
                    printf("\n");  // 命令后换行
                    float altitude = atof(line + 12);

#if ENABLE_DPS310
                    esp_err_t ret = dps310_legacy_set_altitude_ref(altitude);
                    if (ret == ESP_OK) {
                        altitude_calibrated = true;

                        // 读取当前气压作为相对高度参考
                        dps310_data_t dps310_data;
                        if (dps310_legacy_read_data(&dps310_data) == ESP_OK) {
                            reference_pressure = dps310_data.pressure;
                            ESP_LOGI(TAG, "Altitude reference set to %.2f m", altitude);
                            ESP_LOGI(TAG, "Current pressure: %.2f hPa", reference_pressure / 100.0f);
                        }
                    } else {
                        ESP_LOGE(TAG, "Failed to set altitude reference");
                    }
#else
                    ESP_LOGW(TAG, "DPS310 is not enabled");
#endif
                } else if (strncmp(line, "set_voc_baseline ", 17) == 0) {
                    printf("\n");
                    float baseline = atof(line + 17);
#if ENABLE_SGP41
                    esp_err_t ret = sgp41_set_voc_baseline(baseline);
                    if (ret == ESP_OK) {
                        printf("VOC baseline set to: %.0f\n", baseline);
                        printf("New thresholds: Index 100=%.0f, 200=%.0f, 300=%.0f, 400=%.0f\n",
                               baseline, baseline + 5000.0f, baseline + 10000.0f, baseline + 15000.0f);
                    } else {
                        printf("Failed to set VOC baseline (valid range: 10000-60000)\n");
                    }
#else
                    printf("SGP41 is not enabled\n");
#endif
                } else if (strncmp(line, "set_nox_baseline ", 17) == 0) {
                    printf("\n");
                    float baseline = atof(line + 17);
#if ENABLE_SGP41
                    esp_err_t ret = sgp41_set_nox_baseline(baseline);
                    if (ret == ESP_OK) {
                        printf("NOx baseline set to: %.0f\n", baseline);
                        printf("New thresholds: Index 100=%.0f, 200=%.0f, 300=%.0f, 400=%.0f\n",
                               baseline, baseline + 3000.0f, baseline + 7000.0f, baseline + 12000.0f);
                    } else {
                        printf("Failed to set NOx baseline (valid range: 5000-40000)\n");
                    }
#else
                    printf("SGP41 is not enabled\n");
#endif
                } else if (strcmp(line, "get_baseline") == 0) {
                    printf("\n");
#if ENABLE_SGP41
                    float voc_baseline = sgp41_get_voc_baseline();
                    float nox_baseline = sgp41_get_nox_baseline();
                    printf("Current baselines:\n");
                    printf("  VOC: %.0f (default: 27000)\n", voc_baseline);
                    printf("  NOx: %.0f (default: 15000)\n", nox_baseline);
#else
                    printf("SGP41 is not enabled\n");
#endif
                } else if (strncmp(line, "calibrate_co2 ", 14) == 0) {
                    printf("\n");
                    uint16_t target_co2 = (uint16_t)atoi(line + 14);
#if ENABLE_SCD41
                    if (target_co2 < 400 || target_co2 > 2000) {
                        printf("Invalid CO2 value (valid range: 400-2000 ppm)\n");
                        printf("Typical outdoor fresh air: 400-450 ppm\n");
                    } else {
                        printf("Starting SCD41 forced recalibration to %d ppm...\n", target_co2);
                        printf("WARNING: Sensor must be in stable conditions for 3+ minutes\n");

                        int16_t frc_correction = 0;
                        esp_err_t ret = scd41_perform_forced_calibration(target_co2, &frc_correction);

                        // 检查是否校准失败 (0x8000 = -32768 表示校准错误)
                        if (ret == ESP_OK && frc_correction != -32768) {
                            printf("Calibration successful!\n");
                            printf("FRC correction: %d ppm\n", frc_correction);
                            printf("Restarting measurement...\n");
                            vTaskDelay(pdMS_TO_TICKS(500));
                            scd41_start_periodic_measurement();
                        } else {
                            printf("Calibration FAILED!\n");
                            if (frc_correction == -32768) {
                                printf("Sensor returned error code 0x8000\n");
                            }
                            printf("Possible causes:\n");
                            printf("  1. Sensor not running for 3+ minutes\n");
                            printf("  2. Environment CO2 not stable\n");
                            printf("  3. Target value too different from actual\n");
                            printf("\nTry 'factory_reset' to restore SCD41 defaults\n");
                            // 重启测量
                            vTaskDelay(pdMS_TO_TICKS(500));
                            scd41_start_periodic_measurement();
                        }
                    }
#else
                    printf("SCD41 is not enabled\n");
#endif
                } else if (strcmp(line, "factory_reset") == 0) {
                    printf("\n");
#if ENABLE_SCD41
                    printf("Performing SCD41 factory reset...\n");
                    esp_err_t ret = scd41_factory_reset();
                    if (ret == ESP_OK) {
                        printf("Factory reset successful!\n");
                        printf("Restarting measurement...\n");
                        vTaskDelay(pdMS_TO_TICKS(1200));  // 工厂重置需要等待更长时间
                        scd41_start_periodic_measurement();
                    } else {
                        printf("Factory reset failed: %s\n", esp_err_to_name(ret));
                    }
#else
                    printf("SCD41 is not enabled\n");
#endif
                } else if (strncmp(line, "set_altitude ", 13) == 0) {
                    printf("\n");
                    uint16_t altitude = (uint16_t)atoi(line + 13);
#if ENABLE_SCD41
                    if (altitude > 3000) {
                        printf("Invalid altitude (valid range: 0-3000 m)\n");
                    } else {
                        printf("Setting SCD41 altitude compensation to %d m...\n", altitude);
                        esp_err_t ret = scd41_set_sensor_altitude(altitude);
                        if (ret == ESP_OK) {
                            printf("Altitude set successfully!\n");
                            printf("Restarting measurement...\n");
                            vTaskDelay(pdMS_TO_TICKS(500));
                            scd41_start_periodic_measurement();
                        } else {
                            printf("Failed to set altitude: %s\n", esp_err_to_name(ret));
                        }
                    }
#else
                    printf("SCD41 is not enabled\n");
#endif
                } else if (strncmp(line, "set_temp_offset ", 16) == 0) {
                    printf("\n");
                    float temp_offset = atof(line + 16);
#if ENABLE_SCD41
                    if (temp_offset < -10.0f || temp_offset > 10.0f) {
                        printf("Invalid temperature offset (valid range: -10.0 to 10.0 °C)\n");
                    } else {
                        printf("Setting SCD41 temperature offset to %.2f °C...\n", temp_offset);
                        esp_err_t ret = scd41_set_temperature_offset(temp_offset);
                        if (ret == ESP_OK) {
                            printf("Temperature offset set successfully!\n");
                            printf("Restarting measurement...\n");
                            vTaskDelay(pdMS_TO_TICKS(500));
                            scd41_start_periodic_measurement();
                        } else {
                            printf("Failed to set temperature offset: %s\n", esp_err_to_name(ret));
                        }
                    }
#else
                    printf("SCD41 is not enabled\n");
#endif
                } else if (strcmp(line, "help") == 0) {
                    printf("\n=== AirSense Available Commands ===\n");
                    printf("\n--- SCD41 CO2 Sensor ---\n");
                    printf("  calibrate_co2 <ppm>      - Forced CO2 calibration (400-2000)\n");
                    printf("  factory_reset            - Restore SCD41 to factory defaults\n");
                    printf("  set_temp_offset <celsius>- Manual temp offset (auto at startup)\n");
                    printf("  Note: Pressure & temperature compensation are automatic\n");
                    printf("\n--- SGP41 VOC/NOx Sensor ---\n");
                    printf("  set_voc_baseline <value> - Set VOC baseline (10000-60000)\n");
                    printf("  set_nox_baseline <value> - Set NOx baseline (5000-40000)\n");
                    printf("  get_baseline             - Show current baseline values\n");
                    printf("\n--- DPS310 Pressure Sensor ---\n");
                    printf("  set_alt_ref <altitude>   - Set altitude reference (meters)\n");
                    printf("\n--- General ---\n");
                    printf("  help                     - Show this help message\n");
                    printf("\nNote: SCD41 auto-compensated by DPS310 pressure\n");
                } else if (line_pos > 0) {
                    printf("Unknown command: %s\n", line);
                    printf("Type 'help' for available commands\n");
                }

                line_pos = 0;
            }
        } else if (c == '\b' || c == 0x7F) {  // Backspace
            if (line_pos > 0) {
                line_pos--;
                printf("\b \b");  // 删除字符
            }
        } else if (line_pos < sizeof(line) - 1) {
            line[line_pos++] = c;
            putchar(c);  // 回显字符
        }
    }
}

/**
 * @brief 主函数
 */
void app_main(void) {
    ESP_LOGI(TAG, "AirSense Air Quality Monitor");
    ESP_LOGI(TAG, "MCU: ESP32-C5");
    ESP_LOGI(TAG, "Version: 1.0.0");

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化所有传感器
    init_all_sensors();

    // 创建传感器采集任务
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    // 创建串口命令处理任务
    xTaskCreate(console_task, "console_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "System started successfully");
    ESP_LOGI(TAG, "Type 'help' for available commands");
}
