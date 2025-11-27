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
 * - Dart WZ-H3-N: HCHO甲醛电化学传感器 (UART1，与S88LP二选一)
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

// esp-idf-lib库头文件
#include <i2cdev.h>  // esp-idf-lib的I2C辅助库
#include <dps310.h>  // esp-idf-lib的DPS310驱动

// 传感器驱动头文件
#include "sensors/senseair_s8.h"  // S88LP驱动 (兼容S8协议)
#include "sensors/sgp41.h"
#include "sensors/dart_wzh3n.h"

static const char *TAG = "AirSense";

// 传感器使能开关 (ESP32-C5验证 - 仅使用DPS310)
#define ENABLE_S88LP            0  // S88LP CO2传感器 (UART1)
#define ENABLE_SGP41            0  // SGP41 VOC/NOx传感器 (I2C0)
#define ENABLE_DART_WZH3N       0  // WZ-H3-N HCHO传感器 (UART1，与S88LP二选一)
#define ENABLE_DPS310           1  // DPS310气压传感器 (I2C0) - 验证中

// 引脚定义 (ESP32-C5-DevKitC-1)
// UART1 - S88LP CO2传感器 或 WZ-H3-N HCHO传感器 (二选一)
#define S88LP_TXD_PIN           GPIO_NUM_5
#define S88LP_RXD_PIN           GPIO_NUM_4

// UART1 - Dart WZ-H3-N (与S88LP共用UART1，不能同时启用)
#define DART_TXD_PIN            GPIO_NUM_5
#define DART_RXD_PIN            GPIO_NUM_4

// I2C0 - SGP41和DPS310共用 (ESP32-C5开发板)
#define I2C0_SDA_PIN            GPIO_NUM_2
#define I2C0_SCL_PIN            GPIO_NUM_3

// 注: 所有I2C传感器通过扩展板共用I2C0
// 注: UART0用于固件烧录和日志输出，不可占用

// 测量间隔 (ms) - 128次过采样测量时间~344ms
#define MEASURE_INTERVAL_MS     3000  // 3秒间隔,给128次过采样足够余量

// DPS310设备描述符和状态（全局变量）
#if ENABLE_DPS310
static dps310_t dps310_dev;
static bool altitude_calibrated = false;  // 海拔是否已校准
static float reference_pressure = 0;      // 参考气压(开机时记录)

// IIR低通滤波器配置 (根据DPS310文档建议)
#define IIR_FILTER_ALPHA 0.2f  // 平滑系数: 0.1=很平滑/慢, 0.5=中等, 0.9=快速响应
static float filtered_pressure = 0;
static bool filter_initialized = false;

// 温度稳定性检测
static float last_temperature = 0;
static bool temperature_stable = false;
#define TEMP_STABLE_THRESHOLD 0.05f  // 温度变化<0.05°C视为稳定
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

#if ENABLE_SGP41
    // 初始化SGP41 (I2C0)
    sgp41_config_t sgp41_config = {
        .i2c_num = I2C_NUM_0,
        .sda_pin = I2C0_SDA_PIN,
        .scl_pin = I2C0_SCL_PIN,
        .i2c_freq_hz = SGP41_I2C_FREQ_HZ,
    };
    ret = sgp41_init(&sgp41_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SGP41 initialized");

        // 获取序列号
        uint8_t serial[6];
        if (sgp41_get_serial_number(serial) == ESP_OK) {
            ESP_LOGI(TAG, "SGP41 Serial: %02X%02X%02X%02X%02X%02X",
                     serial[0], serial[1], serial[2], serial[3], serial[4], serial[5]);
        }

        // 执行自检
        if (sgp41_self_test() == ESP_OK) {
            ESP_LOGI(TAG, "SGP41 self-test passed");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize SGP41: %s", esp_err_to_name(ret));
    }
#endif

#if ENABLE_DART_WZH3N
    // 初始化Dart WZ-H3-N (UART1) - ESP32-C5不支持UART2
    dart_wzh3n_config_t dart_config = {
        .uart_num = UART_NUM_1,
        .tx_pin = DART_TXD_PIN,
        .rx_pin = DART_RXD_PIN,
        .baud_rate = 9600,
        .active_upload = false,
    };
    ret = dart_wzh3n_init(&dart_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Dart WZ-H3-N initialized");
    } else {
        ESP_LOGE(TAG, "Failed to initialize Dart WZ-H3-N: %s", esp_err_to_name(ret));
    }
#endif

#if ENABLE_DPS310
    // 使用esp-idf-lib的DPS310驱动
    ESP_LOGI(TAG, "Initializing DPS310 (esp-idf-lib) on I2C_NUM_0 (SDA:%d, SCL:%d, Addr:0x77)",
             I2C0_SDA_PIN, I2C0_SCL_PIN);

    // 初始化I2C描述符
    memset(&dps310_dev, 0, sizeof(dps310_t));

    // DPS310_I2C_ADDRESS_1 = 0x77 (SDO接VDD)
    // DPS310_I2C_ADDRESS_0 = 0x76 (SDO接GND)
    ret = dps310_init_desc(&dps310_dev, 0x77, I2C_NUM_0, I2C0_SDA_PIN, I2C0_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init DPS310 descriptor: %s", esp_err_to_name(ret));
    } else {
        // 配置DPS310参数 - 使用最高精度128次过采样
        // 128次过采样: 测量时间~344ms, 精度±0.002 hPa (±0.02m)
        dps310_config_t config = {
            .pm_rate = DPS310_PM_RATE_1,           // 1Hz采样率
            .pm_oversampling = DPS310_PM_PRC_128,  // 128次过采样 - 最高精度
            .tmp_rate = DPS310_TMP_RATE_1,         // 1Hz采样率
            .tmp_oversampling = DPS310_TMP_PRC_128,// 128次过采样 - 最高精度
            .tmp_src = DPS310_TMP_SRC_EXTERNAL,    // 使用MEMS内部温度传感器(更准确)
            .tmp_coef = DPS310_TMP_SRC_EXTERNAL,   // 使用外部温度系数
            .int_fifo_mode = DPS310_INT_FIFO_DISABLE,
            .int_tmp_mode = DPS310_INT_TMP_DISABLE,
            .int_prs_mode = DPS310_INT_PRS_DISABLE,
            .t_shift_mode = DPS310_T_SHIFT_ENABLE, // 启用温度结果移位(修正过采样)
            .p_shift_mode = DPS310_P_SHIFT_ENABLE, // 启用气压结果移位(修正过采样)
            .fifo_en_mode = DPS310_FIFO_DISABLE,
            .spi_mode = DPS310_SPI_MODE_4WIRE,
        };

        // 初始化DPS310设备
        ret = dps310_init(&dps310_dev, &config);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "DPS310 initialized successfully");

            // 启动连续测量模式
            ret = dps310_set_mode(&dps310_dev, DPS310_MODE_BACKGROUND_ALL);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "DPS310 continuous measurement mode started");
                ESP_LOGI(TAG, "Use 'set_alt_ref <altitude>' command to calibrate altitude");
                ESP_LOGI(TAG, "Example: set_alt_ref 7.3");
            } else {
                ESP_LOGW(TAG, "Failed to start continuous mode: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize DPS310: %s (0x%x)", esp_err_to_name(ret), ret);
            ESP_LOGW(TAG, "Troubleshooting:");
            ESP_LOGW(TAG, "  - Check I2C connections (SDA=GPIO%d, SCL=GPIO%d)", I2C0_SDA_PIN, I2C0_SCL_PIN);
            ESP_LOGW(TAG, "  - Verify sensor power (VDD=3.3V, GND connected)");
            ESP_LOGW(TAG, "  - Check SDO pin connection (0x77=VDD, 0x76=GND)");
            ESP_LOGW(TAG, "  - Add 2.2k-10k pullup resistors on SDA/SCL");
        }
    }
#endif

    ESP_LOGI(TAG, "Sensor initialization complete");
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
        // 读取SGP41 - VOC/NOx
        sgp41_data_t sgp41_data;
        // 使用默认温湿度补偿值: 25°C, 50%RH
        if (sgp41_measure_raw(50, 25, &sgp41_data) == ESP_OK && sgp41_data.valid) {
            printf("[SGP41] VOC raw: %d, NOx raw: %d, VOC index: %ld, NOx index: %ld\n",
                   sgp41_data.sraw_voc, sgp41_data.sraw_nox,
                   (long)sgp41_data.voc_index, (long)sgp41_data.nox_index);
        } else {
            printf("[SGP41] Failed to read VOC/NOx\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

#if ENABLE_DART_WZH3N
        // 读取Dart WZ-H3-N - NH3
        dart_wzh3n_data_t dart_data;
        if (dart_wzh3n_read_concentration(&dart_data) == ESP_OK && dart_data.valid) {
            printf("[Dart] NH3: %.2f ppm, Full Range: %d ppm\n",
                   dart_data.nh3_ppm, dart_data.full_range);
        } else {
            printf("[Dart] Failed to read NH3\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
#endif

#if ENABLE_DPS310
        // 读取DPS310 - 气压/温度/海拔
        float pressure, temperature;

        esp_err_t temp_ret = dps310_read_temp(&dps310_dev, &temperature);
        esp_err_t pres_ret = dps310_read_pressure(&dps310_dev, &pressure);

        if (temp_ret == ESP_OK && pres_ret == ESP_OK) {
            // 检测温度稳定性
            float temp_change = fabsf(temperature - last_temperature);
            temperature_stable = (temp_change < TEMP_STABLE_THRESHOLD);
            last_temperature = temperature;

            // 应用IIR低通滤波器平滑气压数据(根据DPS310文档建议)
            float raw_pressure = pressure;
            pressure = iir_lowpass_filter(pressure);

            // 记录开机时的参考气压(用于相对高度计算)
            if (reference_pressure == 0) {
                reference_pressure = pressure;
                ESP_LOGI(TAG, "Reference pressure set to %.2f hPa at startup", reference_pressure / 100.0f);
            }

            // 计算相对高度变化(相对于开机位置)
            float height_change = (reference_pressure - pressure) / 12.0f;

            // 温度稳定性标记
            const char *temp_status = temperature_stable ? "STABLE" : "UNSTABLE";

            if (altitude_calibrated) {
                // 如果已校准,显示绝对海拔
                float altitude;
                esp_err_t alt_ret = dps310_read_altitude(&dps310_dev, &altitude);
                if (alt_ret == ESP_OK) {
                    printf("[DPS310] Temp: %.2f°C (ΔT=%.3f, %s), P_raw: %.2f, P_filt: %.2f hPa, Alt: %.2f m, Δh: %+.2f m\n",
                           temperature, temp_change, temp_status, raw_pressure / 100.0f, pressure / 100.0f, altitude, height_change);
                } else {
                    printf("[DPS310] Temp: %.2f°C (ΔT=%.3f, %s), P_raw: %.2f, P_filt: %.2f hPa, Δh: %+.2f m (alt fail)\n",
                           temperature, temp_change, temp_status, raw_pressure / 100.0f, pressure / 100.0f, height_change);
                }
            } else {
                // 未校准,只显示相对高度
                printf("[DPS310] Temp: %.2f°C (ΔT=%.3f, %s), P_raw: %.2f, P_filt: %.2f hPa, Δh: %+.2f m (uncalib)\n",
                       temperature, temp_change, temp_status, raw_pressure / 100.0f, pressure / 100.0f, height_change);
            }
        } else {
            if (temp_ret != ESP_OK) {
                printf("[DPS310] Failed to read temperature: %s\n", esp_err_to_name(temp_ret));
            }
            if (pres_ret != ESP_OK) {
                printf("[DPS310] Failed to read pressure: %s\n", esp_err_to_name(pres_ret));
            }
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
                    // 先读取当前气压,确保传感器已稳定
                    float current_pressure;
                    esp_err_t ret = dps310_read_pressure(&dps310_dev, &current_pressure);
                    if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to read pressure for calibration: %s", esp_err_to_name(ret));
                    } else {
                        ESP_LOGI(TAG, "Current pressure: %.2f hPa", current_pressure / 100.0f);
                        ESP_LOGI(TAG, "Setting altitude reference to %.2f m", altitude);

                        // 在校准前先停止连续测量模式,避免在BACKGROUND模式下修改oversampling导致数据异常
                        ret = dps310_set_mode(&dps310_dev, DPS310_MODE_STANDBY);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to set STANDBY mode: %s", esp_err_to_name(ret));
                        } else {
                            ESP_LOGI(TAG, "Sensor set to STANDBY mode for calibration");
                            vTaskDelay(pdMS_TO_TICKS(100));  // 等待传感器稳定

                            ret = dps310_calibrate_altitude(&dps310_dev, altitude);
                            if (ret == ESP_OK) {
                                // 校准成功后,恢复连续测量模式
                                ret = dps310_set_mode(&dps310_dev, DPS310_MODE_BACKGROUND_ALL);
                                if (ret == ESP_OK) {
                                    altitude_calibrated = true;
                                    // 重置相对高度参考点为当前气压
                                    reference_pressure = current_pressure;
                                    ESP_LOGI(TAG, "Altitude calibration successful!");
                                    ESP_LOGI(TAG, "Sensor mode restored to continuous measurement");
                                    ESP_LOGI(TAG, "Relative height reference reset to current position");
                                } else {
                                    ESP_LOGE(TAG, "Failed to restore sensor mode: %s", esp_err_to_name(ret));
                                }
                            } else {
                                ESP_LOGE(TAG, "Altitude calibration failed: %s", esp_err_to_name(ret));
                                // 即使失败也要恢复连续测量模式
                                dps310_set_mode(&dps310_dev, DPS310_MODE_BACKGROUND_ALL);
                            }
                        }
                    }
#else
                    ESP_LOGW(TAG, "DPS310 is not enabled");
#endif
                } else if (strcmp(line, "help") == 0) {
                    printf("\nAvailable commands:\n");
                    printf("  set_alt_ref <altitude> - Set altitude reference in meters\n");
                    printf("  help                   - Show this help message\n");
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

    // 初始化esp-idf-lib的I2C库
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_LOGI(TAG, "I2C library initialized");

    // 初始化所有传感器
    init_all_sensors();

    // 创建传感器采集任务
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    // 创建串口命令处理任务
    xTaskCreate(console_task, "console_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "System started successfully");
    ESP_LOGI(TAG, "Type 'help' for available commands");
}
