/**
 * @file prosense_wzh3n.c
 * @brief Prosense WZ-H3-N 甲醛传感器驱动实现
 *
 * 使用ESP32-C5的LP_UART硬件串口
 */

#include "prosense_wzh3n.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "soc/uart_periph.h"

static const char *TAG = "WZ-H3-N";

// 帧大小
#define PROSENSE_FRAME_SIZE         9

// LP_UART配置
#define LP_UART_NUM                 LP_UART_NUM_0
#define LP_UART_BUF_SIZE            256

static bool s_active_upload = false;
static bool s_initialized = false;

/**
 * @brief 计算校验和
 */
static uint8_t calculate_checksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (~sum) + 1;  // 取反加1
}

/**
 * @brief 解析问答模式响应帧
 *
 * 问答模式响应帧格式 (9字节):
 * [0] 起始位: 0xFF
 * [1] 命令: 0x86
 * [2] 气体浓度高位 (ug/m³)
 * [3] 气体浓度低位 (ug/m³)
 * [4] 保留
 * [5] 保留
 * [6] 气体浓度高位 (ppb)
 * [7] 气体浓度低位 (ppb)
 * [8] 校验值
 */
static esp_err_t parse_qa_frame(const uint8_t *frame, prosense_wzh3n_data_t *data) {
    // 验证命令字节
    if (frame[1] != 0x86) {
        ESP_LOGE(TAG, "Invalid command byte: 0x%02X (expected 0x86)", frame[1]);
        return ESP_FAIL;
    }

    // 验证校验和 (byte[1] 到 byte[7] 求和后取反加1)
    uint8_t checksum = calculate_checksum(&frame[1], 7);
    if (checksum != frame[8]) {
        ESP_LOGE(TAG, "Checksum error: calc=0x%02X, recv=0x%02X", checksum, frame[8]);
        return ESP_FAIL;
    }

    // 解析浓度值
    data->hcho_ugm3 = (frame[2] << 8) | frame[3];  // ug/m³
    data->hcho_ppb = (frame[6] << 8) | frame[7];   // ppb

    // 转换为常用单位
    data->hcho_mgm3 = data->hcho_ugm3 / 1000.0f;  // ug/m³ -> mg/m³
    data->hcho_ppm = data->hcho_ppb / 1000.0f;    // ppb -> ppm

    data->full_range = 0;
    data->gas_name = 0;
    data->unit = 0;
    data->decimal_places = 0;
    data->valid = true;

    ESP_LOGD(TAG, "Q&A Mode - HCHO: %d ug/m³ (%.3f mg/m³), %d ppb (%.3f ppm)",
             data->hcho_ugm3, data->hcho_mgm3, data->hcho_ppb, data->hcho_ppm);

    return ESP_OK;
}

/**
 * @brief 解析主动上传模式数据帧
 *
 * 主动上传帧格式 (9字节):
 * [0] 起始位: 0xFF
 * [1] 气体名称: 0x11 (HCHO) 或 0x17 (CH2O)
 * [2] 单位: 0x04 (ppb) 或 0x10 (ug/m³)
 * [3] 小数位数
 * [4] 气体浓度高位
 * [5] 气体浓度低位
 * [6] 满量程高位
 * [7] 满量程低位
 * [8] 校验值
 */
static esp_err_t parse_active_frame(const uint8_t *frame, prosense_wzh3n_data_t *data) {
    // 验证气体名称 (HCHO = 0x11 或 CH2O = 0x17)
    if (frame[1] != PROSENSE_GAS_HCHO && frame[1] != PROSENSE_GAS_CH2O) {
        ESP_LOGW(TAG, "Unknown gas name: 0x%02X (expected 0x11 or 0x17 for HCHO)", frame[1]);
    }

    // 验证校验和 (byte[1] 到 byte[7] 求和后取反加1)
    uint8_t checksum = calculate_checksum(&frame[1], 7);
    if (checksum != frame[8]) {
        ESP_LOGE(TAG, "Checksum error: calc=0x%02X, recv=0x%02X", checksum, frame[8]);
        return ESP_FAIL;
    }

    // 解析帧数据
    data->gas_name = frame[1];
    data->unit = frame[2];
    data->decimal_places = frame[3];

    // 浓度值 (高字节 + 低字节)
    uint16_t concentration = (frame[4] << 8) | frame[5];

    // 满量程值
    data->full_range = (frame[6] << 8) | frame[7];

    // 根据单位和小数位数转换浓度
    float raw_value = (float)concentration;
    for (int i = 0; i < data->decimal_places; i++) {
        raw_value /= 10.0f;
    }

    if (data->unit == 0x04) {  // ppb
        data->hcho_ppb = concentration;
        data->hcho_ppm = raw_value / 1000.0f;
        // 估算 ug/m³ (HCHO: 1 ppb ≈ 1.25 ug/m³ at 25°C)
        data->hcho_ugm3 = (uint16_t)(raw_value * 1.25f);
        data->hcho_mgm3 = data->hcho_ugm3 / 1000.0f;
    } else if (data->unit == 0x10) {  // ug/m³
        data->hcho_ugm3 = concentration;
        data->hcho_mgm3 = raw_value / 1000.0f;
        // 估算 ppb (HCHO: 1 ug/m³ ≈ 0.8 ppb at 25°C)
        data->hcho_ppb = (uint16_t)(raw_value * 0.8f);
        data->hcho_ppm = data->hcho_ppb / 1000.0f;
    } else {
        // 其他单位，暂时直接赋值
        data->hcho_ppb = concentration;
        data->hcho_ppm = raw_value;
        data->hcho_ugm3 = 0;
        data->hcho_mgm3 = 0;
    }

    data->valid = true;

    ESP_LOGD(TAG, "Active Mode - HCHO: %d ppb (%.3f ppm), Full Range: %d",
             data->hcho_ppb, data->hcho_ppm, data->full_range);

    return ESP_OK;
}

esp_err_t prosense_wzh3n_init(const prosense_wzh3n_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_active_upload = config->active_upload;

    // 配置LP_UART参数
    // LP_UART必须使用LP_UART_SCLK_DEFAULT时钟源
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = LP_UART_SCLK_DEFAULT,  // LP_UART专用时钟源
    };

    // 安装UART驱动
    esp_err_t ret = uart_driver_install(LP_UART_NUM, LP_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install LP_UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(LP_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LP_UART: %s", esp_err_to_name(ret));
        uart_driver_delete(LP_UART_NUM);
        return ret;
    }

    // LP_UART引脚是固定的 (GPIO4=RX, GPIO5=TX)，不需要调用uart_set_pin
    // 但仍然调用以确保引脚配置正确
    ret = uart_set_pin(LP_UART_NUM, config->tx_pin, config->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "uart_set_pin returned %s (LP_UART pins are fixed)", esp_err_to_name(ret));
        // LP_UART引脚固定，忽略此错误继续
    }

    s_initialized = true;

    // 如果配置为问答模式，发送切换命令
    if (!s_active_upload) {
        // 等待传感器稳定
        vTaskDelay(pdMS_TO_TICKS(500));

        // 切换到问答模式: FF 01 78 41 00 00 00 00 46
        uint8_t switch_cmd[9] = {0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46};
        uart_write_bytes(LP_UART_NUM, switch_cmd, sizeof(switch_cmd));

        vTaskDelay(pdMS_TO_TICKS(100));
        uart_flush_input(LP_UART_NUM);  // 清空可能的响应

        ESP_LOGI(TAG, "Switched to Q&A mode");
    }

    ESP_LOGI(TAG, "Prosense WZ-H3-N initialized on LP_UART (TX:GPIO%d, RX:GPIO%d, Mode:%s)",
             config->tx_pin, config->rx_pin,
             s_active_upload ? "Active Upload" : "Q&A");

    return ESP_OK;
}

esp_err_t prosense_wzh3n_read_concentration(prosense_wzh3n_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    data->valid = false;

    // 构造读取浓度命令: FF 01 86 00 00 00 00 00 79
    uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

    // 重试机制
    for (int retry = 0; retry < 3; retry++) {
        // 清空接收缓冲区
        uart_flush_input(LP_UART_NUM);

        // 发送命令
        int sent = uart_write_bytes(LP_UART_NUM, cmd, sizeof(cmd));
        if (sent != sizeof(cmd)) {
            ESP_LOGE(TAG, "Failed to send command");
            continue;
        }

        // 等待响应
        vTaskDelay(pdMS_TO_TICKS(150));

        // 读取响应到缓冲区
        uint8_t buffer[20];
        int total_len = uart_read_bytes(LP_UART_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(500));

        if (total_len < PROSENSE_FRAME_SIZE) {
            ESP_LOGD(TAG, "Retry %d: only %d bytes received", retry, total_len);
            continue;
        }

        // 在缓冲区中查找有效帧 (起始位0xFF + 命令0x86)
        for (int i = 0; i <= total_len - PROSENSE_FRAME_SIZE; i++) {
            if (buffer[i] == 0xFF && buffer[i + 1] == 0x86) {
                // 验证校验和
                uint8_t checksum = calculate_checksum(&buffer[i + 1], 7);
                if (checksum == buffer[i + 8]) {
                    return parse_qa_frame(&buffer[i], data);
                }
            }
        }
    }

    ESP_LOGW(TAG, "Failed to read concentration after retries");
    return ESP_FAIL;
}

esp_err_t prosense_wzh3n_read_active(prosense_wzh3n_data_t *data, uint32_t timeout_ms) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    data->valid = false;

    // 重试机制 - 最多尝试3次
    for (int retry = 0; retry < 3; retry++) {
        // 清空缓冲区，等待新的完整帧
        uart_flush_input(LP_UART_NUM);

        // 传感器每秒发送一次，等待足够时间接收完整帧
        vTaskDelay(pdMS_TO_TICKS(1200));

        // 读取缓冲区中所有数据
        uint8_t buffer[32];
        int total_len = uart_read_bytes(LP_UART_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(300));

        if (total_len <= 0) {
            ESP_LOGD(TAG, "Retry %d: no data", retry);
            continue;
        }

        // 至少需要9字节才能解析
        if (total_len < PROSENSE_FRAME_SIZE) {
            continue;
        }

        // 在缓冲区中查找有效的9字节帧
        // 帧格式: [0]FF [1]气体 [2]单位 [3]小数 [4]浓度高 [5]浓度低 [6]量程高 [7]量程低 [8]校验
        for (int i = 0; i <= total_len - 9; i++) {
            if (buffer[i] == 0xFF &&
                (buffer[i + 1] == PROSENSE_GAS_HCHO || buffer[i + 1] == PROSENSE_GAS_CH2O)) {

                // 验证9字节帧校验和
                uint8_t checksum = calculate_checksum(&buffer[i + 1], 7);
                if (checksum == buffer[i + 8]) {
                    return parse_active_frame(&buffer[i], data);
                }
            }
        }

        ESP_LOGD(TAG, "Retry %d: no valid frame found", retry);
    }

    ESP_LOGW(TAG, "Failed to read valid frame after retries");
    return ESP_FAIL;
}

esp_err_t prosense_wzh3n_deinit(void) {
    if (!s_initialized) {
        return ESP_OK;
    }

    uart_driver_delete(LP_UART_NUM);
    s_initialized = false;
    ESP_LOGI(TAG, "WZ-H3-N deinitialized");
    return ESP_OK;
}
