/**
 * @file dart_wzh3n.c
 * @brief Dart WZ-H3-N 电化学传感器驱动实现
 */

#include "dart_wzh3n.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Dart_WZH3N";

#define UART_BUF_SIZE           256
#define DART_FRAME_SIZE         9

static uart_port_t s_uart_num = UART_NUM_1;  // ESP32-C5 only has UART0/1
static bool s_active_upload = false;

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
 * @brief 解析Dart传感器数据帧
 * 帧格式: [起始位0xFF] [起始位0x01/0x86/0x87] [气体名称] [单位] [小数位] [浓度高] [浓度低] [满量程高] [满量程低] [校验和]
 */
static esp_err_t parse_dart_frame(const uint8_t *frame, dart_wzh3n_data_t *data) {
    if (frame[0] != 0xFF) {
        ESP_LOGE(TAG, "Invalid frame header: 0x%02X", frame[0]);
        return ESP_FAIL;
    }

    // 验证校验和
    uint8_t checksum = calculate_checksum(frame, DART_FRAME_SIZE - 1);
    if (checksum != frame[DART_FRAME_SIZE - 1]) {
        ESP_LOGE(TAG, "Checksum error: calc=0x%02X, recv=0x%02X", checksum, frame[DART_FRAME_SIZE - 1]);
        return ESP_FAIL;
    }

    // 解析数据
    data->unit = frame[3];
    data->decimal_places = frame[4];

    // 浓度值 (高字节 + 低字节)
    uint16_t concentration = (frame[5] << 8) | frame[6];

    // 满量程值
    data->full_range = (frame[7] << 8) | frame[8];

    // 根据小数位数转换浓度
    data->nh3_ppm = concentration;
    for (int i = 0; i < data->decimal_places; i++) {
        data->nh3_ppm /= 10.0;
    }

    data->valid = true;

    ESP_LOGD(TAG, "NH3: %.2f ppm, Unit: %d, Decimals: %d, Full Range: %d ppm",
             data->nh3_ppm, data->unit, data->decimal_places, data->full_range);

    return ESP_OK;
}

esp_err_t dart_wzh3n_init(const dart_wzh3n_config_t *config) {
    if (config == NULL) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    s_uart_num = config->uart_num;
    s_active_upload = config->active_upload;

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

    ESP_LOGI(TAG, "Dart WZ-H3-N initialized on UART%d (TX:%d, RX:%d, Mode:%s)",
             s_uart_num, config->tx_pin, config->rx_pin,
             s_active_upload ? "Active Upload" : "Q&A");

    return ESP_OK;
}

esp_err_t dart_wzh3n_read_concentration(dart_wzh3n_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    data->valid = false;

    // 构造读取浓度命令
    uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

    // 清空接收缓冲区
    uart_flush_input(s_uart_num);

    // 发送命令
    int sent = uart_write_bytes(s_uart_num, cmd, sizeof(cmd));
    if (sent != sizeof(cmd)) {
        ESP_LOGE(TAG, "Failed to send command");
        return ESP_FAIL;
    }

    // 等待响应
    vTaskDelay(pdMS_TO_TICKS(100));

    // 读取响应
    uint8_t response[UART_BUF_SIZE];
    int read_len = uart_read_bytes(s_uart_num, response, DART_FRAME_SIZE, pdMS_TO_TICKS(1000));

    if (read_len != DART_FRAME_SIZE) {
        ESP_LOGE(TAG, "Invalid response length: %d", read_len);
        return ESP_FAIL;
    }

    return parse_dart_frame(response, data);
}

esp_err_t dart_wzh3n_read_active(dart_wzh3n_data_t *data, uint32_t timeout_ms) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    data->valid = false;

    // 等待主动上传的数据帧
    uint8_t response[UART_BUF_SIZE];
    int read_len = uart_read_bytes(s_uart_num, response, DART_FRAME_SIZE, pdMS_TO_TICKS(timeout_ms));

    if (read_len != DART_FRAME_SIZE) {
        ESP_LOGE(TAG, "No data received or invalid length: %d", read_len);
        return ESP_FAIL;
    }

    return parse_dart_frame(response, data);
}

esp_err_t dart_wzh3n_deinit(void) {
    uart_driver_delete(s_uart_num);
    ESP_LOGI(TAG, "Dart WZ-H3-N deinitialized");
    return ESP_OK;
}
