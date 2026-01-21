#include "uart_echo.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT UART_NUM_0
#define UART_BUF_SIZE 256

static const char *TAG = "UART_ECHO";

/* --- Task privada --- */
static void uart_echo_task(void *arg)
{
    uint8_t data[UART_BUF_SIZE];

    ESP_LOGI(TAG, "UART echo task started");

    while (1) {
        int len = uart_read_bytes(
            UART_PORT,
            data,
            UART_BUF_SIZE - 1,
            pdMS_TO_TICKS(100)
        );

        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Recibido: %s", (char *)data);
            uart_write_bytes(UART_PORT, (const char *)data, len);
        }
    }
}

/* --- Inicialización pública --- */
void uart_echo_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    xTaskCreate(
        uart_echo_task,
        "uart_echo_task",
        4096,
        NULL,
        5,
        NULL
    );
}
