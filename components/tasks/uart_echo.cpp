#include "uart_echo.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_PORT UART_NUM_0
#define UART_RX_BUF_SIZE 256
#define LINE_BUF_SIZE 128

static const char *TAG = "UART_ECHO";

/* --- Task privada --- */
static void uart_echo_task(void *arg)
{
    uint8_t rx_buf[UART_RX_BUF_SIZE];
    char line_buf[LINE_BUF_SIZE];
    int line_idx = 0;

    bool line_overflow = false;

    ESP_LOGI(TAG, "UART echo task started");

    while (1)
    {
        int len = uart_read_bytes(
            UART_PORT,
            rx_buf,
            sizeof(rx_buf),
            pdMS_TO_TICKS(100));

        for (int i = 0; i < len; i++)
        {
            char c = (char)rx_buf[i];
            if (c == '\n' || c == '\r')
            {
                if (line_overflow)
                    ESP_LOGW(TAG, "Linea descartada: buffer lleno");
                else if (line_idx > 0)
                {
                    line_buf[line_idx] = '\0';

                    ESP_LOGI(TAG, "Linea recibida: %s", line_buf);
                    uart_write_bytes(UART_PORT, line_buf, line_idx);
                    uart_write_bytes(UART_PORT, "\r\n", 2);
                }
                line_idx = 0;
                line_overflow = false;
                continue;
            }
            if (!line_overflow)
            {
                if (line_idx < LINE_BUF_SIZE - 1)
                    line_buf[line_idx++] = c;
                else
                    line_overflow = true;
            }
        }
    }
}

/* --- Inicialización pública --- */
void uart_echo_init(void)
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;

    uart_driver_install(UART_PORT, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
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
        NULL);
}
