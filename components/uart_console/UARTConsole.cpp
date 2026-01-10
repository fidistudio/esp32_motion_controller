#include "UARTConsole.h"

#include <cstring>

void UARTConsole::init() {

  uart_config_t uartConfig{};
  uartConfig.baud_rate = 115200;
  uartConfig.data_bits = UART_DATA_8_BITS;
  uartConfig.parity = UART_PARITY_DISABLE;
  uartConfig.stop_bits = UART_STOP_BITS_1;
  uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uartConfig.source_clk = UART_SCLK_DEFAULT;

  uart_driver_install(UART_PORT, RX_BUF_SIZE * 2, 0, 0, nullptr, 0);

  uart_param_config(UART_PORT, &uartConfig);
}

bool UARTConsole::readLine(char *buffer, size_t maxLen) {

  int len = uart_read_bytes(UART_PORT, buffer, maxLen - 1, pdMS_TO_TICKS(50));

  if (len <= 0) {
    return false;
  }

  buffer[len] = '\0';
  trimNewline(buffer);
  return true;
}

void UARTConsole::trimNewline(char *str) {
  for (char *p = str; *p; ++p) {
    if (*p == '\r' || *p == '\n') {
      *p = '\0';
      return;
    }
  }
}
