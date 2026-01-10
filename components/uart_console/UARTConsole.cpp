#include "UARTConsole.h"

#include "driver/uart.h"
#include <cstring>

void UARTConsole::init() {

  const uart_config_t uartConfig = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };

  uart_driver_install(UART_PORT, RX_BUF_SIZE * 2, 0, 0, nullptr, 0);
  uart_param_config(UART_PORT, &uartConfig);
}

bool UARTConsole::readLine(char *buffer, size_t maxLen) {

  int len = uart_read_bytes(UART_PORT, reinterpret_cast<uint8_t *>(buffer),
                            maxLen - 1, pdMS_TO_TICKS(50));

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
