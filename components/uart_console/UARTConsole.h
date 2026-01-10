#pragma once

#include "driver/uart.h"
#include <cstddef>

class UARTConsole {
public:
  void init();
  bool readLine(char *buffer, size_t maxLen);

private:
  static constexpr uart_port_t UART_PORT = UART_NUM_0;
  static constexpr int RX_BUF_SIZE = 128;

  void trimNewline(char *str);
};
