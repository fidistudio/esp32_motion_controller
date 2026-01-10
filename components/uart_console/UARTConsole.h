#pragma once

#include <cstddef>
#include <cstdint>

class UARTConsole {
public:
  void init();
  bool readLine(char *buffer, size_t maxLen);

private:
  static constexpr int UART_PORT = 0;
  static constexpr int RX_BUF_SIZE = 128;

  void trimNewline(char *str);
};
