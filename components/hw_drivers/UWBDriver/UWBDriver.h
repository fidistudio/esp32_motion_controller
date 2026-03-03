#pragma once

#include "driver/uart.h"
#include <cstdint>
#include <string>

class UWBDriver {
public:
  // Constructor: uart_num (UART0,1,2), pines TX/RX y baudrate opcional
  UWBDriver(uart_port_t uart_num, int tx_pin, int rx_pin,
            int baud_rate = 115200);

  // Envía comando al tag y devuelve distancia en metros
  bool getDistance(const char *tag_id, float &distance_m);

private:
  uart_port_t uart_num_;
  static constexpr int RX_BUF_SIZE = 256;

  bool sendCommand(const std::string &cmd);
  bool readLine(std::string &line, uint32_t timeout_ms);
  bool parseDistance(const std::string &line, float &distance_m);
};
