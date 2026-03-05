#include "UWBDriver.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdlib> // atof o std::stof
#include <cstring>
#include <string>

static const char *TAG = "UWBDriver";

UWBDriver::UWBDriver(uart_port_t uart_num, int tx_pin, int rx_pin,
                     int baud_rate)
    : uart_num_(uart_num) {
  uart_config_t uart_config = {};
  uart_config.baud_rate = baud_rate;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_APB;

  // Instala driver UART
  uart_driver_install(uart_num_, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(uart_num_, &uart_config);
  uart_set_pin(uart_num_, tx_pin, rx_pin, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
}

// ------------------ Enviar comando AT ------------------
bool UWBDriver::sendCommand(const std::string &cmd) {
  std::string full_cmd = cmd + "\r\n";
  int written =
      uart_write_bytes(uart_num_, full_cmd.c_str(), full_cmd.length());
  return written == full_cmd.length();
}

// ------------------ Leer línea con timeout ------------------
bool UWBDriver::readLine(std::string &line, uint32_t timeout_ms) {
  uint8_t c;
  line.clear();

  int64_t start = esp_timer_get_time(); // microsegundos

  while (true) {
    int len = uart_read_bytes(uart_num_, &c, 1, pdMS_TO_TICKS(10));

    if (len > 0) {
      if (c == '\n')
        return true;
      if (c != '\r')
        line += (char)c;
    }

    int64_t now = esp_timer_get_time();
    if ((now - start) / 1000 > timeout_ms)
      return false;
  }
}

// ------------------ Parsear distancia ------------------
bool UWBDriver::parseDistance(const std::string &line, float &distance_m) {
  size_t pos = line.find(" cm");
  if (pos == std::string::npos)
    return false;

  size_t start = line.rfind(",", pos);
  if (start == std::string::npos)
    return false;

  std::string number = line.substr(start + 1, pos - start - 1);

  float cm = std::stof(number);
  distance_m = cm / 100.0f; // convertir a metros
  return true;
}

// ------------------ Función pública ------------------
bool UWBDriver::getDistance(const char *tag_id, float &distance_m) {

  std::string cmd = "AT+ANCHOR_SEND=";
  cmd += tag_id;
  cmd += ",4,TEST";

  if (!sendCommand(cmd))
    return false;

  std::string line;
  std::string expected = "+ANCHOR_RCV=";
  expected += tag_id;

  int64_t start = esp_timer_get_time();

  while (true) {

    if (!readLine(line, 200))
      return false;

    if (line.find(expected) != std::string::npos) {
      return parseDistance(line, distance_m);
    }

    // timeout total de 1s
    if ((esp_timer_get_time() - start) > 1000000)
      return false;
  }
}
