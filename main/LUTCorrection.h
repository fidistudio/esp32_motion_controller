#pragma once

#include "Encoder.h"
#include <stdbool.h>
#include <stdint.h>
#include <string>

extern "C" {
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
}

class LUTCorrection {
public:
  LUTCorrection(encoder_state_t *encoder_state,
                const std::string &nvs_namespace);

  void saveLUT();
  void loadLUT();
  void printLUT();

private:
  encoder_state_t *state_;
  std::string nvs_namespace_;
  const char *TAG = "LUTCorrection";
};
