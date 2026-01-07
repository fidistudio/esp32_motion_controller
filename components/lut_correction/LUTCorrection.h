#pragma once

#include "Encoder.h"
#include <string>

extern "C" {
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
}

// ------------------------------------------------
// LUTCorrection
// Responsible only for persistence of LUT data
// ------------------------------------------------

class LUTCorrection {
public:
  LUTCorrection(EncoderRuntimeState *encoderState,
                const std::string &nvsNamespace);

  void save();
  void load();
  void print() const;

private:
  EncoderRuntimeState *state_;
  std::string nvsNamespace_;

  static constexpr const char *TAG = "LUTCorrection";

  bool openNVS(nvs_handle_t &handle, nvs_open_mode mode) const;
};
