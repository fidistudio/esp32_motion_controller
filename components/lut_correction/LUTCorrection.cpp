#include "LUTCorrection.h"
#include <stdio.h>
#include <string.h>

// ---------------- Constructor ----------------

LUTCorrection::LUTCorrection(EncoderRuntimeState *encoderState,
                             const std::string &nvsNamespace)
    : state_(encoderState), nvsNamespace_(nvsNamespace) {
  // NVS must be initialized elsewhere (app_main)
}

// ---------------- Public API ----------------

void LUTCorrection::save() {
  nvs_handle_t nvs;
  if (!openNVS(nvs, NVS_READWRITE)) {
    return;
  }

  float lut[PULSES_PER_REV][2] = {};

  // Preserve the opposite direction if it already exists
  size_t lutSize = sizeof(lut);
  esp_err_t err = nvs_get_blob(nvs, "lut", lut, &lutSize);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "No previous LUT found, creating new one");
  }

  const int dirIndex = state_->isDirectionInverted ? 1 : 0;
  for (int i = 0; i < PULSES_PER_REV; i++) {
    lut[i][dirIndex] = state_->sectorCorrection[i][dirIndex];
  }

  err = nvs_set_blob(nvs, "lut", lut, sizeof(lut));
  if (err == ESP_OK) {
    nvs_commit(nvs);
    ESP_LOGI(TAG, "LUT (%s) saved to NVS",
             state_->isDirectionInverted ? "INVERTED" : "NORMAL");
  } else {
    ESP_LOGE(TAG, "Failed to save LUT: %s", esp_err_to_name(err));
  }

  nvs_close(nvs);
}

void LUTCorrection::load() {
  nvs_handle_t nvs;
  if (!openNVS(nvs, NVS_READONLY)) {
    return;
  }

  float lut[PULSES_PER_REV][2];
  size_t lutSize = sizeof(lut);

  esp_err_t err = nvs_get_blob(nvs, "lut", lut, &lutSize);
  if (err == ESP_OK) {
    for (int i = 0; i < PULSES_PER_REV; i++) {
      state_->sectorCorrection[i][0] = lut[i][0];
      state_->sectorCorrection[i][1] = lut[i][1];
    }
    ESP_LOGI(TAG, "LUT (NORMAL + INVERTED) loaded from NVS");
  } else {
    ESP_LOGW(TAG, "No LUT found in NVS (%s)", esp_err_to_name(err));
  }

  nvs_close(nvs);
}

void LUTCorrection::print() const {
  nvs_handle_t nvs;
  if (!openNVS(nvs, NVS_READONLY)) {
    return;
  }

  float lut[PULSES_PER_REV][2];
  size_t lutSize = sizeof(lut);

  esp_err_t err = nvs_get_blob(nvs, "lut", lut, &lutSize);
  if (err == ESP_OK) {
    printf("\n===== LUT stored in NVS =====\n");
    for (int i = 0; i < PULSES_PER_REV; i++) {
      printf("Sector %02d | + %.3f | - %.3f\n", i, lut[i][0], lut[i][1]);
    }
    printf("============================\n\n");
  } else {
    ESP_LOGW(TAG, "No LUT found in NVS (%s)", esp_err_to_name(err));
  }

  nvs_close(nvs);
}

// ---------------- Private Helpers ----------------

bool LUTCorrection::openNVS(nvs_handle_t &handle, nvs_open_mode mode) const {
  esp_err_t err = nvs_open(nvsNamespace_.c_str(), mode, &handle);

  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to open NVS (%s)", esp_err_to_name(err));
    return false;
  }
  return true;
}
