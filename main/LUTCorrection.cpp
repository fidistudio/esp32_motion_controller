#include "LUTCorrection.h"
#include <string.h>

LUTCorrection::LUTCorrection(encoder_state_t *encoder_state,
                             const std::string &nvs_namespace)
    : state_(encoder_state), nvs_namespace_(nvs_namespace) {
  // NVS should already be initialized in app_main
}

// ---------------- Save LUT to NVS ----------------
void LUTCorrection::saveLUT() {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open(nvs_namespace_.c_str(), NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    return;
  }

  float lut[PULSES_PER_REV][2];
  memset(lut, 0, sizeof(lut));

  // Load existing LUT if available (so we don’t overwrite the other direction)
  size_t lut_size = sizeof(lut);
  err = nvs_get_blob(nvs, "lut", lut, &lut_size);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "No previous LUT found, creating new one...");
  }

  int dir_idx = state_->direction_inverted ? 1 : 0;
  for (int i = 0; i < PULSES_PER_REV; i++) {
    lut[i][dir_idx] = state_->sector_correction_factor[i][dir_idx];
  }

  err = nvs_set_blob(nvs, "lut", lut, sizeof(lut));
  if (err == ESP_OK) {
    nvs_commit(nvs);
    ESP_LOGI(TAG, "LUT (%s) saved to NVS.",
             state_->direction_inverted ? "INVERTED" : "NORMAL");
  } else {
    ESP_LOGE(TAG, "Failed to save LUT: %s", esp_err_to_name(err));
  }

  nvs_close(nvs);
}

// ---------------- Load LUT from NVS ----------------
void LUTCorrection::loadLUT() {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open(nvs_namespace_.c_str(), NVS_READONLY, &nvs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "No NVS data found (%s)", esp_err_to_name(err));
    return;
  }

  float lut[PULSES_PER_REV][2];
  size_t lut_size = sizeof(lut);

  err = nvs_get_blob(nvs, "lut", lut, &lut_size);
  if (err == ESP_OK) {
    for (int i = 0; i < PULSES_PER_REV; i++) {
      state_->sector_correction_factor[i][0] = lut[i][0];
      state_->sector_correction_factor[i][1] = lut[i][1];
    }
    ESP_LOGI(TAG, "LUT (NORMAL + INVERTED) loaded from NVS.");
  } else {
    ESP_LOGW(TAG, "No LUT found in NVS (%s)", esp_err_to_name(err));
  }

  nvs_close(nvs);
}

void LUTCorrection::printLUT() {
  nvs_handle_t nvs;
  esp_err_t err = nvs_open(nvs_namespace_.c_str(), NVS_READONLY, &nvs);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "No NVS data found (%s)", esp_err_to_name(err));
    return;
  }

  float lut[PULSES_PER_REV][2];
  size_t lut_size = sizeof(lut);

  err = nvs_get_blob(nvs, "lut", lut, &lut_size);
  if (err == ESP_OK) {
    printf("\n===== LUT almacenada en NVS =====\n");
    for (int i = 0; i < PULSES_PER_REV; i++) {
      printf("Sector %02d | + %.3f  | - %.3f\n", i, lut[i][0], lut[i][1]);
    }
    printf("=================================\n\n");
  } else {
    ESP_LOGW(TAG, "No LUT found in NVS (%s)", esp_err_to_name(err));
  }

  nvs_close(nvs);
}
