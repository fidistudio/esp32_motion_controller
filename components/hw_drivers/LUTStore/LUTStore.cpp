#include "LUTStore.h"
#include <string.h>

LUTStore::LUTStore(float (*lut)[2], int8_t NUM_SECTORS,
                   const std::string &nvs_namespace)
    : lut_(lut), NUM_SECTORS_(NUM_SECTORS),
      nvs_namespace_(nvs_namespace)
{
}

// ---------------- Save LUT to NVS ----------------
void LUTStore::saveLUT(bool inverted)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(nvs_namespace_.c_str(), NVS_READWRITE, &nvs);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return;
    }

    float lut[NUM_SECTORS_][2];
    memset(lut, 0, sizeof(lut));

    // Load existing LUT if available (so we don’t overwrite the other direction)
    size_t lut_size = sizeof(lut);
    err = nvs_get_blob(nvs, "lut", lut, &lut_size);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(TAG, "No previous LUT found, creating new one...");
    }

    for (int i = 0; i < NUM_SECTORS_; i++)
    {
        lut[i][inverted] = lut_[i][inverted];
    }

    err = nvs_set_blob(nvs, "lut", lut, sizeof(lut));
    if (err == ESP_OK)
    {
        nvs_commit(nvs);
        ESP_LOGI(TAG, "LUT (%s) saved to NVS.",
                 inverted ? "INVERTED" : "NORMAL");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to save LUT: %s", esp_err_to_name(err));
    }

    nvs_close(nvs);
}

// ---------------- Load LUT from NVS ----------------
void LUTStore::loadLUT()
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(nvs_namespace_.c_str(), NVS_READONLY, &nvs);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "No NVS data found (%s)", esp_err_to_name(err));
        return;
    }

    float lut[NUM_SECTORS_][2];
    size_t lut_size = sizeof(lut);

    err = nvs_get_blob(nvs, "lut", lut, &lut_size);
    if (err == ESP_OK)
    {
        for (int i = 0; i < NUM_SECTORS_; i++)
        {
            lut_[i][0] = lut[i][0];
            lut_[i][1] = lut[i][1];
        }
        ESP_LOGI(TAG, "LUT (NORMAL + INVERTED) loaded from NVS.");
    }
    else
    {
        ESP_LOGW(TAG, "No LUT found in NVS (%s)", esp_err_to_name(err));
    }

    nvs_close(nvs);
}