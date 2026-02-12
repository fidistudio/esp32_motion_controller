#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string>
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

class LUTStore
{
public:
    LUTStore(float (*lut)[2], int8_t NUM_SECTORS,
             const std::string &nvs_namespace);

    void saveLUT(bool inverted);
    void loadLUT();

private:
    float (*lut_)[2];
    int8_t NUM_SECTORS_;
    std::string nvs_namespace_;
    const char *TAG = "LUTStore";
};
