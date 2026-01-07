#pragma once

#include "esp_err.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

extern "C" {
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
}

// ---------------- Constants ----------------

constexpr int PULSES_PER_REV = 15;
constexpr int MAX_CALIBRATION_REVS = 15;
constexpr int GLITCH_FILTER_NS = 10000; // 10 µs

constexpr int PCNT_SINGLE_PULSE_HIGH = 2;
constexpr int PCNT_SINGLE_PULSE_LOW = -1;

// ---------------- Runtime State ----------------

struct EncoderRuntimeState {
  volatile int64_t lastPulseTimeUs;
  volatile int64_t pulseIntervalUs;
  volatile int currentSector;

  bool isDirectionInverted;
  bool useCorrectionLUT;

  bool calibrationRequested;
  bool isCalibrating;
  bool velocityReset;
  int completedRevolutions;

  int64_t sectorIntervals[PULSES_PER_REV][MAX_CALIBRATION_REVS];
  float sectorCorrection[PULSES_PER_REV][2];
};

// ---------------- Encoder Class ----------------

class Encoder {
public:
  Encoder(gpio_num_t encoder_gpio, gpio_num_t sector_gpio);

  void begin();

  bool isCalibrating() const { return state_.isCalibrating; }
  void requestCalibration() { state_.calibrationRequested = true; }

  void invertDirection();
  void setDirectionNormal();

  void resetVelocity();
  void enableVelocityTracking();

  float computeRadPerSec();
  float computeRPM();

  EncoderRuntimeState &state();
  const EncoderRuntimeState &state() const;

private:
  gpio_num_t encoder_gpio_;
  gpio_num_t sector_gpio_;

  pcnt_unit_handle_t encoder_unit_;
  pcnt_unit_handle_t sector_unit_;

  EncoderRuntimeState state_{};

  // ---- PCNT setup ----
  void configurePCNTUnit(pcnt_unit_handle_t *unit);
  void configurePCNTChannel(pcnt_unit_handle_t unit, gpio_num_t gpio);
  void configureGlitchFilter(pcnt_unit_handle_t unit);
  void startPCNT(pcnt_unit_handle_t unit);

  void setupEncoderPCNT();
  void setupSectorPCNT();

  // ---- ISR callbacks ----
  static bool IRAM_ATTR onEncoderPulse(pcnt_unit_handle_t unit,
                                       const pcnt_watch_event_data_t *edata,
                                       void *user_ctx);

  static bool IRAM_ATTR onSectorReset(pcnt_unit_handle_t unit,
                                      const pcnt_watch_event_data_t *edata,
                                      void *user_ctx);

  // ---- ISR helpers ----
  static inline void updatePulseTiming(EncoderRuntimeState *state, int64_t now);

  static inline void advanceSector(EncoderRuntimeState *state);

  // ---- Math helpers ----
  float computeAngularVelocity(float scaleFactor);
  float applySectorCorrection(int sector, int64_t intervalUs) const;
  int previousSector() const;

  void remapCurrentSector();

  static portMUX_TYPE encoder_mux;
};
