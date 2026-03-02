#include "PIDController.h"

#include "esp_log.h"
#include <algorithm> // std::clamp

namespace {
constexpr float kMinSampleTime = 1e-9f;
}

PIDController::PIDController(const PIDGains &gains, const PIDTiming &timing,
                             float maxStep)
    : gains_(gains), timing_(timing), maxStep_(maxStep) {

  if (timing_.sampleTime <= 0.0f) {
    timing_.sampleTime = kMinSampleTime;
  }

  reset();
  computeContinuousModel();
  discretizeModel();
}

void PIDController::reset() {
  prevError1_ = 0.0f;
  prevError2_ = 0.0f;
  prevOutput1_ = 0.0f;
  prevOutput2_ = 0.0f;
}

float PIDController::update(float error) {
  // PID recursivo
  float output = (b0_ * error) + (b1_ * prevError1_) + (b2_ * prevError2_) -
                 (a1_ * prevOutput1_) - (a2_ * prevOutput2_);

  // Aplicar delta limit
  float delta = output - prevOutput1_;
  delta = std::clamp(delta, -maxStep_, maxStep_);
  output = prevOutput1_ + delta;

  // Guardar estados
  prevError2_ = prevError1_;
  prevError1_ = error;
  prevOutput2_ = prevOutput1_;
  prevOutput1_ = output;

  return output;
}

void PIDController::computeContinuousModel() {
  contA_ = gains_.kp * timing_.filterTime + gains_.kd;
  contB_ = gains_.kp + gains_.ki * timing_.filterTime;
  contC_ = gains_.ki;
}

void PIDController::discretizeModel() {
  const float alpha = 2.0f / timing_.sampleTime;
  const float alpha2 = alpha * alpha;

  const float denominator = (timing_.filterTime * alpha2) + alpha;

  b0_ = (contA_ * alpha2 + contB_ * alpha + contC_) / denominator;
  b1_ = (-2.0f * contA_ * alpha2 + 2.0f * contC_) / denominator;
  b2_ = (contA_ * alpha2 - contB_ * alpha + contC_) / denominator;

  a1_ = (-2.0f * timing_.filterTime * alpha) /
        (timing_.filterTime * alpha + 1.0f);

  a2_ =
      (timing_.filterTime * alpha - 1.0f) / (timing_.filterTime * alpha + 1.0f);
  ESP_LOGI("PID", "alpha=%f denominator=%f", alpha, denominator);
}
