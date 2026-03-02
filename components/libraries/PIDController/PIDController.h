#pragma once

#include <algorithm> // para std::clamp
#include <cstdint>

struct PIDGains {
  float kp;
  float ki;
  float kd;
};

struct PIDTiming {
  float sampleTime; // Ts [s]
  float filterTime; // Tf [s] (0 = no filter)
};

class PIDController {
public:
  PIDController(const PIDGains &gains, const PIDTiming &timing,
                float maxStep = 1.0f);

  float update(float error);
  void reset();

  void setMaxStep(float maxStep) { maxStep_ = maxStep; }

private:
  void computeContinuousModel();
  void discretizeModel();

  PIDGains gains_;
  PIDTiming timing_;

  float contA_;
  float contB_;
  float contC_;

  float b0_;
  float b1_;
  float b2_;
  float a1_;
  float a2_;

  float prevError1_;
  float prevError2_;
  float prevOutput1_;
  float prevOutput2_;

  float maxStep_;
};
