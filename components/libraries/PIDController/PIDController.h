#pragma once

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
  PIDController(const PIDGains &gains, const PIDTiming &timing);

  float update(float error);
  void reset();

private:
  // --- Initialization steps ---
  void computeContinuousModel();
  void discretizeModel();

  // --- User configuration ---
  PIDGains gains_;
  PIDTiming timing_;

  // --- Continuous-time model coefficients ---
  float contA_;
  float contB_;
  float contC_;

  // --- Discrete-time (Tustin) coefficients ---
  float b0_;
  float b1_;
  float b2_;
  float a1_;
  float a2_;

  // --- Controller state ---
  float prevError1_;
  float prevError2_;
  float prevOutput1_;
  float prevOutput2_;
};
