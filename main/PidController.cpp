
#include "PidController.h"

PidController::PidController(const PidConfig &cfg)
    : b0_(0.0), b1_(0.0), b2_(0.0), a1_(0.0), a2_(0.0), e1_(0.0), e2_(0.0),
      u1_(0.0), u2_(0.0) {
  computeDiscreteCoefficients(cfg);
}

void PidController::reset() {
  e1_ = e2_ = 0.0;
  u1_ = u2_ = 0.0;
}

void PidController::computeDiscreteCoefficients(const PidConfig &c) {
  const double Ts = c.ts;
  const double Kp = c.kp;
  const double Ki = c.ki;
  const double Kd = c.kd;
  const double Tf = (c.tf <= 0.0) ? 1e-9 : c.tf; // avoid division by zero

  // Continuous PID with derivative filter:
  // C(s) = Kp + Ki/s + Kd*s / (Tf*s + 1)

  // Tustin substitution:
  // s = (2/Ts) * (1 - z^-1) / (1 + z^-1)

  const double alpha = 2.0 / Ts;

  // Compute discrete coefficients following Tustin for full PID + filter.
  // These formulas come directly from symbolic discretization.

  const double k1 = Kp + Ki / alpha + Kd * alpha;
  const double k2 = -Kp + Ki / alpha - 2.0 * Kd * alpha;
  const double k3 = -Ki / alpha + Kd * alpha;

  const double d1 = (Tf * alpha - 1.0);
  const double d2 = (Tf * alpha + 1.0);

  // Final discrete coefficients:
  // u[k] = b0*e[k] + b1*e[k-1] + b2*e[k-2] - a1*u[k-1] - a2*u[k-2]

  b0_ = k1 / d2;
  b1_ = k2 / d2;
  b2_ = k3 / d2;
  a1_ = d1 / d2;
  a2_ = 0.0; // First-order derivative filter → only a1, no a2
}

double PidController::compute(double error) {
  // Difference equation
  const double u = b0_ * error + b1_ * e1_ + b2_ * e2_ - a1_ * u1_ - a2_ * u2_;

  // Shift memory
  e2_ = e1_;
  e1_ = error;

  u2_ = u1_;
  u1_ = u;

  return u;
}
