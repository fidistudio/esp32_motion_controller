
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cstdint>

struct PidConfig {
  double kp;
  double ki;
  double kd;
  double ts; // Sample time in seconds
  double tf; // Derivative filter time constant (Tf = 0 means no filter)
};

class PidController {
public:
  explicit PidController(const PidConfig &config);

  double compute(double error);
  void reset();

private:
  void computeDiscreteCoefficients(const PidConfig &cfg);

  // Discrete-time coefficients (Tustin)
  double b0_;
  double b1_;
  double b2_;
  double a1_;
  double a2_;

  // Previous values
  double e1_;
  double e2_;
  double u1_;
  double u2_;
};

#endif // PID_CONTROLLER_H
