#pragma once

class SystemActions {
public:
  virtual ~SystemActions() = default;

  virtual void setMotorDuty(float duty) = 0;
  virtual void startStepTest() = 0;
  virtual void startRampTest() = 0;
  virtual void enableLUT(bool enable) = 0;
  virtual void invertEncoder(bool inverted) = 0;
};
