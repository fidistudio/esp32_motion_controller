#include "CommandProcessor.h"

#include <cstdio>
#include <cstring>

CommandProcessor::CommandProcessor(SystemActions &actions) : actions(actions) {}

void CommandProcessor::process(const char *cmd) {

  if (startsWith(cmd, "motor1")) {
    float duty;
    if (sscanf(cmd + 7, "%f", &duty) == 1) {
      actions.setMotorDuty(duty);
    }
    return;
  }

  if (strcmp(cmd, "step") == 0) {
    actions.startStepTest();
    return;
  }

  if (strcmp(cmd, "ramp") == 0) {
    actions.startRampTest();
    return;
  }

  if (startsWith(cmd, "lut")) {
    actions.enableLUT(strcmp(cmd + 4, "true") == 0);
    return;
  }

  if (startsWith(cmd, "invert")) {
    actions.invertEncoder(strcmp(cmd + 7, "true") == 0);
    return;
  }
}

bool CommandProcessor::startsWith(const char *str, const char *prefix) const {
  return std::strncmp(str, prefix, std::strlen(prefix)) == 0;
}
