#pragma once

#include "SystemActions.h"

class CommandProcessor {
public:
  explicit CommandProcessor(SystemActions &actions);

  void process(const char *command);

private:
  SystemActions &actions;

  bool startsWith(const char *str, const char *prefix) const;
};
