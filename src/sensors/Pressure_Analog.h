#pragma once
#include "ISensor.h"
bool detectAnalogPressure();

class PressureAnalog : public IPressureSensor {
public:
  explicit PressureAnalog(int analogPin) : pin(analogPin) {}
  bool init() override;
  PressureSample readPressure() override;
private:
  int pin;
};
