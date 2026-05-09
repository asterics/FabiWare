#pragma once
#include "ISensor.h"

class ForceAnalog : public IForceSensor {
public:
  ForceAnalog(int pinX, int pinY) : xPin(pinX), yPin(pinY) {}
  bool init() override;
  ForceSample readForce() override;
  void calibrate() override;
private:
  int xPin;
  int yPin;
  int calibX_ = 512;
  int calibY_ = 512;
};
