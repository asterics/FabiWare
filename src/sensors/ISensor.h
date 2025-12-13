// Minimal sensor interfaces for modularization
#pragma once

#include <stdint.h>

struct PressureSample {
  int32_t raw;      // raw sensor value
  uint8_t hasData;  // 1 if new data available
};

struct ForceSample {
  int32_t xRaw;
  int32_t yRaw;
  uint8_t hasData;  // 1 if new data available
};

struct ButtonSample {
  uint16_t buttons;
  uint8_t hasData;  // 1 if new data available
};

class IPressureSensor {
public:
  virtual ~IPressureSensor() {}
  virtual bool init() = 0;
  virtual PressureSample readPressure() = 0;
  virtual void calibrate() {} // optional: sensor-specific calibration
};

class IForceSensor {
public:
  virtual ~IForceSensor() {}
  virtual bool init() = 0;
  virtual ForceSample readForce() = 0;
  virtual void calibrate() {} // optional: sensor-specific calibration
};

class IButtonSensor {
public:
  virtual ~IButtonSensor() {}
  virtual bool init() = 0;
  virtual ButtonSample readButtons() = 0;
};
