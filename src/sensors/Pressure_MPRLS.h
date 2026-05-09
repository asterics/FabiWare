#pragma once
#include "ISensor.h"
#include "../sensors.h"
#include <Wire.h>

// MPRLS-specific configuration and filtering parameters
#define MPRLS_READ_TIMEOUT (20)
#define MPRLS_STATUS_POWERED (0x40)
#define MPRLS_STATUS_BUSY (0x20)
#define MPRLS_STATUS_FAILED (0x04)
#define MPRLS_STATUS_MATHSAT (0x01)
#define MPRLS_STATUS_MASK (0b01100101)
#define MPRLS_DIVIDER 2
#define MPRLS_MEDIAN_VALUES 5
#define SPIKE_DETECTION_THRESHOLD 1000

class PressureMPRLS : public IPressureSensor {
public:
  bool init() override;
  PressureSample readPressure() override;
  void calibrate() override;
};

