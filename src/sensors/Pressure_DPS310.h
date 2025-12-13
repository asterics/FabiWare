#pragma once
#include "ISensor.h"
#include "../sensors.h"
#include <Wire.h>

// DPS310-specific configuration and filtering parameters
#define DPS_R_PSR_B2 0x00
#define DPS_R_PSR_B1 0x01
#define DPS_R_PSR_B0 0x02
#define DPS_R_PRS_CFG 0x06
#define DPS_R_MEAS_CFG 0x08
#define DPS_R_CFG_REG 0x09
#define DPS_R_RESET 0x0C

#define DPS_SCALEFACTOR  -20
#define DPS_DIVIDER  3
#define DPS_SPIKE_DETECTION_THRESHOLD 150
#define DPS_MEDIAN_VALUES 5

class PressureDPS310 : public IPressureSensor {
public:
  bool init() override;
  PressureSample readPressure() override;
  void calibrate() override;
};

