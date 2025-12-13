#pragma once

#include <Adafruit_NAU7802.h>  //NAU7802 library (Benjamin Aigner's fork with channel change feature)
#include "ISensor.h"
#include "../sensors.h"

// Forward declaration to avoid duplicate header inclusion issues
class Adafruit_NAU7802;

/**
   @brief Sensorboard IDs for different signal processing parameters
*/
#define SENSORBOARD_SG_HIGH      0
#define SENSORBOARD_SG_MEDIUM    1
#define SENSORBOARD_SG_LOW       2
#define SENSORBOARD_SG_VERY_LOW  3
#define SENSORBOARD_SMD_HIGH     4
#define SENSORBOARD_SMD_MEDIUM   5
#define SENSORBOARD_SMD_LOW      6
#define SENSORBOARD_SMD_VERY_LOW 7

// NAU7802-specific parameters
#define NAU_DIVIDER 120

class ForceNAU7802 : public IForceSensor {
public:
  ForceNAU7802(int drdyPin);
  bool init() override;
  ForceSample readForce() override;
  void calibrate() override;
private:
  int drdy_;
  uint8_t channel_ = 0;
};
