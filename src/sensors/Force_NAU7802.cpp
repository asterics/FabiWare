#include "Force_NAU7802.h"
#include <Arduino.h>

// Access global loadcell filters defined in sensors.cpp
extern LoadcellSensor XS;
extern LoadcellSensor YS;

Adafruit_NAU7802 nau;

ForceNAU7802::ForceNAU7802(int drdyPin) : drdy_(drdyPin) {}

bool ForceNAU7802::init() {
  if (nau.begin(&Wire1)) {
    #ifdef DEBUG_OUTPUT_SENSORS
      DEBUG_OUT.println("SEN: Found NAU7802");
    #endif
  } else {
    #ifdef DEBUG_OUTPUT_SENSORS
      DEBUG_OUT.println("SEN: cannot find NAU7802 sensorboard");
    #endif
    return false;
  }

  pinMode(drdy_, INPUT);

  // apply configuration 
  nau.setLDO(NAU7802_3V0);
  nau.setGain(NAU7802_GAIN_128);
  nau.setRate(NAU7802_RATE_320SPS);
  nau.setPGACap(NAU7802_CAP_OFF);

  while (!nau.calibrate(NAU7802_CALMOD_INTERNAL)) {
    Serial.println("SEN: Failed to set NAU internal calibration, retrying!");
    delay(1000);
  }

  for (uint8_t i = 0; i < 10; i++) {
    while (!nau.available()) delay(1);
    nau.read();
  }

  nau.setChannel(NAU7802_CHANNEL1);
  channel_ = 0;

  // set default signal processing parameters for X and Y axis (see sensors.cpp)
  XS.setThresholdDecay(0.95);         YS.setThresholdDecay(0.95);
  XS.setBaselineLowpass(0.25);        YS.setBaselineLowpass(0.25);
  XS.setNoiseLowpass(3);              YS.setNoiseLowpass(3);
  XS.setActivityLowpass(2);           YS.setActivityLowpass(2);
  XS.setIdleDetectionPeriod(1000);    YS.setIdleDetectionPeriod(1000);
  XS.setIdleDetectionThreshold(3000); YS.setIdleDetectionThreshold(3000);
  XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE);
  YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE);
  return true;
}

ForceSample ForceNAU7802::readForce() {
  ForceSample s{0,0,0};
  if (digitalRead(drdy_) == HIGH && nau.available()) {
    int32_t val = nau.read();
    if (channel_ == 0) {
      // reading channel 1 (X)
      YS.lockBaseline(XS.isMoving());
      if (val) s.xRaw = val / NAU_DIVIDER;
      nau.setChannel(NAU7802_CHANNEL2);
      channel_ = 1;
    } else {
      // reading channel 2 (Y)
      XS.lockBaseline(YS.isMoving());
      if (val) s.yRaw = val / NAU_DIVIDER;
      nau.setChannel(NAU7802_CHANNEL1);
      channel_ = 0;
    }
    s.hasData = 1;
  }
  return s;
}

void ForceNAU7802::calibrate() {
  XS.calib();
  YS.calib();
}


// Moved from sensors.cpp: configure signal processing profiles for XS/YS
void setSensorBoard(int sensorBoardID)
{
  switch (sensorBoardID) {
    case SENSORBOARD_SG_HIGH:
      XS.setGain(0.5);                    YS.setGain(0.5);
      XS.setMovementThreshold(2000);      YS.setMovementThreshold(2000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(500);  YS.setIdleDetectionThreshold(500);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      break;
    case SENSORBOARD_SG_MEDIUM:
      XS.setGain(0.25);                   YS.setGain(0.25);
      XS.setMovementThreshold(1500);      YS.setMovementThreshold(1500);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(250);  YS.setIdleDetectionThreshold(250);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      break;
    case SENSORBOARD_SG_LOW:
      XS.setGain(0.1);                    YS.setGain(0.1);
      XS.setMovementThreshold(2000);      YS.setMovementThreshold(2000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(120);  YS.setIdleDetectionThreshold(120);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      break;
    case SENSORBOARD_SG_VERY_LOW:
      XS.setGain(0.05);                   YS.setGain(0.05);
      XS.setMovementThreshold(2500);      YS.setMovementThreshold(2500);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(60);   YS.setIdleDetectionThreshold(60);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      break;
    case SENSORBOARD_SMD_HIGH:
      XS.setGain(-1.0);                   YS.setGain(-1.5);
      XS.setMovementThreshold(2000);      YS.setMovementThreshold(3000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(500);  YS.setIdleDetectionThreshold(750);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
      break;
    case SENSORBOARD_SMD_MEDIUM:
      XS.setGain(-1.0);                   YS.setGain(-1.5);
      XS.setMovementThreshold(4000);      YS.setMovementThreshold(6000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(500);  YS.setIdleDetectionThreshold(750);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
      break;
    case SENSORBOARD_SMD_LOW:
      XS.setGain(-0.5);                   YS.setGain(-0.75);
      XS.setMovementThreshold(3000);      YS.setMovementThreshold(4000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(350);  YS.setIdleDetectionThreshold(500);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
      break;
    case SENSORBOARD_SMD_VERY_LOW:
      XS.setGain(-0.5);                   YS.setGain(-0.75);
      XS.setMovementThreshold(4000);      YS.setMovementThreshold(6000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(350);  YS.setIdleDetectionThreshold(500);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
      break;
  }
}
