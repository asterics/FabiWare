#include "Pressure_MPRLS.h"
#include "../FabiWare.h"
#include "../sensors.h"
#include "utils.h"
#include <Wire.h>

static int medianBuf[MPRLS_MEDIAN_VALUES] = {0};
static MedianState medianState{medianBuf, MPRLS_MEDIAN_VALUES, 0};

// Uses Wire1 and addresses defined in FabiWare.h

bool PressureMPRLS::init() {
  Wire1.beginTransmission(MPRLS_ADDR);
  uint8_t result = Wire1.endTransmission();
  if (result == 0) {
    #ifdef DEBUG_OUTPUT_SENSORS
      DEBUG_OUT.println("SEN: found MPRLS");
    #endif
  } else {
    #ifdef DEBUG_OUTPUT_SENSORS
      DEBUG_OUT.println("SEN: cannot find MPRLS");
    #endif
    return false;
  }

  // configure pressure signal processing (see sensors.cpp)
  PS.setGain(1.0);
  PS.setSampleRate(PRESSURE_MAX_SAMPLINGRATE);
  PS.setBaselineLowpass(0.4);
  PS.setNoiseLowpass(10.0);
  PS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE);
  PS.setActivityLowpass(1);
  PS.setIdleDetectionPeriod(500);
  PS.setIdleDetectionThreshold(500);
  return true;
}

PressureSample PressureMPRLS::readPressure() {
  PressureSample s{0, 0};
  const uint8_t expected = 4;
  uint8_t buf[4] = {0};
  int received = Wire1.requestFrom(MPRLS_ADDR, expected);
  if (received == expected) {
    for (uint8_t i = 0; i < 4; i++) buf[i] = Wire1.read();
    int32_t raw = (uint32_t(buf[1]) << 16) | (uint32_t(buf[2]) << 8) | (uint32_t(buf[3]));
    // spike filter via median
    int med = calculateMedian(raw, &medianState);
    if (abs(med - raw) > SPIKE_DETECTION_THRESHOLD) raw = med;
    // signal conditioning
    int filtered = PS.process(raw);
    if (filtered > 0) filtered = sqrt(filtered);
    if (filtered < 0) filtered = -sqrt(-filtered);
    s.raw = 512 + filtered / MPRLS_DIVIDER;
    s.hasData = 1;
  }
  // trigger new conversion
  Wire1.beginTransmission(MPRLS_ADDR);
  Wire1.write(0xAA);
  Wire1.write(0);
  Wire1.write(0);
  Wire1.endTransmission();
  return s;
}

void PressureMPRLS::calibrate() {
  PS.calib();
}
