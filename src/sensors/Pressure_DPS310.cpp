#include "Pressure_DPS310.h"
#include "../FabiWare.h"
#include "../sensors.h"
#include "utils.h"
#include <Wire.h>

static int medianBuf[DPS_MEDIAN_VALUES] = {0};
static MedianState medianState{medianBuf, DPS_MEDIAN_VALUES, 0};

static inline int32_t twosComplement24(int32_t val) {
  if (val & (1 << 23)) {
    val -= (1 << 24);
  }
  return val;
}

bool PressureDPS310::init() {
  Wire1.beginTransmission(DPS310_ADDR);
  uint8_t result = Wire1.endTransmission();
  if (result == 0) {
    #ifdef DEBUG_OUTPUT_SENSORS
      DEBUG_OUT.println("SEN: found DPS310");
    #endif
  } else {
    #ifdef DEBUG_OUTPUT_SENSORS
      DEBUG_OUT.println("SEN: cannot find DPS310");
    #endif
    return false;
  }

  Wire1.beginTransmission(DPS310_ADDR);
  Wire1.write(0x06); // DPS_R_PRS_CFG
  Wire1.write((0b111 << 4) | (0b0000));
  Wire1.endTransmission();

  Wire1.beginTransmission(DPS310_ADDR);
  Wire1.write(0x08); // DPS_R_MEAS_CFG
  Wire1.write(0b101);
  Wire1.endTransmission();

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

PressureSample PressureDPS310::readPressure() {
  PressureSample s{0, 0};
  Wire1.beginTransmission(DPS310_ADDR);
  Wire1.write(0x00); // DPS_R_PSR_B2
  Wire1.endTransmission();

  Wire1.requestFrom(DPS310_ADDR, 3);
  uint8_t b[3] = {0};
  for (uint8_t i = 0; i < 3; i++) {
    b[i] = Wire1.read();
  }
  int32_t r_p = (uint32_t(b[0]) << 16) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]));
  int32_t raw = twosComplement24(r_p);
  raw *= DPS_SCALEFACTOR;
  int med = calculateMedian(raw, &medianState);
  if (abs(med - raw) > DPS_SPIKE_DETECTION_THRESHOLD) raw = med;
  int filtered = PS.process(raw);
  if (filtered > 0) filtered = sqrt(filtered);
  if (filtered < 0) filtered = -sqrt(-filtered);
  s.raw = 512 + filtered / DPS_DIVIDER;
  s.hasData = 1;
  return s;
}

void PressureDPS310::calibrate() {
  PS.calib();
}
