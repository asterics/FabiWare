#include "Fabi_GenericI2C.h"
#include "../sensors.h"

bool FabiGenericI2C::init() {
  // request capabilities from device and log
  Wire1.beginTransmission(FABI_I2C_ADDON_ADDR);
  Wire1.write(FABI_I2C_CMD_GET_CAPS);
  uint8_t result = Wire1.endTransmission();
  if (result != 0) {
    #ifdef DEBUG_OUTPUT_SENSORS
      Serial.println("SEN: cannot find generic FABI I2C sensor.");
    #endif
    return false;
  }

  if (Wire1.requestFrom(FABI_I2C_ADDON_ADDR, 1) == 1) {
    caps = Wire1.read();
    #ifdef DEBUG_OUTPUT_SENSORS
      Serial.printf("SEN: Generic FABI I2C sensor found! Capabilities Mask: 0x%02X\n", caps);
      if (caps & FABI_I2C_CAP_XY) Serial.println(" - Supports X and Y Axis");
      if (caps & FABI_I2C_CAP_PRESSURE) Serial.println(" - Supports Pressure");
      if (caps & FABI_I2C_CAP_BUTTONS) Serial.println(" - Supports Buttons");
    #endif
    return true;
  }
  #ifdef DEBUG_OUTPUT_SENSORS
    Serial.println("Slave did not respond to capability request.");
  #endif
  return false;
}

ForceSample FabiGenericI2C::readForce() {
  ForceSample s{0,0,0};
  Wire1.beginTransmission(FABI_I2C_ADDON_ADDR);
  Wire1.write(FABI_I2C_CAP_XY);
  Wire1.endTransmission();
  const uint8_t expectedBytes = 5;
  int received = Wire1.requestFrom(FABI_I2C_ADDON_ADDR, expectedBytes);
  if (received == expectedBytes) {
    uint8_t reportType = Wire1.read();
    int16_t x = (Wire1.read() | (Wire1.read() << 8));
    int16_t y = (Wire1.read() | (Wire1.read() << 8));
    if (reportType & FABI_I2C_CAP_XY) {
      s.xRaw = x;
      s.yRaw = y;
      s.hasData = 1;
    }
  }
  return s;
}

PressureSample FabiGenericI2C::readPressure() {
  PressureSample s{0,0};
  Wire1.beginTransmission(FABI_I2C_ADDON_ADDR);
  Wire1.write(FABI_I2C_CAP_PRESSURE);
  Wire1.endTransmission();
  const uint8_t expectedBytes = 3;
  int received = Wire1.requestFrom(FABI_I2C_ADDON_ADDR, expectedBytes);
  if (received == expectedBytes) {
    uint8_t reportType = Wire1.read();
    uint16_t p = (Wire1.read() | (Wire1.read() << 8));
    if (reportType & FABI_I2C_CAP_PRESSURE) {
      s.raw = p;
      s.hasData = 1;
    }
  }
  return s;
}

ButtonSample FabiGenericI2C::readButtons() {
  ButtonSample s{0,0};
  Wire1.beginTransmission(FABI_I2C_ADDON_ADDR);
  Wire1.write(FABI_I2C_CAP_BUTTONS);
  Wire1.endTransmission();
  const uint8_t expectedBytes = 3;
  int received = Wire1.requestFrom(FABI_I2C_ADDON_ADDR, expectedBytes);
  if (received == expectedBytes) {
    uint8_t reportType = Wire1.read();
    uint16_t b = (Wire1.read() | (Wire1.read() << 8));
    if (reportType & FABI_I2C_CAP_BUTTONS) {
      s.buttons = b;
      s.hasData = 1;
    }
  }
  return s;
}

