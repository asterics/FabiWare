#pragma once
#include "ISensor.h"
#include <Wire.h>


// data structure for FABI I2C transmission

#define FABI_I2C_CMD_GET_CAPS     0xFF    // command to request device capabilities bitmask

#define FABI_I2C_CAP_XY        (1<<0)  // bitmask for 1st data field in FABI generic sensor I2C report: int16_t x / int16_t y  (4 bytes)
#define FABI_I2C_CAP_PRESSURE  (1<<1)  // bitmask for 3rd data field in FABI generic sensor I2C report: int16_t pressure       (2 bytes)
#define FABI_I2C_CAP_BUTTONS   (1<<2)  // bitmask for 4th data field in FABI generic sensor I2C report: uint16_t buttons       (2 bytes)
  

typedef struct {
  uint8_t reportType;      // first byte contains flags (bitmask) for transferred data fields
  int16_t x;
  int16_t y;
  uint16_t pressure;
  uint16_t button_states;
} i2c_fabi_data_t;



class FabiGenericI2C : public IForceSensor, public IPressureSensor, public IButtonSource {
public:
  bool init() override;
  // IForceSensor
  ForceSample readForce() override;
  // IPressureSensor
  PressureSample readPressure() override;
  // IButtonSource
  ButtonSample readButtons() override;
  uint8_t getCapabilities() { return caps; }
private:
  uint8_t caps = 0;
};
