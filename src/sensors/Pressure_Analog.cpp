#include "Pressure_Analog.h"
#include "sensors.h"
#include "utils.h"
#include <Arduino.h>

bool PressureAnalog::init() {
  if (!isAnalogPinFloating(pin)) {
    #ifdef DEBUG_OUTPUT_SENSORS
      DEBUG_OUT.println("SEN: Pressure sensor connected to internal ADC");
    #endif
    pinMode(pin, INPUT);
    return true;
  }
  return false;
}

PressureSample PressureAnalog::readPressure() {
  PressureSample s{0, 0};
  s.raw = analogRead(pin);
  s.hasData = 1;
  return s;
}

