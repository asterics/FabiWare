#include "Force_Analog.h"
#include "sensors.h"
#include "utils.h"
#include <Arduino.h>

bool ForceAnalog::init() {
  if ((!isAnalogPinFloating(xPin)) && (!isAnalogPinFloating(yPin))) {
    #ifdef DEBUG_OUTPUT_SENSORS
      Serial.println("SEN: Force sensor connected to internal ADC");
    #endif
    pinMode(xPin, INPUT);
    pinMode(yPin, INPUT);
    return true;
  }
  #ifdef DEBUG_OUTPUT_SENSORS
    Serial.println("SEN: no force sensor connected");
  #endif
  return false;
}

ForceSample ForceAnalog::readForce() {
  ForceSample s{0,0,0};
  s.xRaw = analogRead(xPin) - calibX_;
  s.yRaw = analogRead(yPin) - calibY_;
  s.hasData = 1;
  return s;
}

void ForceAnalog::calibrate() {
  calibX_ = analogRead(xPin);
  calibY_ = analogRead(yPin);
}

