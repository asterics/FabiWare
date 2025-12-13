/*
  FabiWare - AsTeRICS Foundation
  For more info please visit: https://www.asterics-foundation.org

  Module: sensors.cpp - functions to read pressure & force sensors

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; See the GNU General Public License:
  http://www.gnu.org/licenses/gpl-3.0.en.html

*/

#include "sensors.h"
#include "gpio.h"
#include "modes.h"

// Include sensor interfaces and implementations
#include "sensors/ISensor.h"
#include "sensors/Pressure_MPRLS.h"
#include "sensors/Pressure_DPS310.h"
#include "sensors/Pressure_Analog.h"
#include "sensors/Force_NAU7802.h"
#include "sensors/Force_Analog.h"
#include "sensors/Fabi_GenericI2C.h"
#include "sensors/utils.h"

LoadcellSensor XS, YS;
LoadcellSensor PS;
int sensorWatchdog = -1;
uint8_t reportXValues = 0;
uint8_t reportYValues = 0;

// sensor modules
static ForceNAU7802  nau(DRDY_PIN);
static PressureDPS310 dps;
static PressureMPRLS mprls;
static FabiGenericI2C fabi;
static PressureAnalog panalog(ANALOG_PRESSURE_SENSOR_PIN);
static ForceAnalog fanalog(ANALOG_FORCE_SENSOR_X_PIN, ANALOG_FORCE_SENSOR_Y_PIN);

// available generic sensor interfaces
static IPressureSensor* gPressure = nullptr;
static IForceSensor* gForce = nullptr;
static IButtonSource* gButtons = nullptr;
static FabiGenericI2C* gFabiAddon = nullptr;

// Global structure for passing sensor data to other core
struct CurrentSensorDataCore1 currentSensorDataCore1 {        
  .xRaw=0, .yRaw=0, .pressure=512, 
  .calib_now=CALIBRATION_PERIOD,     // calibrate sensors after startup !
  .calibX=512, .calibY=512,
  .I2CButtonStates=0
};

/**
   @name initSensors
   @brief initialises availabel sensors and prepares readouts. [called from core 1]
   @return none
*/
void initSensors()
{
  // Detect and initialize available pressure sensors
  currentSensorDataCore1.pressureSensorType = PRESSURE_NONE;
  if (dps.init()) {
    currentSensorDataCore1.pressureSensorType = PRESSURE_DPS310;
    gPressure = &dps;
  } else if (mprls.init()) {
    currentSensorDataCore1.pressureSensorType = PRESSURE_MPRLS;
    gPressure = &mprls;
  } else if (panalog.init()) {
      currentSensorDataCore1.pressureSensorType = PRESSURE_INTERNAL_ADC;
      gPressure = &panalog;
  }

  if (currentSensorDataCore1.pressureSensorType == PRESSURE_NONE) {
  #ifdef DEBUG_OUTPUT_SENSORS
    DEBUG_OUT.println("SEN: no pressure sensor connected");
  #endif     
  } 

  // Detect and initialize available force sensors
  currentSensorDataCore1.forceSensorType = FORCE_NONE;
  if (nau.init()) {
    currentSensorDataCore1.forceSensorType = FORCE_NAU7802;
    gForce = &nau;
  } else if (fabi.init()) {

    gFabiAddon = &fabi;

    // get capabilities and update sensor types accordingly
    uint8_t FABIAddOnCapabilities = fabi.getCapabilities();

    if (FABIAddOnCapabilities & FABI_I2C_CAP_XY) {
      gForce = &fabi; 
      currentSensorDataCore1.forceSensorType = FORCE_FABI_GENERIC;
    }
    if (FABIAddOnCapabilities & FABI_I2C_CAP_PRESSURE) {
      gPressure = &fabi; 
      currentSensorDataCore1.pressureSensorType = PRESSURE_FABI_GENERIC;
    }
    if (FABIAddOnCapabilities & FABI_I2C_CAP_BUTTONS) {
      gButtons = &fabi; 
    }
  } else if (fanalog.init()) {
      currentSensorDataCore1.forceSensorType = FORCE_INTERNAL_ADC;
      #ifndef FLIPMOUSE
        // in case the FABI3 PCB is used, we cannot use internal ADC0 for force and pressure ...
        if (currentSensorDataCore1.pressureSensorType == PRESSURE_INTERNAL_ADC) { 
          currentSensorDataCore1.pressureSensorType = PRESSURE_NONE;
        }
      #endif
      gForce = &fanalog;
  }
}


/**
   @name readPressure
   @brief updates and processes new pressure sensor values
   @return none
*/
void readPressure()
{
  int actPressure = 512;
  int newData = 0;

  NB_DELAY_START(pressure_ts, 1000 / PRESSURE_MAX_SAMPLINGRATE)
    if (gPressure) {
      PressureSample s = gPressure->readPressure();
      if (s.hasData) {
        actPressure = s.raw;
        newData = 1;
      }
    }

    // limit pressure values to valid range (0 and 1023 are reserved for bypass)
    if (actPressure < 1) actPressure = 1;
    if (actPressure > 1022) actPressure = 1022;

    if (currentSensorDataCore1.calib_now) actPressure = 512;
    if (newData) {
      mutex_enter_blocking(&(currentSensorDataCore1.sensorDataMutex));
      currentSensorDataCore1.pressure = actPressure;
      mutex_exit(&(currentSensorDataCore1.sensorDataMutex));
    }
  NB_DELAY_END
}

/**
   @name readForce
   @brief updates and processes new force sensor values
   @return none
*/
void readForce()
{
  static uint8_t printCount = 0;
  uint8_t newData=0;
  int32_t currentX = 0, currentY = 0;

  NB_DELAY_START(force_ts, 1000 / FORCE_MAX_SAMPLINGRATE)
    if (gForce) {
      ForceSample s = gForce->readForce();
      if (s.hasData) {
        currentX = s.xRaw;
        currentY = s.yRaw;
        if (currentSensorDataCore1.calib_now) {
          currentX = 0;
          currentY = 0;
        }
        newData = 1;
      }
    }

    // read buttons from generic I2C addon if available
    if (gButtons) {
      ButtonSample bs = gButtons->readButtons();
      if (bs.hasData) {
        mutex_enter_blocking(&(currentSensorDataCore1.sensorDataMutex));
        currentSensorDataCore1.I2CButtonStates = bs.buttons;
        mutex_exit(&(currentSensorDataCore1.sensorDataMutex));
      }
    }

    if (newData) {
      mutex_enter_blocking(&(currentSensorDataCore1.sensorDataMutex));
      currentSensorDataCore1.xRaw =  currentX;
      currentSensorDataCore1.yRaw =  currentY;
      mutex_exit(&(currentSensorDataCore1.sensorDataMutex));
    }
  NB_DELAY_END
}


/**
   @name calculateDirection
   @brief calculates angular direction and force for current x/y sensor values. [called from core 0]
   @param sensorData: pointer to SensorData struct, used by core0
   @return none
*/
void calculateDirection(struct SensorData * sensorData)
{
  sensorData->forceRaw = sqrtf(sensorData->xRaw * sensorData->xRaw + sensorData->yRaw * sensorData->yRaw);
  if (sensorData->forceRaw != 0) {
    sensorData->angle = atan2f ((float)sensorData->yRaw / sensorData->forceRaw, (float)sensorData->xRaw / sensorData->forceRaw );

    // get 8 directions
    sensorData->dir = (180 + 22 + (int)(sensorData->angle * 57.29578)) / 45 + 1; // translate rad to deg and make 8 sections
    if (sensorData->dir > 8) sensorData->dir = 1;
  }
}

/**
   @name applyDeadzone
   @brief calculates deadzone and respective x/y/force values (in sensorData struct). [called from core 0]
   @param sensorData: pointer to SensorData struct, used by core0
   @param slotSettings: pointer to SlotSettings struct, used by core0
   @return none
*/
void applyDeadzone(struct SensorData * sensorData, struct SlotSettings * slotSettings)
{
  if (slotSettings->stickMode == STICKMODE_ALTERNATIVE) {

    // rectangular deadzone for alternative modes
    if (sensorData->xRaw < -slotSettings->dx)
      sensorData->x = sensorData->xRaw + slotSettings->dx; // apply deadzone values x direction
    else if (sensorData->xRaw > slotSettings->dx)
      sensorData->x = sensorData->xRaw - slotSettings->dx;
    else sensorData->x = 0;

    if (sensorData->yRaw < -slotSettings->dy)
      sensorData->y = sensorData->yRaw + slotSettings->dy; // apply deadzone values y direction
    else if (sensorData->yRaw > slotSettings->dy)
      sensorData->y = sensorData->yRaw - slotSettings->dy;
    else sensorData->y = 0;

  } else {

    //  circular deadzone for mouse control
    if (sensorData->forceRaw != 0) {
      float a = slotSettings->dx > 0 ? slotSettings->dx : 1 ;
      float b = slotSettings->dy > 0 ? slotSettings->dy : 1 ;
      float s = sinf(sensorData->angle);
      float c = cosf(sensorData->angle);
      sensorData->deadZone =  a * b / sqrtf(a * a * s * s + b * b * c * c); // ellipse equation, polar form
    }
    else sensorData->deadZone = slotSettings->dx;

    sensorData->force = (sensorData->forceRaw < sensorData->deadZone) ? 0 : sensorData->forceRaw - sensorData->deadZone;
    sensorData->x = (int) (sensorData->force * cosf(sensorData->angle));
    sensorData->y = (int) (sensorData->force * sinf(sensorData->angle));
  }
}

/**
   @name checkSensorWatchdog
   @brief checks if an integer value which should be periodically reset when I2C-sensordata is ready exceeds a certain value
   @return true: value within normal range  false: value exceeded -> action must be taken
*/
uint8_t checkSensorWatchdog() {
  // if we never received any valid I2C sensor values, proceed
  if (sensorWatchdog == -1) return (true);
  // if we received valid sensor values at least once, 
  // check if I2C sensors are still active, reset after ~1s (SENSOR_WATCHDOG_TIMEOUT)
  if (sensorWatchdog++ > SENSOR_WATCHDOG_TIMEOUT)
    return (false);
  return (true);
}


/**
   @name startSensorCalibration
   @brief prepares a sensor calibration 
   @return none
*/
void startSensorCalibration() {
   currentSensorDataCore1.calib_now = CALIBRATION_PERIOD;  
}

/**
   @name checkSensorCalibration
   @brief calibrates the offset values for the sensors (pressure & force) if necessary
   @return none
*/
void checkSensorCalibration()
{
  // if calibration running: update calibration counter
  if (!currentSensorDataCore1.calib_now) return;
  currentSensorDataCore1.calib_now--;  

  // calibrate sensors in the middle of the calibration period
  if(currentSensorDataCore1.calib_now != CALIBRATION_PERIOD/2) return;
  if (gForce) gForce->calibrate();
  if (gPressure) gPressure->calibrate();
}

/**
   @name getForceSensorType
   @brief returns the type of the available force sensor
   @return forceSensor_type_t
*/
forceSensor_type_t getForceSensorType() {
  return (currentSensorDataCore1.forceSensorType);
}

/**
   @name getPressureSensorType
   @brief returns the type of the available pressure sensor
   @return pressureSensor_type_t
*/
pressureSensor_type_t getPressureSensorType(){
  return (currentSensorDataCore1.pressureSensorType);
}

