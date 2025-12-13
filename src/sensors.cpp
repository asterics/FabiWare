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

// sensor modules
static PressureDPS310 pressureSensor_DPS;
static PressureMPRLS  pressureSensor_MPRLS;
static PressureAnalog pressureSensor_ADC(ANALOG_PRESSURE_SENSOR_PIN);
static ForceNAU7802   forceSensor_NAU(DRDY_PIN);
static ForceAnalog    forceSensor_ADC(ANALOG_FORCE_SENSOR_X_PIN, ANALOG_FORCE_SENSOR_Y_PIN);
static FabiGenericI2C genericSensorInterface;

// available generic sensor interfaces
static IPressureSensor * gPressure = nullptr;
static IForceSensor    * gForce = nullptr;
static IButtonSensor   * gButtons = nullptr;

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
  if (pressureSensor_DPS.init()) {
    currentSensorDataCore1.pressureSensorType = PRESSURE_DPS310;
    gPressure = &pressureSensor_DPS;
  } else if (pressureSensor_MPRLS.init()) {
    currentSensorDataCore1.pressureSensorType = PRESSURE_MPRLS;
    gPressure = &pressureSensor_MPRLS;
  } else if (pressureSensor_ADC.init()) {
    currentSensorDataCore1.pressureSensorType = PRESSURE_INTERNAL_ADC;
    gPressure = &pressureSensor_ADC;
  }

  if (currentSensorDataCore1.pressureSensorType == PRESSURE_NONE) {
  #ifdef DEBUG_OUTPUT_SENSORS
    DEBUG_OUT.println("SEN: no pressure sensor connected");
  #endif     
  } 

  // Detect and initialize available force sensors
  currentSensorDataCore1.forceSensorType = FORCE_NONE;
  if (forceSensor_NAU.init()) {
    currentSensorDataCore1.forceSensorType = FORCE_NAU7802;
    gForce = &forceSensor_NAU;
  } else if (genericSensorInterface.init()) {

    // get capabilities and update sensor types accordingly
    uint8_t addOnCapabilities = genericSensorInterface.getCapabilities();

    if (addOnCapabilities & FABI_I2C_CAP_XY) {
      gForce = &genericSensorInterface; 
      currentSensorDataCore1.forceSensorType = FORCE_FABI_GENERIC;
    }
    if (addOnCapabilities & FABI_I2C_CAP_PRESSURE) {
      gPressure = &genericSensorInterface; 
      currentSensorDataCore1.pressureSensorType = PRESSURE_FABI_GENERIC;
    }
    if (addOnCapabilities & FABI_I2C_CAP_BUTTONS) {
      gButtons = &genericSensorInterface; 
    }
  } else if (forceSensor_ADC.init()) {
      currentSensorDataCore1.forceSensorType = FORCE_INTERNAL_ADC;

      // handle special cases for ADC force sensor
      #ifndef FLIPMOUSE
        // in case the FABI3 PCB is used, we cannot use internal ADC0 for both (force and pressure) ...
        if (currentSensorDataCore1.pressureSensorType == PRESSURE_INTERNAL_ADC) { 
          currentSensorDataCore1.pressureSensorType = PRESSURE_NONE;
          gPressure = nullptr;
        }

        // if no I2C devices are detected on Wire1, we can use the SDA/SCL pins as GPIOs for the analog sensors!
        if(testI2Cdevices(&Wire1) == 0) {
          currentState.useI2CasGPIO = true;
          //Serial.println("I2C->GPIO");
          Wire1.end();
          pinMode(PIN_WIRE1_SDA_,INPUT_PULLUP);
          pinMode(PIN_WIRE1_SCL_,INPUT_PULLUP);
        }
      #endif
      gForce = &forceSensor_ADC;
  }

}


/**
   @name readPressure
   @brief updates and processes new pressure sensor values
   @return none
*/
void readPressure()
{
  NB_DELAY_START(pressure_ts, 1000 / PRESSURE_MAX_SAMPLINGRATE)
    if (gPressure) {
      int actPressure = 512;
      PressureSample s = gPressure->readPressure();
      if (s.hasData) {
        actPressure = s.raw;
        // limit pressure values to valid range (0 and 1023 are reserved for bypass)
        if (actPressure < 1) actPressure = 1;
        if (actPressure > 1022) actPressure = 1022;

        if (currentSensorDataCore1.calib_now) actPressure = 512;
        mutex_enter_blocking(&(currentSensorDataCore1.sensorDataMutex));
        currentSensorDataCore1.pressure = actPressure;
        mutex_exit(&(currentSensorDataCore1.sensorDataMutex));
      }
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
  NB_DELAY_START(force_ts, 1000 / FORCE_MAX_SAMPLINGRATE)
    if (gForce) {
      int32_t currentX = 0, currentY = 0;
      ForceSample s = gForce->readForce();
      if (s.hasData) {
        currentX = s.xRaw;
        currentY = s.yRaw;
        if (currentSensorDataCore1.calib_now) {
          currentX = 0;
          currentY = 0;
        }
        mutex_enter_blocking(&(currentSensorDataCore1.sensorDataMutex));
        currentSensorDataCore1.xRaw =  currentX;
        currentSensorDataCore1.yRaw =  currentY;
        mutex_exit(&(currentSensorDataCore1.sensorDataMutex));
      }
    }
  NB_DELAY_END
}


/**
   @name readButtons
   @brief updates and processes new button sensor values
   @return none
*/
void readButtons()
{
  NB_DELAY_START(buttons_ts, 1000 / BUTTONS_MAX_SAMPLINGRATE)
    // read buttons from generic I2C addon if available
    if (gButtons) {
      ButtonSample bs = gButtons->readButtons();
      if (bs.hasData) {
        mutex_enter_blocking(&(currentSensorDataCore1.sensorDataMutex));
        currentSensorDataCore1.I2CButtonStates = bs.buttons;
        mutex_exit(&(currentSensorDataCore1.sensorDataMutex));
      }
    }
  NB_DELAY_END
}


/**
   @name calculateDirection
   @brief calculates angular direction and force for current x/y sensor values. [called from core 0]
   @param sensorData: pointer to SensorData struct, used by core0
   @return none
*/
void calculateDirection(struct CurrentState * currentState)
{
  currentState->forceRaw = sqrtf(currentState->xRaw * currentState->xRaw + currentState->yRaw * currentState->yRaw);
  if (currentState->forceRaw != 0) {
    currentState->angle = atan2f ((float)currentState->yRaw / currentState->forceRaw, (float)currentState->xRaw / currentState->forceRaw );

    // get 8 directions
    currentState->dir = (180 + 22 + (int)(currentState->angle * 57.29578)) / 45 + 1; // translate rad to deg and make 8 sections
    if (currentState->dir > 8) currentState->dir = 1;
  }
}

/**
   @name applyDeadzone
   @brief calculates deadzone and respective x/y/force values (in sensorData struct). [called from core 0]
   @param sensorData: pointer to SensorData struct, used by core0
   @param slotSettings: pointer to SlotSettings struct, used by core0
   @return none
*/
void applyDeadzone(struct CurrentState * currentState, struct SlotSettings * slotSettings)
{
  if (slotSettings->stickMode == STICKMODE_ALTERNATIVE) {

    // rectangular deadzone for alternative modes
    if (currentState->xRaw < -slotSettings->dx)
      currentState->x = currentState->xRaw + slotSettings->dx; // apply deadzone values x direction
    else if (currentState->xRaw > slotSettings->dx)
      currentState->x = currentState->xRaw - slotSettings->dx;
    else currentState->x = 0;

    if (currentState->yRaw < -slotSettings->dy)
      currentState->y = currentState->yRaw + slotSettings->dy; // apply deadzone values y direction
    else if (currentState->yRaw > slotSettings->dy)
      currentState->y = currentState->yRaw - slotSettings->dy;
    else currentState->y = 0;

  } else {

    //  circular deadzone for mouse control
    if (currentState->forceRaw != 0) {
      float a = slotSettings->dx > 0 ? slotSettings->dx : 1 ;
      float b = slotSettings->dy > 0 ? slotSettings->dy : 1 ;
      float s = sinf(currentState->angle);
      float c = cosf(currentState->angle);
      currentState->deadZone =  a * b / sqrtf(a * a * s * s + b * b * c * c); // ellipse equation, polar form
    }
    else currentState->deadZone = slotSettings->dx;

    currentState->force = (currentState->forceRaw < currentState->deadZone) ? 0 : currentState->forceRaw - currentState->deadZone;
    currentState->x = (int) (currentState->force * cosf(currentState->angle));
    currentState->y = (int) (currentState->force * sinf(currentState->angle));
  }
}

/**
   @name checkSensorWatchdog
   @brief checks if an integer value which should be periodically reset when I2C-sensordata is ready exceeds a certain value
   @return true: value within normal range  false: value exceeded -> action must be taken
*/
uint8_t checkSensorWatchdog() {
  static int sensorWatchdog = -1;

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


/**
   @name testI2CDevices
   @brief checks if I2C devices are present on the given interface and fills the device_list array accordingly
   @param interface: TwoWire interface to check (Wire or Wire1)
   @param resetIfChanged: if true, a reset will be performed if a change in the device list is detected
   @return number of detected devices
*/
int testI2Cdevices(TwoWire *interface, bool resetIfChanged) {

  static uint8_t devicesWire[8] = {0};     // active I2C devices on Wire
  static uint8_t devicesWire1[8] = {0};    // active I2C devices on Wire1

  uint8_t *device_list = (interface == &Wire) ? devicesWire : devicesWire1;  

  //create a copy of list before checking again
  uint8_t olddevices[8];
  memcpy(olddevices, device_list, 8);

  int devicenr = 0;     //currently used I2C address index of supported_devices[]
  int devicecount = 0;  //count of found devices, used as offset in active devices (devicesWire[])

  //reset device list
  memset(device_list,0,8);

  while(supported_devices[devicenr] != 0x00) { //test until address is 0x00
    interface->beginTransmission(supported_devices[devicenr]);
    uint8_t result = interface->endTransmission();
    //if found: add to array of active devices
    if (result == 0) {
      #ifdef DEBUG_OUTPUT_I2C_SCAN
        DEBUG_OUT.print("Found device @0x");
        DEBUG_OUT.println(supported_devices[devicenr],HEX);
      #endif
      device_list[devicecount] = supported_devices[devicenr];
      devicecount++;
    }
    devicenr++;
  }

  if (resetIfChanged) {
    if (memcmp(olddevices, device_list, 8) != 0) { 
      Serial.println("Devices on I2C bus changed -> restarting!");
      watchdog_reboot(0, 0, 10);
      while (1);
    }
  }
  return devicecount;
}
