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

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "FabiWare.h"        // we need the SensorData and SlotSettings structure definitions
#include "Wire.h"            // MPRLS pressure sensor and NAU7802 sensor use I2C
#include <hardware/adc.h>    // for directly reading the internal ADC
#include <LoadcellSensor.h>  // for signal conditioning

// sensor-specific parameters moved into their respective module headers

/**** general sensor related settings */
#define SENSOR_WATCHDOG_TIMEOUT    3000   // watchdog reset time (no NAU sensor data for x millsec. resets device)
#define PRESSURE_MAX_SAMPLINGRATE   100   // maximum sampling frequency of pressure sensors
#define FORCE_MAX_SAMPLINGRATE      100   // maximum sampling frequency of force sensors
#define BUTTONS_MAX_SAMPLINGRATE    100   // maximum sampling frequency of button sensors


/**** detection threshold for floating ADC pin */
#define PINFLOAT_DIFFERENCE_THRESHOLD 500


/**
   @brief Used pressure sensor type. We can use either an analog sensor connected to an ADC pin 
   (e.g. the MPXV7007GP) or the DPS310 / MPRLS sensor boards with I2C
*/
typedef enum {PRESSURE_INTERNAL_ADC, PRESSURE_DPS310, PRESSURE_MPRLS, PRESSURE_FABI_GENERIC, PRESSURE_NONE} pressureSensor_type_t;


/**
   @brief Used force sensor type. We can use the Sensorboard (NAU7802 with Strain gauge or resistive sensors)
   or 2 internal ADC channels.
*/
typedef enum {FORCE_INTERNAL_ADC, FORCE_NAU7802, FORCE_FABI_GENERIC, FORCE_NONE} forceSensor_type_t;

/**
   @brief Data structure for sensor updates performed by Core1.
*/
struct CurrentSensorDataCore1 {
  int xRaw,yRaw; // current y and x force sensor values
  int pressure;  // current pressur value   
  uint16_t calib_now;    // calibration counter, to initiate a calibration procedure
  int calibX;  // x-axis calibration for using internal ADC
  int calibY;  // y-axis calibration for using internal ADC
  uint8_t I2CButtonStates;      // current button states from I2C FABI generic sensor interface
  pressureSensor_type_t pressureSensorType;
  forceSensor_type_t forceSensorType;
  mutex_t sensorDataMutex; // for synchronization of data access between cores
};

//extern pressureSensor_type_t pressureSensorType;
//extern forceSensor_type_t forceSensorType;

extern struct CurrentSensorDataCore1 currentSensorDataCore1;

// expose global signal processors for NAU (X/Y) and pressure (PS)
extern LoadcellSensor XS;
extern LoadcellSensor YS;
extern LoadcellSensor PS;

/**
   @name initSensors
   @brief initializes the sensors (pressure & force)
   @return none
*/
void initSensors();

/**
   @name getForceSensorType
   @brief returns the type of the available force sensor
   @return forceSensor_type_t
*/
forceSensor_type_t getForceSensorType();

/**
   @name getPressureSensorType
   @brief returns the type of the available pressure sensor
   @return pressureSensor_type_t
*/
pressureSensor_type_t getPressureSensorType();

/**
   @name startSensorCalibration
   @brief prepares a sensor calibration 
   @return none
*/
void startSensorCalibration();

/**
   @name checkSensorCalibration
   @brief calibrates the offset values for the sensors (pressure & force) if necessary
   @return none
*/
void checkSensorCalibration();

/**
   @name readPressure
   @brief read current pressure sensor (either DPS310 or MPRLS)
   @note For the MPRLS sensor, it returns the previous measurement & triggers a new one!
   @return none
*/
void readPressure();

/**
   @name readForce
   @brief read current force sensors (might be FSR or RES-DMS)
   @return none
*/
void readForce();

/**
   @name readButtons
   @brief read current button sensor states
   @return none
*/
void readButtons();


/**
   @name calculateDirection
   @brief calculates angular direction and force for current x/y sensor values
   @return none
*/
void calculateDirection(struct CurrentState * sensorData);

/**
   @name applyDeadzone
   @brief applies deadzone algorithm (elliptical or rectangular) to raw x/y sensor data
   @return none
*/
void applyDeadzone(struct CurrentState * sensorData, struct SlotSettings * slotSettings);

/**
   @name setSensorBoard
   @brief activates a certain parameters profile for signal processing, depending on the selected senosorboard ID
   @param sensorBoardID: the ID of the sensorboard signal processing profile  (e.g. SENSOR_BOARD_STRAINGAUGE)
   @return none
*/
void setSensorBoard(int sensorBoardID);

/**
   @name checkSensorWatchdog
   @brief checks if an integer value which should be periodically reset when I2C-sensordata is ready exceeds a certain value
   @return true: value within normal range  false: value exceeded -> action must be taken
*/
uint8_t checkSensorWatchdog();

/**
   @name testI2CDevices
   @brief checks if I2C devices are present on the given interface and fills the device_list array accordingly
   @param interface: TwoWire interface to check (Wire or Wire1)
   @param resetIfChanged: if true, a reset will be performed if a change in the device list is detected
   @return number of detected devices
*/
int testI2Cdevices(TwoWire *interface, bool resetIfChanged = false);

#endif /* _SENSORS_H_ */
