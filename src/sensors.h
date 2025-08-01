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
#include <Adafruit_NAU7802.h>  //NAU7802 library (Benjamin Aigner's fork with channel change feature)

/**** sensor GPIOs & addresses */

#define MPRLS_ADDR 0x18          // I2C address of the MPRLS pressure sensor 
#define DPS310_ADDR 0x77         // I2C address of the DPS310 pressure sensor 

/**** MPRLS related signal shaping parameters */
#define MPRLS_DIVIDER 2                 // divider for the MPRLS raw value.
#define MEDIAN_VALUES 5                 // number of values used for median-based spike filter (for MPRLS sensor)
#define SPIKE_DETECTION_THRESHOLD 1000  // distance from median value which classifies a spike

/**** DPS310 related signal shaping parameters */
#define DPS_SCALEFACTOR  -20            // scale factor for aligning DPS with MPRLS raw values
#define DPS_DIVIDER  3                  // divider for the DPS310 values
#define DPS_SPIKE_DETECTION_THRESHOLD 150  // distance from median value which classifies a spike
#define DPS_MEDIAN_VALUES 5                // number of values used for median-based spike filter (for MPRLS sensor)

/**** NAU7802 related signal shaping parameters */
#define NAU_DIVIDER 120                 // divider for the NAU raw values

/**** general sensor related settings */
#define SENSOR_WATCHDOG_TIMEOUT 3000    // watchdog reset time (no NAU sensor data for x millsec. resets device)
#define PRESSURE_SAMPLINGRATE   100     // sampling frequency of pressure sensor (MPRLS or DPS)

/**** detection threshold for floating ADC pin */
#define PINFLOAT_DIFFERENCE_THRESHOLD 500


/**
   @brief Used pressure sensor type. We can use either an analog sensor connected to an ADC pin 
   (e.g. the MPXV7007GP) or the DPS310 / MPRLS sensor boards with I2C
*/
typedef enum {PRESSURE_INTERNAL_ADC, PRESSURE_DPS310, PRESSURE_MPRLS, PRESSURE_NONE} pressureSensor_type_t;


/**
   @brief Used force sensor type. We can use the Sensorboard (NAU7802 with Strain gauge or resistive sensors)
   or 2 internal ADC channels.
*/
typedef enum {FORCE_INTERNAL_ADC, FORCE_NAU7802, FORCE_NONE} forceSensor_type_t;

/**
   @brief Data structure for sensor updates performed by Core1.
*/
struct CurrentSensorDataCore1 {
  int xRaw,yRaw; // current y and x force sensor values
  int pressure;  // current pressur value   
  uint16_t calib_now;    // calibration counter, to initiate a calibration procedure
  int calibX;  // x-axis calibration for using internal ADC
  int calibY;  // y-axis calibration for using internal ADC
  pressureSensor_type_t pressureSensorType;
  forceSensor_type_t forceSensorType;
  mutex_t sensorDataMutex; // for synchronization of data access between cores
};

//extern pressureSensor_type_t pressureSensorType;
//extern forceSensor_type_t forceSensorType;

extern struct CurrentSensorDataCore1 currentSensorDataCore1;

/**
   @brief Sensorboard IDs for different signal processing parameters
*/
#define SENSORBOARD_SG_HIGH      0
#define SENSORBOARD_SG_MEDIUM    1
#define SENSORBOARD_SG_LOW       2
#define SENSORBOARD_SG_VERY_LOW  3
#define SENSORBOARD_SMD_HIGH     4
#define SENSORBOARD_SMD_MEDIUM   5
#define SENSORBOARD_SMD_LOW      6
#define SENSORBOARD_SMD_VERY_LOW 7
#define SENSORBOARD_REPORT_X    10   // enable / disable signal processing values reporting for X axis
#define SENSORBOARD_REPORT_Y    11   // enable / disable signal processing values reporting for Y axis

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
   @name calculateDirection
   @brief calculates angular direction and force for current x/y sensor values
   @return none
*/
void calculateDirection(struct SensorData * sensorData);

/**
   @name applyDeadzone
   @brief applies deadzone algorithm (elliptical or rectangular) to raw x/y sensor data
   @return none
*/
void applyDeadzone(struct SensorData * sensorData, struct SlotSettings * slotSettings);

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
   @name getMPRLSValue
   @brief called periodically in order to read out MPRLS pressure data
          expected sampling rate ca. 100 Hz
   @param newVal: pointer where result will be stored
   @return status byte of MPRLS
*/
int getMPRLSValue(int32_t * newVal);

/**
   @name getNAUValues
   @brief called if new data from NAU7802 is available.
          reads NAU data, changes NAU channel and reads MPRLS data
          expected sampling rate ca. 30 Hz per channel (-> 60 Hz interpolated)
   @param actX, actY: pointers where results will be stored
   @return status byte of MPRLS
*/
void getNAUValues(int32_t * actX, int32_t * actY);


/**
   @name calculateMedian
   @brief calculates median value (attention: uses static buffer, useable for just 1 signal!)
   @param value: next sample
   @return median value
*/
int calculateMedian(int value);


/**
   @name isAnalogPinFloating
   @brief performs a check if an ADC pin is floating or a sensor is connected
   @return true if the ADC pin is floating, false otherwise
*/
int isAnalogPinFloating (int pin);



#endif /* _SENSORS_H_ */
