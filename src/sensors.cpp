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

Adafruit_NAU7802 nau;
LoadcellSensor XS, YS;
LoadcellSensor PS;
int sensorWatchdog = -1;

#define MPRLS_READ_TIMEOUT (20)     ///< millis
#define MPRLS_STATUS_POWERED (0x40) ///< Status SPI powered bit
#define MPRLS_STATUS_BUSY (0x20)    ///< Status busy bit
#define MPRLS_STATUS_FAILED (0x04)  ///< Status bit for integrity fail
#define MPRLS_STATUS_MATHSAT (0x01) ///< Status bit for math saturation
#define MPRLS_STATUS_MASK (0b01100101) ///< Sensor status mask: only these bits are set


#define DPS_R_PSR_B2 0x00
#define DPS_R_PSR_B1 0x01
#define DPS_R_PSR_B0 0x02
#define DPS_R_PRS_CFG 0x06
#define DPS_R_MEAS_CFG 0x08
#define DPS_R_CFG_REG 0x09
#define DPS_R_RESET 0x0C

/**
   @brief Global variables for passing sensor data from the ISR
*/
uint8_t channel, newData = 0;
int32_t nau_x = 0, nau_y = 0;
int32_t pressure_rawval = 512;
uint8_t reportXValues = 0, reportYValues = 0;

struct CurrentSensorDataCore1 currentSensorDataCore1 {        
  .xRaw=0, .yRaw=0, .pressure=512, 
  .calib_now=CALIBRATION_PERIOD,     // calibrate sensors after startup !
  .calibX=512, .calibY=512
};


/**
   @name configureNAU
   @brief initialises the NAU7802 chip for desired sampling rate and gain
   @return none
*/
void configureNAU() {
  nau.setLDO(NAU7802_3V0);   // NAU7802_2V7, NAU7802_2V4
  nau.setGain(NAU7802_GAIN_128);  // NAU7802_GAIN_64, NAU7802_GAIN_32
  nau.setRate(NAU7802_RATE_320SPS);  // NAU7802_RATE_80SPS
  nau.setPGACap(NAU7802_CAP_OFF); //disable PGA capacitor on channel 2

  // trigger internal calibration
  while (! nau.calibrate(NAU7802_CALMOD_INTERNAL)) {
    Serial.println("SEN: Failed to set NAU internal calibration, retrying!");
    delay(1000);
  }

  // flush ADC
  for (uint8_t i = 0; i < 10; i++) {
    while (! nau.available()) delay(1);
    nau.read();
  }
}

/**
   @name configureDPS
   @brief initialises the DPS310 chip for desired sampling rate and config
   @return none
*/
void configureDPS() {
  //set corresponding config bytes
  Wire1.beginTransmission(DPS310_ADDR);
  Wire1.write(DPS_R_PRS_CFG);
  //128Hz measurements, no oversampling
  Wire1.write( (0b111 << 4) | (0b0000) );
  Wire1.endTransmission();
    
  Wire1.beginTransmission(DPS310_ADDR);
  Wire1.write(DPS_R_MEAS_CFG);
  //start continous pressure measurement (background mode)
  Wire1.write(0b101);
  Wire1.endTransmission();
}


/**
   @name initSensors
   @brief initialises I2C interface, prepares NAU and MPRLS/DPS310 readouts. [called from core 1]
   @return none
*/
void initSensors()
{
  currentSensorDataCore1.pressureSensorType = PRESSURE_NONE;
  //detect if there is a DPS310 sensor connected to I2C (Wire)

  Wire1.beginTransmission(DPS310_ADDR);
  uint8_t result = Wire1.endTransmission();
  if (result == 0) {
  #ifdef DEBUG_OUTPUT_SENSORS
    Serial.println("SEN: found DPS310");
  #endif
    // we found the DPS310 sensor, so use it!
    currentSensorDataCore1.pressureSensorType = PRESSURE_DPS310;
    configureDPS();
    #ifdef DEBUG_OUTPUT_SENSORS
        Serial.println("SEN: setup DPS310 finished");
    #endif
  } else {
    #ifdef DEBUG_OUTPUT_SENSORS
        Serial.println("SEN: cannot find DPS310");
    #endif
  
    //detect if there is a MPRLS sensor connected to I2C (Wire)
    Wire1.beginTransmission(MPRLS_ADDR);
    result = Wire1.endTransmission();
    if (result == 0) {
    #ifdef DEBUG_OUTPUT_SENSORS
      Serial.println("SEN: found MPRLS");
    #endif
      // we found the MPRLS sensor, so use it!
      currentSensorDataCore1.pressureSensorType = PRESSURE_MPRLS;
    } else {
      #ifdef DEBUG_OUTPUT_SENSORS
        Serial.println("SEN: cannot find MPRLS");
      #endif
      // check if an analog pressure sensor (e.g. MPX7007) is connected to the internal ADC
      if (!isAnalogPinFloating(ANALOG_PRESSURE_SENSOR_PIN)) {
        #ifdef DEBUG_OUTPUT_SENSORS
          Serial.println("SEN: Pressure sensor connected to internal ADC");
        #endif
        currentSensorDataCore1.pressureSensorType = PRESSURE_INTERNAL_ADC;
      }
    }
  }

  if (currentSensorDataCore1.pressureSensorType != PRESSURE_NONE) {
    // set signal processing parameters for sip/puff pressure sensor
    PS.setGain(1.0);  // adjust gain for pressure sensor
    PS.setSampleRate(PRESSURE_SAMPLINGRATE);    
    PS.setBaselineLowpass(0.4);
    PS.setNoiseLowpass(10.0);

    // PS.setStartupTime(2000);
    PS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE);
    PS.setActivityLowpass(1);
    PS.setIdleDetectionPeriod(500);
    PS.setIdleDetectionThreshold(500);
  } else {
  #ifdef DEBUG_OUTPUT_SENSORS
    Serial.println("SEN: no pressure sensor connected");
  #endif     
  } 

  currentSensorDataCore1.forceSensorType = FORCE_NONE;
  //NAU7802 init
  if (nau.begin(&Wire1)) {
    #ifdef DEBUG_OUTPUT_SENSORS
        Serial.println("SEN: Found NAU7802");
    #endif
    currentSensorDataCore1.forceSensorType = FORCE_NAU7802;
    pinMode (DRDY_PIN, INPUT);
    nau.setChannel(NAU7802_CHANNEL1);
    configureNAU();
    nau.setChannel(NAU7802_CHANNEL2);
    configureNAU();
    nau.setChannel(NAU7802_CHANNEL1);
    channel = 1;

    // set default signal processing parameters for X and Y axis (NAU loadcell amplifier)
    XS.setThresholdDecay(0.95);         YS.setThresholdDecay(0.95);        // threshold adaptation decrease over time
    XS.setBaselineLowpass(0.25);        YS.setBaselineLowpass(0.25);       // low-pass cutoff frequency for the baseline adjustment
    XS.setNoiseLowpass(3);              YS.setNoiseLowpass(3);             // low-pass cutoff frequency for valid signal
    XS.setActivityLowpass(2);           YS.setActivityLowpass(2);          // low-pass cutoff frequency for idle detection / autocalibration
    XS.setIdleDetectionPeriod(1000);    YS.setIdleDetectionPeriod(1000);   // 1 second idle detection period (autocalibration)
    XS.setIdleDetectionThreshold(3000); YS.setIdleDetectionThreshold(3000);// gain-compensated threshold value for idle detection (autocalibration)
    XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE);
    YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE);             // enable autocalibration
  } else {
    #ifdef DEBUG_OUTPUT_SENSORS
      Serial.println("SEN: cannot find NAU7802 sensorboard");
    #endif

    // check if an analog x/y sensor (e.g. a joystick module) is connected to the internal ADC
    if ((!isAnalogPinFloating(ANALOG_FORCE_SENSOR_X_PIN)) && (!isAnalogPinFloating(ANALOG_FORCE_SENSOR_Y_PIN))) {
      #ifdef DEBUG_OUTPUT_SENSORS
        Serial.println("SEN: Force sensor connected to internal ADC");
      #endif
      currentSensorDataCore1.forceSensorType = FORCE_INTERNAL_ADC;
      #ifndef FLIPMOUSE
        // in case the FABI3 PCB is used, we cannot use internal ADC0 for force and pressure ...
        if (currentSensorDataCore1.pressureSensorType == PRESSURE_INTERNAL_ADC) { 
          currentSensorDataCore1.pressureSensorType = PRESSURE_NONE;
        }
      #endif
    }
    else {
     #ifdef DEBUG_OUTPUT_SENSORS
      Serial.println("SEN: no force sensor connected");
    #endif     
    } 
  }
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
  if(currentSensorDataCore1.calib_now != CALIBRATION_PERIOD/2) return;

  // calibrate sensors in the middle of the calibration period
  switch (currentSensorDataCore1.forceSensorType) {
    case FORCE_INTERNAL_ADC: 
          currentSensorDataCore1.calibX = analogRead (ANALOG_FORCE_SENSOR_X_PIN);
          currentSensorDataCore1.calibY = analogRead (ANALOG_FORCE_SENSOR_Y_PIN);
          break;

    case FORCE_NAU7802:
          XS.calib();
          YS.calib();
          break;   
  }

  switch (currentSensorDataCore1.pressureSensorType) {
    case PRESSURE_INTERNAL_ADC: 
    case PRESSURE_DPS310:
    case PRESSURE_MPRLS: 
          PS.calib();
          break;
  }
}


/**
   @name getMPRLSValue
   @brief called periodically in order to read out MPRLS pressure data
          expected sampling rate ca. 100 Hz
   @param newVal: pointer where result will be stored
   @return status byte of MPRLS
*/
int getMPRLSValue(int32_t * newVal) {
  uint8_t buffer[4]  = {0};

  // request status byte
  // myWire.requestFrom(MPRLS_ADDR,1);
  // buffer[0] = myWire.read();

  //request all 4 bytes
  Wire1.requestFrom(MPRLS_ADDR, 4);
  for (uint8_t i = 0; i < 4; i++) buffer[i] = Wire1.read();

  // update value (ignore status byte errors but return the status byte!)
  *newVal = (uint32_t(buffer[1]) << 16) | (uint32_t(buffer[2]) << 8) | (uint32_t(buffer[3]));

  //trigger new conversion
  Wire1.beginTransmission(MPRLS_ADDR);
  Wire1.write(0xAA);
  Wire1.write(0);
  Wire1.write(0);
  Wire1.endTransmission();

  return (buffer[0]);
}

static int32_t twosComplement(int32_t val, uint8_t bits) {
  if (val & ((uint32_t)0x01 << (bits - 1))) {
    val -= (uint32_t)0x01 << bits;
  }
  return val;
}

/**
   @name getDPSValue
   @brief called periodically in order to read out DPS310 pressure data
          expected sampling rate ca. 100 Hz
   @param newVal: pointer where result will be stored
   @return status byte of DPS310
*/
int getDPSValue(int32_t * newVal) {
  //set address to first data register (Byte 2)
  Wire1.beginTransmission(DPS310_ADDR);
  Wire1.write(DPS_R_PSR_B2);
  
  //necessary?
  Wire1.endTransmission();
  
  Wire1.requestFrom(DPS310_ADDR, 3);
  uint8_t buffer[3]  = {0};
  for (uint8_t i = 0; i < 3; i++) buffer[i] = Wire1.read();


  // update value (ignore status byte errors but return the status byte!)
  int32_t r_p = (uint32_t(buffer[0]) << 16) | (uint32_t(buffer[1]) << 8) | (uint32_t(buffer[2]));
  *newVal = twosComplement(r_p,24);

  return (0);
}


/**
   @name getNAUValues
   @brief called if new data from NAU7802 is available.
          reads NAU data, changes NAU channel and reads MPRLS data
          expected sampling rate ca. 30 Hz per channel (-> 60 Hz interpolated)
   @param actX, actY: pointers where results will be stored
   @return none
*/
void getNAUValues(int32_t * actX, int32_t * actY) {
  static int32_t xChange = 0, yChange = 0;

  if (channel == 1) {
    xChange = (XS.process(nau.read()) - *actX) / 2;
    nau.setChannel(NAU7802_CHANNEL2);
    *actX += xChange;
    *actY += yChange;
    channel = 2;
  }
  else {
    yChange = (YS.process(nau.read()) - *actY) / 2;
    nau.setChannel(NAU7802_CHANNEL1);
    *actX += xChange;
    *actY += yChange;
    channel = 1;
  }
  sensorWatchdog = 0; // we got data, reset watchdog counter!
}


/**
   @name readPressure
   @brief updates and processes new pressure sensor values form MPRLS. [called from core 1]
   @return none
*/
void readPressure()
{
  int actPressure = 512;

  NB_DELAY_START(pressure_ts,1000/PRESSURE_SAMPLINGRATE)
    switch (currentSensorDataCore1.pressureSensorType)
    {
      case PRESSURE_MPRLS:
      {
          // get new value from MPRLS chip
          int mprlsStatus = getMPRLSValue(&pressure_rawval);

  #ifdef DEBUG_MPRLS_ERRORFLAGS
          // any errors?  - just indicate them via serial message
          if (mprlsStatus & MPRLS_STATUS_BUSY) {
            Serial.println("MPRLS: busy");
          }
          if (mprlsStatus & MPRLS_STATUS_FAILED) {
            Serial.println("MPRLS:failed");
          }
          if (mprlsStatus & MPRLS_STATUS_MATHSAT) {
            Serial.println("MPRLS:saturated");
          }
  #endif

          int med = calculateMedian(pressure_rawval);
          if (abs(med - pressure_rawval) >  SPIKE_DETECTION_THRESHOLD) {
            pressure_rawval = med;
          }

          // calculate filtered pressure value, apply signal conditioning
          int mprls_filtered = PS.process(pressure_rawval);
          if (mprls_filtered > 0) mprls_filtered = sqrt(mprls_filtered);
          if (mprls_filtered < 0) mprls_filtered = -sqrt(-mprls_filtered);

          actPressure = 512 + mprls_filtered / MPRLS_DIVIDER;
        }
        break;

      case PRESSURE_DPS310:
        {
          // get new value from DPS chip
          getDPSValue(&pressure_rawval);
          pressure_rawval *= DPS_SCALEFACTOR;
          
          int med = calculateMedian(pressure_rawval);
          if (abs(med - pressure_rawval) >  DPS_SPIKE_DETECTION_THRESHOLD) {
            pressure_rawval = med;
          }

          // calculate filtered pressure value, apply signal conditioning
          int dps_filtered = PS.process(pressure_rawval);
          if (dps_filtered > 0) dps_filtered = sqrt(dps_filtered);
          if (dps_filtered < 0) dps_filtered = -sqrt(-dps_filtered);

          actPressure = 512 + dps_filtered / DPS_DIVIDER;
        }
        break;
      
      case PRESSURE_INTERNAL_ADC:
        actPressure = analogRead(ANALOG_PRESSURE_SENSOR_PIN);
        break;

      default:
      case PRESSURE_NONE:
        actPressure = 512;
        break;
    }
    
    // clamp to 1/1022 (allows disabling strong sip/puff)
    if (actPressure < 1) actPressure = 1;
    if (actPressure > 1022) actPressure = 1022;

    // during calibration period: set pressure to center (bypass)
    if (currentSensorDataCore1.calib_now) actPressure = 512;

    // here we provide new pressure values for further processing by core 0 !
    mutex_enter_blocking(&(currentSensorDataCore1.sensorDataMutex));
    currentSensorDataCore1.pressure = actPressure;
    mutex_exit(&(currentSensorDataCore1.sensorDataMutex));
  NB_DELAY_END
}

/**
   @name readForce
   @brief updates and processes new  x/y sensor values from NAU7802. [called from core 1]
   @return none
*/
void readForce()
{
  static uint8_t printCount = 0;
  uint8_t newData=0;
  int32_t currentX = 0, currentY = 0;

  switch (currentSensorDataCore1.forceSensorType)
  {
    case FORCE_INTERNAL_ADC:
      newData=1;
      currentX = analogRead (ANALOG_FORCE_SENSOR_X_PIN)-currentSensorDataCore1.calibX;
      currentY = analogRead (ANALOG_FORCE_SENSOR_Y_PIN)-currentSensorDataCore1.calibY;
      break;

    case FORCE_NAU7802:
      // if the Data Ready Pin of NAU chip signals new data: get force sensor values
      if (digitalRead(DRDY_PIN) == HIGH)  { 
        newData=1;
        // get new values from NAU chip
        getNAUValues (&nau_x, &nau_y);

        // prevent unintended baseline correction if other axis is moving
        YS.lockBaseline(XS.isMoving());
        XS.lockBaseline(YS.isMoving());

        // report values if enabled (only every second iteration as we interpolate 2 values)
        if (printCount++ > 1) {
          printCount = 0;
          if (reportXValues) XS.printValues(0x07, 40000);
          if (reportYValues) YS.printValues(0x07, 40000);
          if (reportXValues || reportYValues) Serial.println("");
        }

        if (currentSensorDataCore1.calib_now) {
          // during calibration period: set X/Y value to zero
          currentX = 0;
          currentY = 0;
        }
        else {
          currentX = nau_x / NAU_DIVIDER;
          currentY = nau_y / NAU_DIVIDER;
        }
      }
      break;

    case FORCE_NONE:
    default:
      break;
  }

  if (newData) {
    // here we provide new X/Y values for further processing by core 0 !
    mutex_enter_blocking(&(currentSensorDataCore1.sensorDataMutex));
    currentSensorDataCore1.xRaw =  currentX;
    currentSensorDataCore1.yRaw =  currentY;
    mutex_exit(&(currentSensorDataCore1.sensorDataMutex));
  }
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
   @name setSensorBoard
   @brief activates a certain parameters profile for signal processing, depending on the selected senosorboard ID
   @param sensorBoardID: the ID of the sensorboard signal processing profile (e.g. SENSOR_BOARD_STRAINGAUGE)
   @return none
*/
void setSensorBoard(int sensorBoardID)
{
  switch (sensorBoardID) {
    case SENSORBOARD_SG_HIGH:
      XS.setGain(0.5);                    YS.setGain(0.5);
      XS.setMovementThreshold(2000);      YS.setMovementThreshold(2000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(500);  YS.setIdleDetectionThreshold(500);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      break;
    case SENSORBOARD_SG_MEDIUM:
      XS.setGain(0.25);                   YS.setGain(0.25);
      XS.setMovementThreshold(1500);      YS.setMovementThreshold(1500);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(250);  YS.setIdleDetectionThreshold(250);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      break;
    case SENSORBOARD_SG_LOW:
      XS.setGain(0.1);                    YS.setGain(0.1);
      XS.setMovementThreshold(2000);      YS.setMovementThreshold(2000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(120);  YS.setIdleDetectionThreshold(120);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      break;
    case SENSORBOARD_SG_VERY_LOW:
      XS.setGain(0.05);                   YS.setGain(0.05);
      XS.setMovementThreshold(2500);      YS.setMovementThreshold(2500);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(60);   YS.setIdleDetectionThreshold(60);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_RESET_BASELINE); 
      break;
    case SENSORBOARD_SMD_HIGH:
      XS.setGain(-1.0);                   YS.setGain(-1.5);
      XS.setMovementThreshold(2000);      YS.setMovementThreshold(3000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(500);  YS.setIdleDetectionThreshold(750);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
      break;
    case SENSORBOARD_SMD_MEDIUM:
      XS.setGain(-1.0);                   YS.setGain(-1.5);
      XS.setMovementThreshold(4000);      YS.setMovementThreshold(6000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(500);  YS.setIdleDetectionThreshold(750);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
      break;
    case SENSORBOARD_SMD_LOW:
      XS.setGain(-0.5);                   YS.setGain(-0.75);
      XS.setMovementThreshold(3000);      YS.setMovementThreshold(4000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(350);  YS.setIdleDetectionThreshold(500);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
      break;
    case SENSORBOARD_SMD_VERY_LOW:
      XS.setGain(-0.5);                   YS.setGain(-0.75);
      XS.setMovementThreshold(4000);      YS.setMovementThreshold(6000);
      XS.setIdleDetectionPeriod(200);     YS.setIdleDetectionPeriod(200);
      XS.setIdleDetectionThreshold(350);  YS.setIdleDetectionThreshold(500);
      XS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD); 
      YS.setAutoCalibrationMode(AUTOCALIBRATION_ADAPT_THRESHOLD);
      break;
    //  signal reporting settings
    case SENSORBOARD_REPORT_X:
      reportXValues = !reportXValues;
      break;
    case SENSORBOARD_REPORT_Y:
      reportYValues = !reportYValues;
      break;
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
   @name swap
   @brief swaps two integers, helper function for median
*/
void swap(int *a, int *b) {
  int temp = *a;
  *a = *b;
  *b = temp;
}


/**
   @name quick sort
   @brief sorts array of integers, helper function for median
*/
void quickSort(int values[], int left, int right) {
  if (left >= right) {
    return;
  }

  int pivot = values[left];
  int i = left + 1;
  int j = right;

  while (1) {
    while (i <= j && values[i] <= pivot) i++;
    while (j >= i && values[j] >= pivot) j--;
    if (i > j)
      break;

    swap(&values[i], &values[j]);
  }
  swap(&values[left], &values[j]);

  quickSort(values, left, j - 1);
  quickSort(values, j + 1, right);
}


/**
   @name calculateMedian
   @brief calculates median value (attention: uses static buffer, useable for just 1 signal!)
   @param value: next sample
   @return median value
*/
int calculateMedian(int value) {
  static int medianBuf[MEDIAN_VALUES] = {0};
  static int medianBufPos = 0;
  int values[MEDIAN_VALUES];

  medianBuf[medianBufPos] = value;
  medianBufPos++;
  if (medianBufPos >= MEDIAN_VALUES) medianBufPos = 0;
  memcpy(values, medianBuf, sizeof(int)*MEDIAN_VALUES);

  quickSort(values, 0, MEDIAN_VALUES - 1);

  if (MEDIAN_VALUES % 2 == 0) {
    int midIndex1 = MEDIAN_VALUES / 2 - 1;
    int midIndex2 = MEDIAN_VALUES / 2;
    return (values[midIndex1] + values[midIndex2]) / 2;
  } else {
    int midIndex = MEDIAN_VALUES / 2;
    return values[midIndex];
  }
}


/**
   @name isAnalogPinFloating
   @brief performs a check if an ADC pin is floating or a sensor is connected
   @return true if the ADC pin is floating, false otherwise
*/

int isAnalogPinFloating (int pin) {
  int x,y;
  pinMode(pin, INPUT_PULLUP);
  analogRead (pin);
  x= adc_read();
  
  pinMode(pin, INPUT_PULLDOWN);
  analogRead (pin);
  y= adc_read();

  if (x-y < PINFLOAT_DIFFERENCE_THRESHOLD) return(false);
  return(true);
}
