/*
   FabiWare - AsTeRICS Foundation
   For more info please visit: https://www.asterics-foundation.org

   Module: gpio.h - functions for leds and buttons

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation.
   
   This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; See the GNU General Public License:
	http://www.gnu.org/licenses/gpl-3.0.en.html

*/

#ifndef _GPIO_H_
#define _GPIO_H_

#include <Arduino.h>

/**
   constant definitions for pin assignments
*/

// device-specific settings and pin locations 
#ifdef FLIPMOUSE
  #define NUMBER_OF_PHYSICAL_BUTTONS 3      // number of physical buttons for FlipMouse
  #define PHYSICAL_BUTTON_MAP {17, 28, 20}  // gpio pin assignment for buttons
  #define NEOPIXEL_PIN   15   // physical pin for neopixel LED
  #define TONE_PIN       25   // physical pin for piezo buzzer
  #define IR_LED_PIN     19   //  IR-Led output pin for FlipMouse
  #define DRDY_PIN       21   // Data ready pin of NAU chip for FlipMouse
  #define ANALOG_PRESSURE_SENSOR_PIN A3   // input pin if an analog pressure sensor is used with FlipMouse3 PCB 
#else 
  #define NUMBER_OF_PHYSICAL_BUTTONS 5      // number of physical switches for FABI/FlipPad
  #define PHYSICAL_BUTTON_MAP {11,9,8,4,3}
  #define NEOPIXEL_PIN   10
  #define TONE_PIN        2    
  #define IR_LED_PIN     14
  #define DRDY_PIN       27    // Data ready pin of NAU chip (FABI EXT connector, EXT2 pin)
  #define ANALOG_PRESSURE_SENSOR_PIN A0   // input pin if an analog pressure sensor is used with FABI3 PCB
          // note that A0 is also used by an analog x/y sensor, so both can't be used at the same time

  // pins for battery management system for FABI 
  #define V_BATT_VD_SWITCH_PIN 0    // GPIO pin controlling the voltage divider for battery measurement
  #define V_BATT_MEASURE_PIN  A2    // ADC pin for measuring battery voltage
  #define CHARGE_STAT_PIN      1    // pin connected to the charging status output

  // remap pins for I2C interfaces for FABI / FlipPad
  #define PIN_WIRE0_SDA_ 12
  #define PIN_WIRE0_SCL_ 13
  #define PIN_WIRE1_SDA_ 18
  #define PIN_WIRE1_SCL_ 19

  #define AUDIO_SIGNAL_PIN    6   // PWM audio signal pin
  #define AUDIO_AMP_SD_PIN   15   // PAM audio amp shutdown pin (active low)
#endif


// common pin locations
#define IR_SENSOR_PIN   16   //  input pin of the TSOP IR receiver
#define LDO_ENABLE_PIN   7   // Enable pin for the MIC5504 3,3V regulator (VCC supply for sensors)
#define ANALOG_FORCE_SENSOR_X_PIN A0   // input pin if an analog force sensor is used (x axis) 
#define ANALOG_FORCE_SENSOR_Y_PIN A1   // input pin if an analog force sensor is used (y axis) 


/**
   extern declaration of static variables
   which shall be accessed from other modules
*/
extern int8_t  input_map[NUMBER_OF_PHYSICAL_BUTTONS];  // maps the button number to physical pin

/**
   @name initGPIO
   @brief initializes data direction of gpio pins
   @return none
*/
void initGPIO();

/**
   @name initBlink
   @brief initializes an LED blinking sequence
   @return none
*/
void initBlink(uint8_t count, uint8_t startTime);


/**
   @name updateLeds
   @brief update leds according to current slot number / blinking actions
   @return none
*/
void updateLeds();

/**
   @name setLeds
   @brief set the LEDs (or the Neopixel directly)
   @return none
   @param LEDs (bits 0-3; Neopixel equals bit 0 R; bit 1 G; bit 2 B)
   @note Only used in CIM mode.
*/
void setLeds(uint8_t leds);

#endif
