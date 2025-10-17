/*
    FabiWare - AsTeRICS Foundation
    For more info please visit: https://www.asterics-foundation.org

    Module: lpwFuncs.h - Header file for battery and power management functions

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; See the GNU General Public License:
    http://www.gnu.org/licenses/gpl-3.0.en.html

*/

// Referred datasheet RP2350: https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf

#ifndef LPW_FUNCS_H
#define LPW_FUNCS_H

#ifdef RP2350   // low power support only available for RP2350

    #include "hardware/gpio.h"

    #define inactivityTimeMinutes 3
    #define inactivityTimeSeconds 0

    #define BATTERY_AVERAGING     4    // number of battery measurements to be averaged  (averaging period see BATTERY_UPDATE_INTERVAL, e.g. every 500ms)
    #define MCPSTAT_HIGHZ     0
    #define MCPSTAT_LOW       1
    #define MCPSTAT_HIGH      2
    #define MCPSTAT_UNDEFINED 3
    #define CYW43_WL_GPIO_VBUS_PIN 2

    bool detectUSB();
    void enable3V3();  // enable the 3.3V power rail
    void disable3V3(); // disable the 3.3V power rail
    void enableBatteryMeasurement();      // enables battery voltage measurement circuitry 
    void disableBatteryMeasurement();     // disables battery voltage measurement circuitry
    uint8_t batteryPresenceDetector();    // periodically checks battery status, including battery presence
    int8_t getBatteryPercentage();        // checks battery-% based on a predetermined interval; presupposes presence of battery
    uint16_t readPercentage();            // reads and returns mapped battery percentage (0-100%)
    void performBatteryManagement();      // must be called periodically (e.g. twice a second) from the main loop

    void inactivityHandler();                     // prepares sleep mode
    void dormantUntilInterrupt(int8_t *wake_interrupt_gpios, int8_t amt_gpios); // puts the device into dormant mode
    void userActivity();                          // handles user interaction: resets inactivity counter

    #endif // RP2350
#endif // LPW_FUNCS_H
