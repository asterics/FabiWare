/*
  FabiWare - AsTeRICS Foundation
  For more info please visit: https://www.asterics-foundation.org

  Module: lpwFuncs.cpp - C module for battery and power management functions

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; See the GNU General Public License:
  http://www.gnu.org/licenses/gpl-3.0.en.html

*/

#ifdef RP2350   // low power support only available for RP2350

#include "FabiWare.h"
#include "gpio.h"
#include "display.h"
#include "tone.h"
#include "eeprom.h"
#include "pico/cyw43_arch.h"

extern "C" {
  #include "../lib/power/sleep.h"
}

unsigned long inactivityTime=0;  // measures user inactivity (in ms)

/**
 * @name detectUSB
 * @brief Reads the state of defined CYW43 VBUS Pin (2)
 * @return true : USB Connected (VBUS Presence detected)
 *         false : USB Disconnected (VBUS Presence not detected)
 */
bool detectUSB(){
  currentState.usbConnected = cyw43_arch_gpio_get(CYW43_WL_GPIO_VBUS_PIN);
  static bool prevValue = !currentState.usbConnected;
  
  if (prevValue != currentState.usbConnected){
    #ifdef DEBUG_BATTERY_MANAGEMENT
     if (currentState.usbConnected) Serial.println("USB Connected");
     else Serial.println("USB Disconnected");  // note that this only makes sense if Serial connected still present!
    #endif
    prevValue = currentState.usbConnected;
  }
  return currentState.usbConnected;
}

/**
 * @name batteryPresenceDetector
 * @brief Uses a state-machine to check for the battery's presence.
 *        There are three modes that need to be differentiated: 
 *          a. HIGH Z : Shutdown, No battery present 
 *          b. LOW : Charging (Preconditioning, Constant-Current Fast Charge, Constant Voltage) 
 *          c. HIGH : Charge Complete, Standby
 * @return MCPSTAT_HIGHZ : No battery is present / MCP is in shutdown.
 *         MCPSTAT_LOW   : The battery is present, preconditioning / charging.
 *         MCPSTAT_HIGH  : The battery is present, fully charged - standby mode.
 *         MCPSTAT_UNDEFINED  : Do nothing - default return value when state is being determined.
 */
uint8_t batteryPresenceDetector(){
  static uint16_t battStateCounter=0;  // determines when to pull up/down, to read
  static uint16_t  battStateSum=0;  // accumulates digital reads of the MCP Stat pin which eitehr floats, is LOW or HIGH.
  static uint8_t state = MCPSTAT_UNDEFINED; 

  switch (battStateCounter++) {
    case 0:
      battStateSum=0;  // reset accumulator variable
      gpio_disable_pulls(CHARGE_STAT_PIN); gpio_pull_up(CHARGE_STAT_PIN); break;
    case 1:
      battStateSum += gpio_get(CHARGE_STAT_PIN); break;
    case 2:
      gpio_disable_pulls(CHARGE_STAT_PIN); gpio_pull_down(CHARGE_STAT_PIN); break;
    case 3:
      battStateSum += gpio_get(CHARGE_STAT_PIN); 

      // now we can determine the battery state:
      if (battStateSum == 0) {                           // battery is charging
        state = MCPSTAT_LOW;
        #ifdef DEBUG_BATTERY_MANAGEMENT
          Serial.println("LOW   :\tBattery is Charging"); 
        #endif
      }
      else if (battStateSum == 2) {                    // charging complete or standby
        state = MCPSTAT_HIGH;
        #ifdef DEBUG_BATTERY_MANAGEMENT
          Serial.println("HIGH  :\tBattery Charge Completed"); 
        #endif
      } 
      else {
        state = MCPSTAT_HIGHZ;     // shutdown or no battery present
        #ifdef DEBUG_BATTERY_MANAGEMENT
          Serial.println("HIGH Z:\tBattery not present or in shutdown Mode");
        #endif
      }
      battStateCounter=0; // reset state counter!
    break;
  }

  currentState.MCPSTAT=state;
  return state;
}

/**
 * @name getBatteryPercentage
 * @brief returns an averages battery level (averaging count is defined by the constant BATTERY_AVERAGING).
 * @return 0 - 100  : The average of the battery-% read over last n reads.
 *         -1       : Undefined battery level (no battery present / MCP Stat is HighZ).
 */
int8_t getBatteryPercentage(){
  static int battSum=0, battReadCounter=0, result=0;
  if(currentState.MCPSTAT == MCPSTAT_HIGHZ) return (-1);
  battReadCounter++;
  battSum += readPercentage();
  if (battReadCounter >= BATTERY_AVERAGING ){ 
    result = battSum / BATTERY_AVERAGING;
    battSum = 0; battReadCounter = 0;
  }
  return result;
}

/**
 * @name readPercentage
 * @brief Reads and calculates the battery percentage using ADC input.
 * @return uint16_t Battery percentage (0-100%).
 */
uint16_t readPercentage() {
  int16_t value = map(analogRead(V_BATT_MEASURE_PIN), 518, 682, 0, 100);   // map ADC value to battery percentage range
  return constrain(value, 0, 100); 
}

/**
 * @name performBatteryManagement
 * @brief called periodically, to update battery status
 * @return none
 */
void performBatteryManagement()  {
  detectUSB();
  batteryPresenceDetector();
  currentState.currentBattPercent = getBatteryPercentage();
  batteryDisplay();
  #ifdef DEBUG_BATTERY_MANAGEMENT
    Serial.println("Battery level="+String(currentState.currentBattPercent));
  #endif

  // check user inactivity, possibly initiate power save mode

  if (!currentState.usbConnected || DEBUG_SLEEP_WITH_USB) {
    inactivityTime += BATTERY_UPDATE_INTERVAL;
    if (inactivityTime >= (inactivityTimeMinutes*60000 + inactivityTimeSeconds*1000))
      inactivityHandler();  // time to go to sleep...
  } else inactivityTime=0;
}

/**
 * @name enable3V3
 * @brief Enables the 3.3V power rail for peripherals like LCD, Neopixel or external sensors
 */
void enable3V3() {
  gpio_init(LDO_ENABLE_PIN);
  gpio_set_dir(LDO_ENABLE_PIN, true);
  gpio_put(LDO_ENABLE_PIN, true);
}

/**
 * @name disable3V3
 * @brief Disables the 3.3V power rail
 */
void disable3V3() {
  gpio_put(LDO_ENABLE_PIN, false);
  // gpio_set_dir(LDO_ENABLE_PIN, false);
  // gpio_deinit(LDO_ENABLE_PIN);
}

/**
 * @name enableBatteryMeasurement
 * @brief Initializes battery-measurement related pins.
 */
void enableBatteryMeasurement() {
  detectUSB();
  gpio_init(CHARGE_STAT_PIN);
  gpio_init(V_BATT_VD_SWITCH_PIN);
  gpio_set_dir(V_BATT_VD_SWITCH_PIN, false);
  gpio_init(V_BATT_MEASURE_PIN);
  gpio_set_dir(V_BATT_MEASURE_PIN, false);
}

/**
 * @name disableBatteryMeasurement
 * @brief Deinitializes battery-related pins
 */
void disableBatteryMeasurement() {
  gpio_deinit(CHARGE_STAT_PIN);
  gpio_set_dir(V_BATT_MEASURE_PIN, false);
  gpio_deinit(V_BATT_MEASURE_PIN);
  gpio_disable_pulls(V_BATT_MEASURE_PIN);
}


/**
 * @name configureGPIOForSleep
 * @brief Configure all GPIO pins for minimum power consumption during sleep
 */
void configureGPIOForSleep() {
  // Configure all GPIO pins for low power
  for (int pin = 0; pin < 30; pin++) {
    // Skip pins that are used for wake-up (input_map pins)
    bool isWakeupPin = false;
    for (int i = 0; i < NUMBER_OF_WAKEUP_PINS; i++) {
      if (wakeup_pin_map[i] == pin) {
        isWakeupPin = true;
        break;
      }
    }
    
    // Skip special pins
    if (pin == 23 || pin == 24 || pin == 25) continue; // SMPS, debug pins
    if (pin == LDO_ENABLE_PIN) continue; // Keep LDO control pin
    if (pin == V_BATT_MEASURE_PIN || pin == CHARGE_STAT_PIN) continue; // Battery pins
    
    if (isWakeupPin) {
      // Wake-up pins: configure as input with pull-up (buttons typically pull to ground)
      gpio_init(pin);
      gpio_set_dir(pin, GPIO_IN);
      gpio_pull_up(pin);
    } else {
      // All other pins: configure as input with pull-down to prevent floating
      gpio_init(pin);
      gpio_set_dir(pin, GPIO_IN);
      gpio_pull_down(pin);
      // Optionally deinit unused pins completely:
      // gpio_deinit(pin);
    }
  }
   
}

/**
 * @name dormantUntilInterrupt
 * @brief Puts the device into dormant mode until one of the specified GPIO interrupt wakes it up.
 * @param wake_interrupt_gpios array of pins to monitor.
 */
void dormantUntilInterrupt(int8_t *wake_interrupt_gpios, int8_t amt_gpios) {
  sleep_run_from_lposc(); // use low-power oscillator for minimal power consumption
  sleep_goto_dormant_until_pin(wake_interrupt_gpios, amt_gpios, true, false);
  //sleep_goto_dormant_until_edge_high(wake_interrupt_gpios, amt_gpios); // wait for rising edge interrupt
  sleep_power_up(); // restore sys clocks after waking up (using rosc -> jump starts processor)
  delay(100); // allow some time for system to stabilize after restoring sys clocks
}

/**
 * @name inactivityHandler
 * @brief Handles inactivity by transitioning the system to dormant mode.
 */
void inactivityHandler() {
  inactivityTime=0;
  currentState.goingToSleep=1;   // inform core1 loop that we are going to sleep
  saveLastActiveSlotNumber(); 
  displayMessage((char*)"ByeBye");
  delay(2000);   // time for the user to read the message

  Serial.flush();
  Serial.end();
  Wire.end();
  Wire1.end();
 
  digitalWrite(LED_BUILTIN,LOW);  // make sure the internal LED is off 

  #ifdef RP2350
    MouseBLE.end();     // turn off BLE Mouse
    KeyboardBLE.end();  // turn off BLE Keyboard
     #ifdef FABI_BLEJOYSTICK_ENABLED
      JoystickBLE.end(); // turn off BLE Joystick if enabled
    #endif
    delay(100);  // allow some time for BLE stack to shut down properly
    cyw43_arch_deinit();  // shutdown the CYW43 WiFi/Bluetooth chip
    // Note: do not use the LED after cyw43_arch_deinit() - this would bring up the cyw43-chip again !!
  #endif

  makeTone(TONE_WAKEUP,0);    // play startup tone
  disableBatteryMeasurement();
  disable3V3();  // shut down peripherals
  delay(100);
  configureGPIOForSleep(); // configure all GPIOs for low power consumption
  dormantUntilInterrupt(wakeup_pin_map, NUMBER_OF_WAKEUP_PINS);   // enter sleepMode!
  //  <--   now sleeping!  

  makeTone(TONE_WAKEUP,0);    // play startup tone
  watchdog_reboot(0, 0, 10);  // cause a watchdog reset to wake everything up!
  while (1) { continue; }     
}

/**
 * @name userActivity
 * @brief Resets the inactivity counter upon user interaction.
 */
void userActivity() { // Call of this function can be found in line 181, buttons.cpp
  inactivityTime=0;   // reset the inactivity counter!
}

#endif
