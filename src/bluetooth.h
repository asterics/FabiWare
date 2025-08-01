/*
  FabiWare - AsTeRICS Foundation
  For more info please visit: https://www.asterics-foundation.org

  Module: bluetooth.h - using external Bluetooth addon for mouse/keyboard control
  Copyright (c) Benjamin Aigner
  For a description of the supported commands see: commands.h

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; See the GNU General Public License:
  http://www.gnu.org/licenses/gpl-3.0.en.html
  
*/


#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include "FabiWare.h"

/**
    @name isBluetoothConnected
    @param none
    @return true, if the BT module is connected (paired) false if not

    This method returns true, if the BT module is currently paired to a host device
    False will be returned otherwise
*/
bool isBluetoothConnected();
// void setup_bt_event_handling(void);


#ifdef FLIPMOUSE    // from here, the code is only relevant for the FlipMouse with Arduino Nano 2040 Connect + ESP32 (using our BLE Hid implementation)

  //RX/TX3 are used to communicate with an addon board (mounted on AUX header)
  #define Serial_AUX Serial2
  
  /** BT module upgrade: inactive/idle */
  #define BTMODULE_UPGRADE_IDLE 0
  /** BT module upgrade: starting (waiting for ESP32 to switch into OTA mode) */
  #define BTMODULE_UPGRADE_START 1
  /** BT module upgrade: running (data is transmitted) */
  #define BTMODULE_UPGRADE_RUNNING 2
  
  /**
     @name mouseBT
     @param x relative movement x axis
     @param y relative movement y axis
     @param scroll relative scroll actions
     @return
  
     this method sends a mouse command via the Bluetooth module.
     Mouse movements, buttons and scroll wheel.
     The limit for the movement is +127/-127
  */
  void mouseBT(int x, int y, uint8_t scroll);
  
  
  /**
     @name mouseBTPress
     @param mousebutton uint8_t contains all buttons to be pressed (masked): (1<<0) left; (1<<1) right; (1<<2) middle
     @return none
  */
  void mouseBTPress(uint8_t mousebutton);
  
  /**
     @name mouseBTRelease
     @param mousebutton uint8_t contains all buttons to be release (masked): (1<<0) left; (1<<1) right; (1<<2) middle
     @return none
  */
  void mouseBTRelease(uint8_t mousebutton);
  
  
  /**
     @name isMouseBTPressed
     @param mousebutton buttons which should be polled
     @return boolan
  */
  boolean isMouseBTPressed(uint8_t mousebutton);
  
  /**
     @name initBluetooth
     @param none
     @return none
  
     Initialize the Bluetooth module on the external serial port.
     If the module returns a valid version string, BT communication is
     enabled (bt_enable is set to 1).
  
     @see bt_enable
  
  */
  void initBluetooth();
  
  
  /**
     @name detectBTResponse
     @param int c: incoming character from BT module
     @return outgoming character (value for c is propagated)
  
     detects certain replies from the BT module (eg. if a paired connection was returned after sendind $GC)
  */
  int detectBTResponse (int c);
  
  /**
     @name updateBTConnectionState
     @return none
  
     periodically polls the BT modue for connections
  */
  void updateBTConnectionState ();
  
  
  /**
     @name setBTName
     @param char * BTName: module name for BT-advertising
     @return none
  
     sets the BT module name for advertising/pairing.
  
  */
  void setBTName(char * BTName);
  
  
  
  /**
     @name unpairAllBT
     @return none
  
     forget all paired devices
  
  */
  void unpairAllBT();
  
  
  /**
     @name keyboardBTPrint
     @param char* writeString string to typed by the Bluetooth HID keyboard
     @return none
  
     This method prints out an ASCII string (no modifiers available!!!) via the
     Bluetooth module
  
     @todo We should use the keyboard maps from ESP32, can store all of them. But how to handle any multibyte strings?
  */
  void keyboardBTPrint(char * writeString);
  
  /**
     @name keyboardBTReleaseAll
     @param none
     @return none
  
     Release all previous pressed keyboard keys
  */
  void keyboardBTReleaseAll();
  
  /**
     @name keyboardBTPress
     @param int k Key to be pressed
     @return none
  
     Press a key, value is the same as in Keyboard.press().
     Because the Keyboard library does not export the raw keycodes or
     the full report, we copy the code of the Keyboard library to here.
  */
  void keyboardBTPress(int k);
  
  /**
     @name keyboardBTRelease
     @param int k Key to be released
     @return none
  
     Release a key, value is the same as in Keyboard.release().
     Because the Keyboard library does not export the raw keycodes or
     the full report, we copy the code of the Keyboard library to here.
  */
  void keyboardBTRelease(int k);
  
  /**
     @name isBluetoothAvailable
     @param none
     @return true, if the BT module is available, false if not
  
     This method returns true, if the BT module is available and delivered
     a valid version string
     False will be returned otherwise
  */
  bool isBluetoothAvailable();
  

  /**
     @name startBTPairing
     @param none
     @return none
     @note Not implemented
  */
  bool startBTPairing();
  
  
  /**
     @name performAddonUpgrade
     @param none
     @return none
  
     handle states and data transfer for BT-Addon firmware update
  */
  void performAddonUpgrade();
  
  
  /**
     @name resetBTModule
     @param downloadMode if true, ESP32 is put in FW download mode
     @return none
  
     resets the ESP32 connected to the RP2020 on the ArduinoNanoConnect board
  */
  void resetBTModule (int downloadMode);
  
  
  /**
     @name joystickBTAxis
     @param int axis       select axis (0-5)
     @param int value      new value for axis, 0-1023
     @return none
 
     Updates axis on the Joystick report for the BT firmware. Updated report is sent.
     @note Parameter range for axis is 0-1023, but we only have int8_t ranges, so it is mapped.
  */
  void joystickBTAxis(int axis, int value);
  
  
  /**
     @name joystickBTButton
     @param uint8_t nr    button number (1-32)
     @param int     val   state for button, 0 released; != 0 pressed
  
     Update button field of the BT joystick report & sends it.
  */
  void joystickBTButton(uint8_t nr, int val);
  
  
  /**
     @name joystickBtHat
     @param int     val   Hat position, 0-360 or -1
  
     Update BT joystick hat: 0-360 for position (mapped to 8 positions); -1 is rest position
     The updated report is sent.
  */
  void joystickBTHat(int val);
 #endif
#endif
  
