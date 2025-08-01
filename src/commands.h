/*
      FabiWare - AsTeRICS Foundation
      For more info please visit: https://www.asterics-foundation.org

      Module: commands.h - implementation of the AT-commands, header file


   Supported AT-commands:
   (sent via serial interface, 115200 baud, using spaces between parameters.  Enter (<cr>, ASCII-code 0x0d) finishes a command)

          AT                returns "OK"
          AT ID             returns identification string (e.g. "FABI v3.0")
          AT BM <uint>      puts button into programming mode (e.g. "AT BM 2" -> next AT-command defines the new function for button 2)
                            for the FlipMouse, there are 19 buttons available (3 physical buttons, 16 virtual functions)
                            for FABI/FlipPad, there are 21 buttons available (5 physical buttons, 16 virtual functions)
                            (see buttons.h)

    USB HID commands:

          AT CL             click left mouse button
          AT CR             click right mouse button
          AT CM             click middle mouse button
          AT CD             click double with left mouse button

          AT HL             hold the left mouse button (automatic release when user action has ended)
          AT HR             hold the right mouse button (automatic release when user action has ended)
          AT HM             hold the middle mouse button (automatic release when user action has ended)

          AT RL             release the left mouse button
          AT RR             release the right mouse button
          AT RM             release the middle mouse button

          AT TL             toggle the left mouse button
          AT TR             toggle the right mouse button
          AT TM             toggle the middle mouse button

          AT WU             move mouse wheel up
          AT WD             move mouse wheel down
          AT WS <uint>      set mouse wheel stepsize (e.g. "AT WS 3" sets the wheel stepsize to 3 rows)

          AT MX <int>       move mouse in x direction (e.g. "AT MX 4" moves cursor 4 pixels to the right)
          AT MY <int>       move mouse in y direction (e.g. "AT MY -10" moves cursor 10 pixels up)

          AT J0 <int>       set joystick axis0 (stick 1 x-axis, e.g. "AT J0 512" sets the x-axis of the left stick to middle position)
          AT J1 <int>       set joystick axis1 (stick 1 y-axis, e.g. "AT J1 1023" sets the y-axis of the left stick to full down position)
          AT J2 <int>       set joystick axis2 (stick 2 x-axis, e.g. "AT J2 0" sets the x-axis of the right stick to full left position)
          AT J3 <int>       set joystick axis3 (stick 2 y-axis)
          AT J4 <int>       set joystick axis4 (stick 3 x-axis)
          AT J5 <int>       set joystick axis5 (stick 3 y-axis)
          AT JP <int>       press joystick button (e.g. "AT JP 1" presses joystick button 1, automatic release when user action has ended)
          AT JR <int>       release joystick button (e.g. "AT JR 2" releases joystick button 2)
          AT JH <int>       set joystick hat position (e.g. "AT JH 45" sets joystick hat to 45 degrees)
                            possible values are: 0, 45, 90, 135, 180, 225, 270, 315 and -1 to set center position)

          AT KW <string>    keyboard write string (e.g." AT KW Hello!" writes "Hello!")
          AT KP <string>    key press: press keys once (automatic release after all keys were pressed)
                            (e.g. "AT KP KEY_UP" presses the "Cursor-Up" key, "AT KP KEY_CTRL KEY_ALT KEY_DELETE" presses all three keys)
          AT KH <string>    key hold: hold keys (automatic release when user action is done)
                            (e.g. "AT KH KEY_A" presses the "A" key until  "AT KR KEY_A" is sent)
          AT KT <string>    key toggle: "sticky" hold keys (key will be pressed until "AT KT" command is sent again or a "AT KR" command is sent)
                            in contrast to "AT KH" a finished user action does not release the keys
                            (e.g. "AT KT KEY_A" presses the "A" key until  "AT KT KEY_A" is sent again.)

                            for a list of supported key idientifier strings see below !

          AT KR <string>    key release: releases all keys identified in the string
          AT RA             release all: releases all currently pressed keys and buttons
          AT KL <string>    change keyboard layout. No parameter prints the currently used layout. 
                            Currently supported: de_DE, en_US, es_ES, fr_FR, it_IT, sv_SE, da_DK.

    Cursor control / force sensor settings:

          AT CA           calibration of zeropoint
          AT AX <uint>    acceleration x-axis  (0-100)
          AT AY <uint>    acceleration y-axis  (0-100)
          AT DX <uint>    deadzone x-axis  (0-1000)
          AT DY <uint>    deadzone y-axis  (0-1000)
          AT MS <uint>    maximum speed  (0-100)
          AT AC <uint>    acceleration time (0-100)
          AT SB <uint>    select a sensorboard (profile-ID), adjusts signal processing parameters (0-3)

      Sip and Puff settings:

          AT TS <uint>    treshold for sip action  (0-512)
          AT TP <uint>    treshold for puff action (512-1023)
          AT SP <uint>    treshold for strong puff (512-1023)
          AT SS <uint>    treshold for strong sip (0-512)

    Housekeeping and storage commands:
    
          AT SA <string>  save slotSettings and current button modes to next free eeprom slot under given name (e.g. "AT SA mouse1")
          AT LO <string>  load button modes from eeprom slot (e.g. AT LOAD mouse1 -> loads profile named "mouse1")
          AT LA           load all slots (displays names and slotSettings of all stored slots)
          AT LI           list all saved mode names
          AT NE           next mode will be loaded (wrap around after last slot)
          AT DE <string>  delete slot of given name (deletes all stored slots if no string parameter is given)
          AT RS           resets FABI settings and restores default configuration (deletes EEPROM and restores default Slot "keys")
          AT RE           perform a reboot (SW-reset)

    Bluetooth Add-On specific commands (only supported by FlipMouse):
     
          AT BC <string>  sends parameter to external UART (mostly ESP32 Bluetooth Addon)
          AT BR <uint>    resets the ESP32 bluetooth module (connected to RP 2040 on ArduinoNanoConnect board)  // NOTE: changed for RP2040! 
          AT UG           start addon upgrade, Serial ports are transparent until ("$FIN") is received.


    Reporting and Audio feedback commands:

          AT SC <string>  change slot color: given string 0xRRGGBB                           
          AT SR           start reporting raw values (5 sensor values, starting with "VALUES:")
          AT ER           end reporting raw values
          AT AT <string>  audio transfer: start reception of a wav file of given name (if name is empty, current slot number is used)
                          (the file will be created in the local file system, existing files will be overwritten)
          AT AP <string>  audio play: start playback a wav file of given name (if name is empty, current slot number is used)
          AT AR <string>  audio remove: removes an audio file (if name is empty, current slot number is used)
          AT AL           audio list: list all available audio files
          AT AV <uint>    audio volume: audio volume (0-200 %)
          AT AB <uint>    audio buzzer mode: 0=off, 1=height, 2=height and count

    Time and threshold settings:
          AT AD <int>     time threshold for automatic dwell-click (e.g. "AT DW 700" creates a left click 700 ms after mouse movement, 0=disable)
          AT LP <int>     time threshold for long-press (e.g. "AT LP 1500" select the long-press-function of a button after 1500ms hold time, 0=disable)
          AT MP <int>     time threshold for multi-press (e.g. "AT MP 400" sets the threshold for time between multiple presses to 400 ms, 0=disable)
 
    Mode change and others:
          AT MM <uint>    mouse mode: cursor on (uint==1) or alternative functions on (uint==0)
          AT SW           switch between mouse cursor and alternative functions
          AT BT <uint>    set bluetooth mode, 1=USB only, 2=BT only, 3=both(default)
                          (e.g. AT BT 2 -> send HID commands only via BT if BT-daughter board is available)
          AT MA <string>  execute a command macro containing multiple commands (separated by semicolon)
                          example: "AT MA MX 100;MY 100;CL;"  use backslash to mask semicolon: "AT MA KW \;;CL;" writes a semicolon and then clicks left
          AT WA <uint>    wait (given in milliseconds, useful for macro commands)
          AT NC           no command (idle operation)

    Infrared-specific commands:

          AT IR <string>  record new infrared code and store it under given name (e.g. "AT IR vol_up")
          AT IP <string>  play infrared code with given name (e.g. "AT IP vol_up")
          AT IH <string>  play and hold the infrared code with given name
          AT IS           stop currently playing infrared code
          AT IC <string>  clear infrared code with given name (e.g. "AT IC vol_up")
          AT IW           wipe infrared memory (clear all codes)
          AT IL           lists all stored infrared command names
          AT IT <uint>    set code timeout value for IR Recording (e.g. "AT IT 10" sets 10 milliseconds timeout)

   supported key identifiers for key commands (AT KP/KH/KR/KT):

    KEY_A   KEY_B   KEY_C   KEY_D    KEY_E   KEY_F   KEY_G   KEY_H   KEY_I   KEY_J    KEY_K    KEY_L
    KEY_M   KEY_N   KEY_O   KEY_P    KEY_Q   KEY_R   KEY_S   KEY_T   KEY_U   KEY_V    KEY_W    KEY_X
    KEY_Y   KEY_Z   KEY_1   KEY_2    KEY_3   KEY_4   KEY_5   KEY_6   KEY_7   KEY_8    KEY_9    KEY_0
    KEY_F1  KEY_F2  KEY_F3  KEY_F4   KEY_F5  KEY_F6  KEY_F7  KEY_F8  KEY_F9  KEY_F10  KEY_F11  KEY_F12

    KEY_RIGHT   KEY_LEFT       KEY_DOWN        KEY_UP          KEY_ENTER    KEY_ESC   KEY_BACKSPACE   KEY_TAB
    KEY_HOME    KEY_PAGE_UP    KEY_PAGE_DOWN   KEY_DELETE      KEY_INSERT   KEY_END   KEY_NUM_LOCK    KEY_SCROLL_LOCK
    KEY_SPACE   KEY_CAPS_LOCK  KEY_PAUSE       
    KEY_SHIFT   KEY_CTRL       KEY_ALT         KEY_RIGHT_ALT   KEY_GUI      KEY_RIGHT_GUI
    KEY_SLASH   KEY_BACKSLASH  KEY_LEFT_BRACE  KEY_RIGHT_BRACE KEY_QUOTE    KEY_TILDE    
    KEY_MINUS   KEY_SEMICOLON  KEY_EQUAL       KEY_COMMA       KEY_PERIOD   KEY_MENU
    
    KEYPAD_1        KEYPAD_2       KEYPAD_3        KEYPAD_4         KEYPAD_5        KEYPAD_6   
    KEYPAD_7        KEYPAD_8       KEYPAD_9        KEYPAD_0         KEYPAD_SLASH    KEYPAD_MINUS    KEYPAD_PLUS
    KEYPAD_ENTER    KEYPAD_PERIOD  KEYPAD_ASTERIX


   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; See the GNU General Public License:
   http://www.gnu.org/licenses/gpl-3.0.en.html

*/

#ifndef _COMMANDS_H_
#define _COMMANDS_H_


/**
   atCommands
   enumeration of AT command identifiers
*/
enum atCommands {
  CMD_ID, CMD_BM, CMD_CL, CMD_CR, CMD_CM, CMD_CD, CMD_PL, CMD_PR, CMD_PM, CMD_HL, CMD_HR, CMD_HM,
  CMD_RL, CMD_RR, CMD_RM, CMD_TL, CMD_TR, CMD_TM, CMD_WU, CMD_WD, CMD_WS, CMD_MX, CMD_MY, 
  CMD_J0, CMD_J1, CMD_J2, CMD_J3, CMD_J4, CMD_J5, CMD_JP, CMD_JR, CMD_JH,  
  CMD_KW, CMD_KP, CMD_KH, CMD_KT, CMD_KR, CMD_RA, CMD_KL, 
  CMD_SA, CMD_LO, CMD_LA, CMD_LI, CMD_NE, CMD_DE, CMD_RS, CMD_RE, CMD_NC, 
  CMD_BT, CMD_SC, CMD_SR, CMD_ER, CMD_CA, CMD_MA, CMD_WA, CMD_TS, CMD_TP, CMD_SP, CMD_SS, CMD_IR, 
  CMD_IP, CMD_IH, CMD_IS, CMD_IC, CMD_IW, CMD_IL, CMD_IT, CMD_MM, CMD_SW, CMD_AX, CMD_AY, 
  CMD_DX, CMD_DY, CMD_MS, CMD_AC, CMD_RO, CMD_SB, CMD_AT, CMD_AP, CMD_AR, CMD_AL, CMD_AV, CMD_AB,
  CMD_AD, CMD_LP, CMD_MP, 
#ifdef FLIPMOUSE
  CMD_BC, CMD_BR, CMD_UG,
#endif
  NUM_COMMANDS
};

/**
   atCommandType struct
   holds AT command string and paramter type identifiers
*/
struct atCommandType {
    char atCmd[3];
    uint8_t  partype;
};

/**
   extern declaration of static variables
   which shall be accessed from other modules
*/
extern const struct atCommandType atCommands[];

/**
   @name performCommand (called from parser.cpp)
   @brief performs a particular action/AT command
   @param cmd AT command identifier
   @param par1 numeric parameter for the command
   @param keystring string parameter for the command
   @param periodicMouseMovement if true, mouse will continue moving after action, otherwise only one movement
   @return none
*/
void performCommand (uint8_t cmd, int16_t par1, char * keystring, int8_t periodicMouseMovement);

#endif
