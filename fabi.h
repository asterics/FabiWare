
/* 
     Assistive Button Interface (FABI) - AsTeRICS Foundation - http://www.asterics-foundation.org
      allows control of HID functions via switches and/or AT-commands  
   

   requirements:  Arduino (Pro) Micro or Teensy2.0++ with Teensyduino AddOn for Arduino IDE 
                  (Teensy USB type set to USB composite device: Serial + Keyboard + Mouse + Joystick)
        sensors:  up to 9 momentary switches connected to GPIO pins
                  optional pressure sensor connected to ADC pin A0 for sip/puff actions
       
   
   for a list of supported AT commands, see commands.h / commands.cpp
   
 */


#ifndef _FABI_H_
#define _FABI_H_

#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include "commands.h"


#define VERSION_STRING "FABI v2.3"

//#define TEENSY            //  if a Teensy2.0++ controller is used
#define ARDUINO_PRO_MICRO   //  if Arduino Leonardo or Arduino (Pro) Micro is used 

#ifdef  ARDUINO_PRO_MICRO
  #include <Mouse.h>
  #include <Keyboard.h>
  #define EEPROM_SIZE        1023     // maximum size of EEPROM storage for Arduino (Pro) Micro
#else
  #define EEPROM_SIZE        4096     // maximum size of EEPROM storage for Teensy2.0++
#endif

#define NUMBER_OF_BUTTONS  11         // number of connected or virtual switches
#define NUMBER_OF_PHYSICAL_BUTTONS 9  // number of connected switches
#define NUMBER_OF_LEDS     3          // number of connected leds


#define MOUSE_ACCELDELAY   50         // steps to reach mouse move speed

#define MAX_SLOTNAME_LEN      10      // maximum lenght for a slotname
#define KEYSTRING_BUFFER_LEN 500      // maximum lenght for all string parameters of a slot 
#define MAX_CMDLEN           200      // maximum lenght of a single AT command

#define PARTYPE_NONE   0
#define PARTYPE_UINT   1
#define PARTYPE_INT    2
#define PARTYPE_STRING 3

#define REPORT_NONE  0  
#define REPORT_ONE_SLOT  1
#define REPORT_ALL_SLOTS 2

#define DEFAULT_WAIT_TIME            5   // wait time for one loop interation in milliseconds
#define DEFAULT_CLICK_TIME           8   // time for mouse click (loop iterations from press to release)
#define DOUBLECLICK_MULTIPLIER       5   // CLICK_TIME factor for double clicks
#define DEFAULT_WHEEL_STEPSIZE       3   // stepsize for scroll wheel
#define DEFAULT_SIP_THRESHOLD        0   // sip action disabled per default
#define DEFAULT_PUFF_THRESHOLD    1023   // puff action disabled per default
#define DEFAULT_ANTITREMOR_PRESS     5   // debouncing interval for button-press
#define DEFAULT_ANTITREMOR_RELEASE   2   // debouncing interval for button-release
#define DEFAULT_ANTITREMOR_IDLE      1   // debouncing interval for button idle time
#define DEFAULT_TRESHOLD_TIME     5000   // treshold time for short / long press (5000: disable long press)
#define BUTTON_PRESSED  1
#define BUTTON_RELEASED 0
#define BUTTONSTATE_NOT_PRESSED   0
#define BUTTONSTATE_SHORT_PRESSED 1
#define BUTTONSTATE_LONG_PRESSED  2
#define BUTTONSTATE_IDLE          3

struct settingsType {
  char slotname[MAX_SLOTNAME_LEN];     // EEPROM slotname maximum length
  uint8_t  ws;     // wheel stepsize  
  uint16_t tt;     // threshold time for longpress 
  uint16_t ts;     // threshold sip
  uint16_t tp;     // threshold puff 
  uint16_t ap;     // antitremor press time 
  uint16_t ar;     // antitremor release time 
  uint16_t ai;     // antitremor idle time 
};

struct atCommandType {              // holds settings for a button function 
  char atCmd[3];
  uint8_t  partype;   // type of parameter: int, uint or string
};

struct buttonType {                 // holds settings for a button function 
  int mode;          // buttonmode index (AT command type)
  int value;         // value (if numeric parameter) - note that strings are stored in keystringBuffer[]
};

struct buttonDebouncerType {       // holds working data for button debouncing and longpress detection 
  uint16_t pressCount;
  uint16_t releaseCount;
  uint16_t idleCount;
  uint8_t  pressState;
} ; 

extern uint8_t DebugOutput;
extern uint8_t actSlot;
extern uint8_t reportSlotParameters;
extern uint8_t reportRawValues;
extern struct settingsType settings;
extern int EmptySlotAddress;
extern struct buttonType buttons[NUMBER_OF_BUTTONS];
extern struct buttonDebouncerType buttonDebouncers[NUMBER_OF_BUTTONS];
extern const struct atCommandType atCommands[];
extern char cmdstring[MAX_CMDLEN];                 // buffer for incoming AT commands
extern char keystringBuffer[KEYSTRING_BUFFER_LEN]; // buffer for all string parameters for the buttons of a slot
extern uint16_t freeEEPROMbytes;
extern const int usToDE[];

extern uint8_t leftMouseButton;
extern uint8_t middleMouseButton;
extern uint8_t rightMouseButton;
extern uint8_t leftClickRunning;
extern uint8_t rightClickRunning;
extern uint8_t middleClickRunning;
extern uint8_t doubleClickRunning;
extern char * writeKeystring;
extern int8_t moveX;       
extern int8_t moveY;

char * getKeystring (uint8_t button);
void setKeystring (uint8_t button, char * text);
void printKeystrings ();
uint16_t  keystringMemUsage(uint8_t button);
void parseCommand (char * cmdstr);
void performCommand (uint8_t cmd, int16_t par1, char * keystring, int8_t periodicMouseMovement);
void saveToEEPROM(char * slotname);
void readFromEEPROM(char * slotname);
void deleteSlots();
void listSlots();
void printCurrentSlot();


void BlinkLed();
int freeRam ();
void parseByte (int newByte);

int getKeycode(char*);
void sendToKeyboard( char * );
void pressSingleKeys(char* text); // presses individual keys
void releaseSingleKeys(char* text);  // releases individual keys
void release_all();            // releases all previously pressed keys and buttons

#define strcpy_FM   strcpy_PF
#define strcmp_FM   strcmp_PF
typedef uint_farptr_t uint_farptr_t_FM;

#endif
