/*
  FabiWare - AsTeRICS Foundation
  For more info please visit: https://www.asterics-foundation.org

  Module: buttons.cpp - implementation of the button handling

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; See the GNU General Public License:
  http://www.gnu.org/licenses/gpl-3.0.en.html

*/

#include "FabiWare.h"
#include "buttons.h"
#include "infrared.h"
#include "keys.h"
#include "triggers.h"

struct slotButtonSettings buttons [NUMBER_OF_BUTTONS];   // array for all buttons - type definition see FabiWare.h
char * buttonKeystrings[NUMBER_OF_BUTTONS];              // pointers to keystring parameters
char keystringBuffer[MAX_KEYSTRINGBUFFER_LEN]={0};       // storage for keystring parameters for all buttons
struct buttonDebouncerType buttonDebouncers [NUMBER_OF_BUTTONS];   // array for all buttonsDebouncers - type definition see fabi.h
uint32_t buttonStates = 0;  // current button states for reporting raw values (AT SR)

static void executeTriggerAction(int8_t trigIdx)
{
  if (trigIdx < 0) return;
  performCommand(triggerEntries[trigIdx].mode, triggerEntries[trigIdx].value,
                 getTriggerKeystring(trigIdx), 1);
}

static void releaseButtonHoldAction(int buttonIndex)
{
  switch (buttons[buttonIndex].mode) {
    case CMD_MX: currentState.autoMoveX = 0; break;
    case CMD_MY: currentState.autoMoveY = 0; break;
    case CMD_PL:
    case CMD_HL: mouseRelease(MOUSE_LEFT); break;
    case CMD_PR:
    case CMD_HR: mouseRelease(MOUSE_RIGHT); break;
    case CMD_PM:
    case CMD_HM: mouseRelease(MOUSE_MIDDLE); break;
    case CMD_JP: joystickButton(buttons[buttonIndex].value, 0); break;
    case CMD_J0: joystickAxis(0,512); break;
    case CMD_J1: joystickAxis(1,512); break;
    case CMD_J2: joystickAxis(2,512); break;
    case CMD_J3: joystickAxis(3,512); break;
    case CMD_J4: joystickAxis(4,512); break;
    case CMD_J5: joystickAxis(5,512); break;
    case CMD_JH: joystickHat(-1); break;
    case CMD_KH: releaseKeys(buttonKeystrings[buttonIndex]); break;
    case CMD_IH: stop_IR_command(); break;
    default: break;
  }
}

static void executeNormalButtonAction(int buttonIndex)
{
  performCommand(buttons[buttonIndex].mode, buttons[buttonIndex].value,
                 buttonKeystrings[buttonIndex], 1);

  // If the button is already released when a deferred single action is resolved,
  // immediately release hold-style actions to avoid sticky states.
  if (inHoldMode(buttonIndex) && ((buttonStates & (1UL << buttonIndex)) == 0)) {
    releaseButtonHoldAction(buttonIndex);
  }
}

void initButtonKeystrings()
{
  slotSettings.keystringBufferLen=0;
  for (int i=0;i<NUMBER_OF_BUTTONS;i++) {
    buttonKeystrings[i]=keystringBuffer + slotSettings.keystringBufferLen;
    while (keystringBuffer[slotSettings.keystringBufferLen++]) ; 
  }
#ifdef DEBUG_OUTPUT_FULL
  DEBUG_OUT.print("Init ButtonKeystrings, bufferlen ="); 
  DEBUG_OUT.println(slotSettings.keystringBufferLen);
#endif
}

char * getButtonKeystring(int num)
{
  char * str = keystringBuffer;
  for (int i=0;i<num;i++) {
    if(*str) while(*str++);
    else str++;  
  }
  return(str);
}


void printKeystrings()
{
  char * x = keystringBuffer;
  for (int i=0;i<NUMBER_OF_BUTTONS;i++) {
    if (*x) {
      Serial.print("Keystring ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.println(x);
      while (*x++);
    } else x++;
  }
}
uint16_t setButtonKeystring(uint8_t buttonIndex, char const * newKeystring)
{
  char * keystringAddress = getButtonKeystring(buttonIndex);
  
  int oldKeyStringLen = strlen (keystringAddress);
  char * sourceAddress = keystringAddress + oldKeyStringLen +1;
  
  if (slotSettings.keystringBufferLen - oldKeyStringLen + strlen(newKeystring) >= MAX_KEYSTRINGBUFFER_LEN - 1) 
    return (0);   // new keystring does not fit into buffer !

  uint16_t bytesToMove = keystringBuffer + slotSettings.keystringBufferLen - sourceAddress;
  int delta = strlen(newKeystring) - oldKeyStringLen;  // if positive: expand keystringBuffer!
  char * targetAddress = sourceAddress + delta;
  if (delta) {
     memmove(targetAddress, sourceAddress, bytesToMove);    
  }
    
  strcpy (keystringAddress, newKeystring);  // store the new keystring!
  
  //update ALL keystring pointers, because we might have moved some of them
  char * x = keystringBuffer;
  for (int i=0;i<NUMBER_OF_BUTTONS;i++) {
    if (*x) {
      buttonKeystrings[i] = x;
      while (*x++);
    } else x++;
  }

  slotSettings.keystringBufferLen += delta;  // update buffer length
  
#ifdef DEBUG_OUTPUT_FULL
  printKeystrings();
  DEBUG_OUT.print("bytes left:");DEBUG_OUT.println(MAX_KEYSTRINGBUFFER_LEN-slotSettings.keystringBufferLen);
#endif
  return (MAX_KEYSTRINGBUFFER_LEN - slotSettings.keystringBufferLen);
}


void initButtons() {

  initButtonKeystrings();
  initTriggers();    // clear all trigger entries for this slot

  // set default functions
  for (int i=0;i<NUMBER_OF_BUTTONS;i++) {
    buttons[i].value = 0;
    buttons[i].mode = CMD_NC;  // no command
  }
  
  #ifdef FLIPMOUSE
    buttons[0].mode = CMD_NE; // button1: switch to next slot
    buttons[UP_BUTTON].mode = CMD_KH; setButtonKeystring(UP_BUTTON, "KEY_UP ");
    buttons[DOWN_BUTTON].mode = CMD_KH; setButtonKeystring(DOWN_BUTTON, "KEY_DOWN ");
    buttons[LEFT_BUTTON].mode = CMD_KH; setButtonKeystring(LEFT_BUTTON, "KEY_LEFT "); 
    buttons[RIGHT_BUTTON].mode = CMD_KH; setButtonKeystring(RIGHT_BUTTON, "KEY_RIGHT ");
    buttons[SIP_BUTTON].mode = CMD_HL; // sip: hold left mouse button
    buttons[PUFF_BUTTON].mode = CMD_CR; // puff: click right
    buttons[STRONGPUFF_BUTTON].mode = CMD_CA; // strong puff: calibrate
  #else
    buttons[0].mode = CMD_KH; setButtonKeystring(0, "KEY_SPACE ");
    buttons[1].mode = CMD_KH; setButtonKeystring(1, "KEY_ENTER ");
    buttons[2].mode = CMD_CL;
  #endif
}


void handlePress (int buttonIndex)   // a button was pressed
{
  #ifdef DEBUG_OUTPUT_FULL
    Serial.print("P: "); Serial.println(buttonIndex);
    Serial.print(buttons[buttonIndex].mode); Serial.print(" "); Serial.print(buttons[buttonIndex].value); Serial.print(" "); Serial.println(buttonKeystrings[buttonIndex]);
  #endif
  buttonStates |= (1<<buttonIndex); //save for reporting
  buttonDebouncers[buttonIndex].longPressed = 0;   // reset long-press flag for this press
  buttonDebouncers[buttonIndex].timestamp = millis(); // start measuring hold time for long-press
  // Hierarchical handling for non-hold actions:
  // if double/triple are configured, delay firing until timeout so only the
  // most specific matching action (single/double/triple) is executed.
  int8_t trigDouble = findTrigger((uint8_t)buttonIndex, TRIGGER_TYPE_DOUBLE);
  int8_t trigTriple = findTrigger((uint8_t)buttonIndex, TRIGGER_TYPE_TRIPLE);
  if (globalSettings.thresholdMultiPress > 0 &&
      (trigDouble >= 0 || trigTriple >= 0)) {
    uint32_t now = millis();
    if (buttonDebouncers[buttonIndex].multiPending &&
        ((uint32_t)(now - buttonDebouncers[buttonIndex].lastReleaseTime) <= globalSettings.thresholdMultiPress)) {
      if (buttonDebouncers[buttonIndex].pressCount < 3)
        buttonDebouncers[buttonIndex].pressCount++;
    } else {
      buttonDebouncers[buttonIndex].pressCount = 1;
    }
    buttonDebouncers[buttonIndex].multiPending = 1;

    // If no more specific trigger can follow, execute immediately.
    if (buttonDebouncers[buttonIndex].pressCount >= 3 && trigTriple >= 0) {
      releaseButtonHoldAction(buttonIndex);
      executeTriggerAction(trigTriple);
      buttonDebouncers[buttonIndex].pressCount = 0;
      buttonDebouncers[buttonIndex].multiPending = 0;
    }
    else if (buttonDebouncers[buttonIndex].pressCount >= 2 && trigDouble >= 0 && trigTriple < 0) {
      releaseButtonHoldAction(buttonIndex);
      executeTriggerAction(trigDouble);
      buttonDebouncers[buttonIndex].pressCount = 0;
      buttonDebouncers[buttonIndex].multiPending = 0;
    }
    return;
  }

  // No hierarchical multi-press case: execute normal action immediately.
  executeNormalButtonAction(buttonIndex);
}

void handleRelease (int buttonIndex)    // a button was released: deal with "sticky"-functions
{
  #ifdef DEBUG_OUTPUT_FULL
    Serial.print("R: "); Serial.println(buttonIndex);
  #endif
  buttonStates &= ~(1<<buttonIndex); //save for reporting
  buttonDebouncers[buttonIndex].lastReleaseTime = millis();  // for multi-press detection
  switch (buttons[buttonIndex].mode) {
    // release mouse actions
    case CMD_MX: currentState.autoMoveX = 0; break;
    case CMD_MY: currentState.autoMoveY = 0; break;
    case CMD_PL:
    case CMD_HL:
      mouseRelease(MOUSE_LEFT);
      break;
    case CMD_PR:
    case CMD_HR:
      mouseRelease(MOUSE_RIGHT);
      break; 
    case CMD_PM:
    case CMD_HM:
      mouseRelease(MOUSE_MIDDLE);
      break;
    // release gamepad actions
    case CMD_JP: joystickButton(buttons[buttonIndex].value, 0); break;
    case CMD_J0: joystickAxis(0,512); break;
    case CMD_J1: joystickAxis(1,512); break;
    case CMD_J2: joystickAxis(2,512); break;
    case CMD_J3: joystickAxis(3,512); break;
    case CMD_J4: joystickAxis(4,512); break;
    case CMD_J5: joystickAxis(5,512); break;
    case CMD_JH: joystickHat(-1); break;
    // release keyboard actions
    case CMD_KH: releaseKeys(buttonKeystrings[buttonIndex]); break;
    // release infrared actions
    case CMD_IH:
      stop_IR_command();
      break;
  }
}

void processLongPressTriggers()
{
  if (globalSettings.thresholdLongPress == 0) return;

  uint32_t now = millis();
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    if ((buttonStates & (1UL << i)) == 0) continue;   // currently not pressed
    if (buttonDebouncers[i].longPressed) continue;    // already fired during this press

    uint32_t heldFor = now - buttonDebouncers[i].timestamp;
    if ((uint32_t)heldFor >= globalSettings.thresholdLongPress) {
      int8_t trigIdx = findTrigger((uint8_t)i, TRIGGER_TYPE_LONG);
      buttonDebouncers[i].longPressed = 1;            // fire at most once per press
      if (trigIdx >= 0) {
        // A long-press trigger supersedes any pending single/double/triple resolution
        // from this press sequence.
        buttonDebouncers[i].pressCount = 0;
        buttonDebouncers[i].multiPending = 0;
        releaseButtonHoldAction(i);
        executeTriggerAction(trigIdx);
      }
    }
  }
}

void processMultiPressTriggers()
{
  if (globalSettings.thresholdMultiPress == 0) return;

  uint32_t now = millis();
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    if (!buttonDebouncers[i].multiPending) continue;

    // If button is still held and no follow-up press can occur, resolve delayed
    // hold-style single action once MP timeout elapsed.
    if ((buttonStates & (1UL << i)) != 0) {
      if (buttonDebouncers[i].pressCount == 1 && inHoldMode(i) &&
          ((uint32_t)(now - buttonDebouncers[i].timestamp) > globalSettings.thresholdMultiPress)) {
        executeNormalButtonAction(i);
        buttonDebouncers[i].pressCount = 0;
        buttonDebouncers[i].multiPending = 0;
      }
      continue;
    }

    // lastReleaseTime==0 means no release has happened yet for this sequence.
    if (buttonDebouncers[i].lastReleaseTime == 0) continue;

    if ((uint32_t)(now - buttonDebouncers[i].lastReleaseTime) <= globalSettings.thresholdMultiPress)
      continue; // still within multi-press window

    int8_t trigDouble = findTrigger((uint8_t)i, TRIGGER_TYPE_DOUBLE);
    int8_t trigTriple = findTrigger((uint8_t)i, TRIGGER_TYPE_TRIPLE);

    if (buttonDebouncers[i].pressCount >= 3 && trigTriple >= 0) {
      releaseButtonHoldAction(i);
      executeTriggerAction(trigTriple);
    }
    else if (buttonDebouncers[i].pressCount >= 2 && trigDouble >= 0) {
      releaseButtonHoldAction(i);
      executeTriggerAction(trigDouble);
    }
    else if (buttonDebouncers[i].pressCount >= 1) {
      // fall back to the normal single action after timeout
      executeNormalButtonAction(i);
    }

    buttonDebouncers[i].pressCount = 0;
    buttonDebouncers[i].multiPending = 0;
  }
}


uint8_t handleButton(int i, uint8_t state)    // button debouncing and press detection
{
  if ( buttonDebouncers[i].bounceState == state) {
    if (buttonDebouncers[i].bounceCount < DEFAULT_DEBOUNCING_TIME) {
      buttonDebouncers[i].bounceCount++;
      if (buttonDebouncers[i].bounceCount == DEFAULT_DEBOUNCING_TIME) {

        if (state != buttonDebouncers[i].stableState)  // entering stable state
        {
          buttonDebouncers[i].stableState = state;
          if (state == 1) {      // new stable state: pressed !
            //if (inHoldMode(i))
            buttonStates |= (1<<i); //save for reporting
            handlePress(i);
            #ifdef RP2350
              userActivity(); // keep system from going into dormant mode (see lpwFuncs.h)
            #endif
          }
          else {   // new stable state: released !
            // if (!inHoldMode(i))
            //   handlePress(i);
            buttonStates &= ~(1<<i); //save for reporting
            handleRelease(i);
            return (1);        // indicate that button action has been performed !
          }
        }
      }
    }
    else {  // in stable state
      // --- long-press trigger detection (additive, physical buttons only) ---
      if (buttonDebouncers[i].stableState == 1 &&   // button is currently held
          globalSettings.thresholdLongPress > 0  &&
          !buttonDebouncers[i].longPressed) {
        if ((uint32_t)(millis() - buttonDebouncers[i].timestamp) >= globalSettings.thresholdLongPress) {
          int8_t trigIdx = findTrigger((uint8_t)i, TRIGGER_TYPE_LONG);
          if (trigIdx >= 0) {
            buttonDebouncers[i].longPressed = 1;   // fire only once per press
            performCommand(triggerEntries[trigIdx].mode, triggerEntries[trigIdx].value,
                           getTriggerKeystring(trigIdx), 1);
          } else {
            buttonDebouncers[i].longPressed = 1;   // no trigger defined: suppress repeated checks
          }
        }
      }
    }
  }
  else {
    buttonDebouncers[i].bounceState = state;
    buttonDebouncers[i].bounceCount = 0;
  }
  return (0);
}

uint8_t inHoldMode (int i)
{
  if ((buttons[i].mode == CMD_PL) ||
      (buttons[i].mode == CMD_PR) ||
      (buttons[i].mode == CMD_PM) ||
      (buttons[i].mode == CMD_HL) ||
      (buttons[i].mode == CMD_HR) ||
      (buttons[i].mode == CMD_HM) ||
      (buttons[i].mode == CMD_JP) ||
      (buttons[i].mode == CMD_J0) ||
      (buttons[i].mode == CMD_J1) ||
      (buttons[i].mode == CMD_J2) ||
      (buttons[i].mode == CMD_J3) ||
      (buttons[i].mode == CMD_J4) ||
      (buttons[i].mode == CMD_J5) ||
      (buttons[i].mode == CMD_JH) ||
      (buttons[i].mode == CMD_MX) ||
      (buttons[i].mode == CMD_MY) ||
      (buttons[i].mode == CMD_KH) ||
      (buttons[i].mode == CMD_IH))
    return (1);
  else return (0);
}

void initDebouncers()
{
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) // initialize button array
  {
    buttonDebouncers[i].bounceState   = 0;
    buttonDebouncers[i].stableState   = 0;
    buttonDebouncers[i].bounceCount   = 0;
    buttonDebouncers[i].longPressed   = 0;
    buttonDebouncers[i].pressCount    = 0;
    buttonDebouncers[i].lastReleaseTime = 0;
    buttonDebouncers[i].multiPending  = 0;
  }
}
