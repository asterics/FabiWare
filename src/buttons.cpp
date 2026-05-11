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
static uint8_t  sequenceProgress[MAX_TRIGGER_COUNT];
static uint32_t sequenceLastEvent[MAX_TRIGGER_COUNT];
static uint8_t  sequenceTermPressCount[MAX_TRIGGER_COUNT];
static uint32_t sequenceTermPressTime[MAX_TRIGGER_COUNT];
static uint8_t  normalActionStarted[NUMBER_OF_BUTTONS];
static uint8_t  pendingDoubleEventEmitted[NUMBER_OF_BUTTONS];
static int8_t   pendingLongTrigIdx[NUMBER_OF_BUTTONS];       // deferred long-press tier awaiting release; -1 = none
static uint32_t pendingLongEmittedDuration[NUMBER_OF_BUTTONS]; // last duration at which LONG event was emitted for composite

static void releaseButtonHoldAction(int buttonIndex);

#ifdef DEBUG_OUTPUT_FULL
static void debugPrintTriggerTerm(const struct TriggerTerm *term)
{
  DEBUG_OUT.print(triggerTypeName(term->triggerType));
  DEBUG_OUT.print("(");
  DEBUG_OUT.print(buttonIndexName(term->buttonIndex));
  DEBUG_OUT.print(")");
}
#endif

static void executeTriggerAction(int8_t trigIdx)
{
  if (trigIdx < 0) return;
  performCommand(triggerEntries[trigIdx].mode, triggerEntries[trigIdx].value,
                 getTriggerKeystring(trigIdx), 1);
}

static void releaseSequenceHoldActions(int8_t trigIdx)
{
  if (trigIdx < 0 || trigIdx >= MAX_TRIGGER_COUNT) return;
  for (uint8_t i = 0; i < triggerEntries[trigIdx].termCount; i++) {
    releaseButtonHoldAction(triggerEntries[trigIdx].terms[i].buttonIndex);
  }
}

static bool emitTriggerEvent(uint8_t buttonIndex, uint8_t triggerType, uint32_t eventTimestamp)
{
  uint32_t now = eventTimestamp;
  bool anyCompleted = false;

#ifdef DEBUG_OUTPUT_FULL
  DEBUG_OUT.print("TG EVT: ");
  DEBUG_OUT.print(triggerTypeName(triggerType));
  DEBUG_OUT.print("(");
  DEBUG_OUT.print(buttonIndexName(buttonIndex));
  DEBUG_OUT.print(") t=");
  DEBUG_OUT.println(now);
#endif

  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount < 2) {
      sequenceProgress[i] = 0;
      sequenceTermPressCount[i] = 0;
      continue;
    }

    if (globalSettings.thresholdMultiPress == 0) {
      sequenceProgress[i] = 0;
      sequenceTermPressCount[i] = 0;
      continue;
    }

    if (sequenceProgress[i] > 0 &&
        triggerEntries[i].terms[sequenceProgress[i]].triggerType != TRIGGER_TYPE_LONG &&
        (uint32_t)(now - sequenceLastEvent[i]) > globalSettings.thresholdMultiPress) {
#ifdef DEBUG_OUTPUT_FULL
      DEBUG_OUT.print("TG SEQ reset(timeout): #");
      DEBUG_OUT.print(i);
      DEBUG_OUT.print(" gap=");
      DEBUG_OUT.print((uint32_t)(now - sequenceLastEvent[i]));
      DEBUG_OUT.print(" mp=");
      DEBUG_OUT.println(globalSettings.thresholdMultiPress);
#endif
      sequenceProgress[i] = 0;
      sequenceTermPressCount[i] = 0;
    }

    uint8_t p = sequenceProgress[i];
    if (p >= triggerEntries[i].termCount) p = 0;

    bool matchesNext = false;
    if (p < triggerEntries[i].termCount) {
      matchesNext = (triggerEntries[i].terms[p].buttonIndex == buttonIndex &&
                     triggerEntries[i].terms[p].triggerType == triggerType);
    }

    if (matchesNext) {
#ifdef DEBUG_OUTPUT_FULL
      DEBUG_OUT.print("TG SEQ match: #");
      DEBUG_OUT.print(i);
      DEBUG_OUT.print(" step=");
      DEBUG_OUT.print(p);
      DEBUG_OUT.print(" term=");
      debugPrintTriggerTerm(&triggerEntries[i].terms[p]);
      DEBUG_OUT.println("");
#endif
      p++;
      sequenceProgress[i] = p;
      sequenceLastEvent[i] = now;
      sequenceTermPressCount[i] = 0;
      if (p >= triggerEntries[i].termCount) {
#ifdef DEBUG_OUTPUT_FULL
        DEBUG_OUT.print("TG SEQ complete: #");
        DEBUG_OUT.println(i);
#endif
        releaseSequenceHoldActions((int8_t)i);
        executeTriggerAction((int8_t)i);
        sequenceProgress[i] = 0;
        sequenceTermPressCount[i] = 0;
        anyCompleted = true;
      }
      continue;
    }

    if (triggerEntries[i].terms[0].buttonIndex == buttonIndex &&
        triggerEntries[i].terms[0].triggerType == triggerType) {
#ifdef DEBUG_OUTPUT_FULL
      DEBUG_OUT.print("TG SEQ start: #");
      DEBUG_OUT.print(i);
      DEBUG_OUT.print(" term=");
      debugPrintTriggerTerm(&triggerEntries[i].terms[0]);
      DEBUG_OUT.println("");
#endif
      sequenceProgress[i] = 1;
      sequenceLastEvent[i] = now;
      sequenceTermPressCount[i] = 0;
    } else {
      sequenceProgress[i] = 0;
      sequenceTermPressCount[i] = 0;
    }
  }

  return anyCompleted;
}

static void processSequencePressEvent(uint8_t buttonIndex, uint32_t now)
{
  if (globalSettings.thresholdMultiPress == 0) return;

#ifdef DEBUG_OUTPUT_FULL
  DEBUG_OUT.print("TG PRESS EVT: ");
  DEBUG_OUT.print(buttonIndexName(buttonIndex));
  DEBUG_OUT.print(" t=");
  DEBUG_OUT.println(now);
#endif

  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount < 2) continue;
    if (sequenceProgress[i] == 0 || sequenceProgress[i] >= triggerEntries[i].termCount) continue;

    uint32_t timeoutBaseline = (sequenceTermPressCount[i] > 0) ? sequenceTermPressTime[i] : sequenceLastEvent[i];
    if ((uint32_t)(now - timeoutBaseline) > globalSettings.thresholdMultiPress) {
    #ifdef DEBUG_OUTPUT_FULL
      DEBUG_OUT.print("TG PRESS reset(timeout): #");
      DEBUG_OUT.print(i);
      DEBUG_OUT.print(" gap=");
      DEBUG_OUT.print((uint32_t)(now - timeoutBaseline));
      DEBUG_OUT.print(" mp=");
      DEBUG_OUT.println(globalSettings.thresholdMultiPress);
    #endif
      sequenceProgress[i] = 0;
      sequenceTermPressCount[i] = 0;
      continue;
    }

    struct TriggerTerm expected = triggerEntries[i].terms[sequenceProgress[i]];
    if (expected.buttonIndex != buttonIndex) continue;
    // For TAP terms: only proceed with multi-press latching if tapCount >= 2
    if (!(expected.triggerType == TRIGGER_TYPE_TAP && expected.tapCount >= 2)) {
      // Expected button pressed but it's not a multi-tap — refresh timeout from this press
      // so subsequent presses don't incorrectly expire the window
      sequenceTermPressCount[i] = 0;
      sequenceLastEvent[i] = now;
      continue;
    }

    uint8_t neededPresses = expected.tapCount;
    if (sequenceTermPressCount[i] == 0) {
#ifdef DEBUG_OUTPUT_FULL
      DEBUG_OUT.print("TG PRESS latch-start: #");
      DEBUG_OUT.print(i);
      DEBUG_OUT.print(" expect=");
      debugPrintTriggerTerm(&expected);
      DEBUG_OUT.println("");
#endif
      sequenceTermPressCount[i] = 1;
      sequenceTermPressTime[i] = now;
      continue;
    }

    if ((uint32_t)(now - sequenceTermPressTime[i]) <= globalSettings.thresholdMultiPress) {
      sequenceTermPressCount[i]++;
#ifdef DEBUG_OUTPUT_FULL
      DEBUG_OUT.print("TG PRESS latch-advance: #");
      DEBUG_OUT.print(i);
      DEBUG_OUT.print(" count=");
      DEBUG_OUT.print(sequenceTermPressCount[i]);
      DEBUG_OUT.print(" need=");
      DEBUG_OUT.print(neededPresses);
      DEBUG_OUT.print(" gap=");
      DEBUG_OUT.println((uint32_t)(now - sequenceTermPressTime[i]));
#endif
      sequenceTermPressTime[i] = now;
      if (sequenceTermPressCount[i] >= neededPresses) {
        uint8_t p = sequenceProgress[i] + 1;
#ifdef DEBUG_OUTPUT_FULL
        DEBUG_OUT.print("TG PRESS term-complete: #");
        DEBUG_OUT.print(i);
        DEBUG_OUT.print(" expect=");
        debugPrintTriggerTerm(&expected);
        DEBUG_OUT.println("");
#endif
        sequenceProgress[i] = p;
        sequenceLastEvent[i] = now;
        sequenceTermPressCount[i] = 0;
        if (p >= triggerEntries[i].termCount) {
#ifdef DEBUG_OUTPUT_FULL
          DEBUG_OUT.print("TG PRESS complete: #");
          DEBUG_OUT.println(i);
#endif
          releaseSequenceHoldActions((int8_t)i);
          executeTriggerAction((int8_t)i);
          sequenceProgress[i] = 0;
        }
      }
    } else {
#ifdef DEBUG_OUTPUT_FULL
      DEBUG_OUT.print("TG PRESS latch-restart(timeout): #");
      DEBUG_OUT.print(i);
      DEBUG_OUT.print(" gap=");
      DEBUG_OUT.print((uint32_t)(now - sequenceTermPressTime[i]));
      DEBUG_OUT.print(" mp=");
      DEBUG_OUT.println(globalSettings.thresholdMultiPress);
#endif
      sequenceTermPressCount[i] = 1;
      sequenceTermPressTime[i] = now;
    }
  }
}

static bool isAwaitingCompositeSingleTerm(uint8_t buttonIndex)
{
  if (globalSettings.thresholdMultiPress == 0) return false;

  uint32_t now = millis();
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount < 2) continue;

    uint8_t p = sequenceProgress[i];
    if (p == 0 || p >= triggerEntries[i].termCount) continue;

    if ((uint32_t)(now - sequenceLastEvent[i]) > globalSettings.thresholdMultiPress)
      continue;

    if (triggerEntries[i].terms[p].triggerType == TRIGGER_TYPE_TAP && triggerEntries[i].terms[p].tapCount == 1 &&
        triggerEntries[i].terms[p].buttonIndex == buttonIndex)
      return true;
  }

  return false;
}

static int8_t findTapTrigger(uint8_t buttonIndex, uint8_t tapCount)
{
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount == 1 &&
        triggerEntries[i].terms[0].buttonIndex == buttonIndex &&
        triggerEntries[i].terms[0].triggerType == TRIGGER_TYPE_TAP &&
        triggerEntries[i].terms[0].tapCount == tapCount) {
      return (int8_t)i;
    }
  }
  return -1;
}

static uint8_t getMaxTapCountForButton(uint8_t buttonIndex)
{
  uint8_t maxTapCount = 0;
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount == 0) continue;
    for (uint8_t t = 0; t < triggerEntries[i].termCount; t++) {
      if (triggerEntries[i].terms[t].buttonIndex == buttonIndex &&
          triggerEntries[i].terms[t].triggerType == TRIGGER_TYPE_TAP) {
        if (triggerEntries[i].terms[t].tapCount > maxTapCount) {
          maxTapCount = triggerEntries[i].terms[t].tapCount;
        }
      }
    }
  }
  return maxTapCount;
}

static int8_t findSingleTapTrigger(uint8_t buttonIndex)
{
  return findTapTrigger(buttonIndex, 1);
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
    { struct TriggerTerm t = {TRIGGER_TYPE_PRESS, 0, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_NE, 0, ""); }               // B1: next slot
    { struct TriggerTerm t = {TRIGGER_TYPE_PRESS, UP_BUTTON, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_KH, 0, "KEY_UP "); }        // up: hold KEY_UP
    { struct TriggerTerm t = {TRIGGER_TYPE_PRESS, DOWN_BUTTON, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_KH, 0, "KEY_DOWN "); }      // down: hold KEY_DOWN
    { struct TriggerTerm t = {TRIGGER_TYPE_PRESS, LEFT_BUTTON, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_KH, 0, "KEY_LEFT "); }      // left: hold KEY_LEFT
    { struct TriggerTerm t = {TRIGGER_TYPE_PRESS, RIGHT_BUTTON, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_KH, 0, "KEY_RIGHT "); }     // right: hold KEY_RIGHT
    { struct TriggerTerm t = {TRIGGER_TYPE_PRESS, SIP_BUTTON, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_HL, 0, ""); }               // sip: hold left mouse
    { struct TriggerTerm t = {TRIGGER_TYPE_TAP, PUFF_BUTTON, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_CR, 0, ""); }               // puff: click right
    { struct TriggerTerm t = {TRIGGER_TYPE_TAP, STRONGPUFF_BUTTON, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_CA, 0, ""); }               // strongpuff: calibrate
  #else
    { struct TriggerTerm t = {TRIGGER_TYPE_PRESS, 0, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_KH, 0, "KEY_SPACE "); }     // B1: hold SPACE
    { struct TriggerTerm t = {TRIGGER_TYPE_PRESS, 1, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_KH, 0, "KEY_ENTER "); }     // B2: hold ENTER
    { struct TriggerTerm t = {TRIGGER_TYPE_TAP, 2, 1, 0};
      addOrReplaceTriggerSequence(&t, 1, CMD_CL, 0, ""); }               // B3: click left
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
  pendingLongTrigIdx[buttonIndex] = -1;
  pendingLongEmittedDuration[buttonIndex] = 0;
  processSequencePressEvent((uint8_t)buttonIndex, buttonDebouncers[buttonIndex].timestamp);
  // Hierarchical handling for non-hold actions:
  // if higher tap counts are configured, delay firing until timeout so only the
  // most specific tap-count action is executed.
  uint8_t maxTapCount = getMaxTapCountForButton((uint8_t)buttonIndex);
  bool hasMultiTapTriggers = (maxTapCount > 1);
  bool awaitingCompositeSingleTerm = isAwaitingCompositeSingleTerm((uint8_t)buttonIndex);
  
  if (globalSettings.thresholdMultiPress > 0 &&
      (maxTapCount > 0 || awaitingCompositeSingleTerm)) {
    normalActionStarted[buttonIndex] = 0;
    uint32_t now = millis();
    if (buttonDebouncers[buttonIndex].multiPending &&
        ((uint32_t)(now - buttonDebouncers[buttonIndex].lastReleaseTime) <= globalSettings.thresholdMultiPress)) {
      if (buttonDebouncers[buttonIndex].pressCount < maxTapCount)
        buttonDebouncers[buttonIndex].pressCount++;
    } else {
      buttonDebouncers[buttonIndex].pressCount = 1;
      pendingDoubleEventEmitted[buttonIndex] = 0;
    }
    buttonDebouncers[buttonIndex].multiPending = 1;

    // If no more specific trigger can follow, execute immediately.
    if (buttonDebouncers[buttonIndex].pressCount >= maxTapCount && maxTapCount > 1) {
      int8_t trigIdx = findTapTrigger((uint8_t)buttonIndex, maxTapCount);
      if (trigIdx >= 0) {
        releaseButtonHoldAction(buttonIndex);
        executeTriggerAction(trigIdx);
        (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_TAP, millis());
        buttonDebouncers[buttonIndex].pressCount = 0;
        buttonDebouncers[buttonIndex].multiPending = 0;
        pendingDoubleEventEmitted[buttonIndex] = 0;
      }
    }
    else if (buttonDebouncers[buttonIndex].pressCount >= 2 && hasMultiTapTriggers) {
      int8_t trigIdx = findTapTrigger((uint8_t)buttonIndex, 2);
      if (trigIdx >= 0) {
        if (maxTapCount > 2) {
          if (!pendingDoubleEventEmitted[buttonIndex]) {
            (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_TAP, millis());
            pendingDoubleEventEmitted[buttonIndex] = 1;
          }
        } else {
          releaseButtonHoldAction(buttonIndex);
          executeTriggerAction(trigIdx);
          (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_TAP, millis());
          buttonDebouncers[buttonIndex].pressCount = 0;
          buttonDebouncers[buttonIndex].multiPending = 0;
          pendingDoubleEventEmitted[buttonIndex] = 0;
        }
      }
    }
    return;
  }

  // Check for simple (single-term) PRESS triggers and execute immediately
  for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
    if (triggerEntries[j].termCount == 1 &&
        triggerEntries[j].terms[0].buttonIndex == (uint8_t)buttonIndex &&
        triggerEntries[j].terms[0].triggerType == TRIGGER_TYPE_PRESS) {
      // Mirror hold-type actions into buttons[] so handleRelease auto-releases them
      buttons[buttonIndex].mode = triggerEntries[j].mode;
      buttons[buttonIndex].value = triggerEntries[j].value;
      setButtonKeystring(buttonIndex, getTriggerKeystring(j));
      executeTriggerAction((int8_t)j);
      normalActionStarted[buttonIndex] = inHoldMode(buttonIndex) ? 1 : 0;
      return;
    }
  }

  // Check for composite PRESS terms and emit event for sequence processing
  for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
    if (triggerEntries[j].termCount >= 2) {
      for (uint8_t t = 0; t < triggerEntries[j].termCount; t++) {
        if (triggerEntries[j].terms[t].buttonIndex == (uint8_t)buttonIndex &&
            triggerEntries[j].terms[t].triggerType == TRIGGER_TYPE_PRESS) {
          (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_PRESS, buttonDebouncers[buttonIndex].timestamp);
          normalActionStarted[buttonIndex] = 0;
          return;
        }
      }
    }
  }

  // No hierarchical multi-press case: execute normal action immediately.
}

void handleRelease (int buttonIndex)    // a button was released: deal with "sticky"-functions
{
  #ifdef DEBUG_OUTPUT_FULL
    Serial.print("R: "); Serial.println(buttonIndex);
  #endif
  buttonStates &= ~(1<<buttonIndex); //save for reporting
  uint32_t releaseNow = millis();
  buttonDebouncers[buttonIndex].lastReleaseTime = releaseNow;  // for multi-press detection

  // Refresh the sequence timeout baseline so the MP window is measured from
  // the release of the preceding term's button, not from its press.
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount < 2) continue;
    if (sequenceProgress[i] == 0 || sequenceProgress[i] >= triggerEntries[i].termCount) continue;
    if (sequenceTermPressCount[i] > 0) continue; // already latching next term, don't disturb
    // Check if this button belongs to the preceding (already matched) term
    uint8_t prevTermIdx = sequenceProgress[i] - 1;
    if (triggerEntries[i].terms[prevTermIdx].buttonIndex == (uint8_t)buttonIndex) {
      sequenceLastEvent[i] = releaseNow;
    }
  }

  // Fire deferred long-press trigger if button released before highest duration tier was crossed
  if (pendingLongTrigIdx[buttonIndex] >= 0 && !buttonDebouncers[buttonIndex].longPressed) {
    int8_t trigIdx = pendingLongTrigIdx[buttonIndex];
    pendingLongTrigIdx[buttonIndex] = -1;
    buttonDebouncers[buttonIndex].longPressed = 1;
    buttonDebouncers[buttonIndex].pressCount = 0;
    buttonDebouncers[buttonIndex].multiPending = 0;
    releaseButtonHoldAction(buttonIndex);
    executeTriggerAction(trigIdx);
    normalActionStarted[buttonIndex] = 0;
    pendingDoubleEventEmitted[buttonIndex] = 0;
    return;
  }

  // Check for simple (single-term) RELEASE triggers and execute immediately
  for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
    if (triggerEntries[j].termCount == 1 &&
        triggerEntries[j].terms[0].buttonIndex == (uint8_t)buttonIndex &&
        triggerEntries[j].terms[0].triggerType == TRIGGER_TYPE_RELEASE) {
      executeTriggerAction((int8_t)j);
      normalActionStarted[buttonIndex] = 0;
      pendingDoubleEventEmitted[buttonIndex] = 0;
      return;
    }
  }

  // Check for composite RELEASE terms and emit event for sequence processing
  for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
    if (triggerEntries[j].termCount >= 2) {
      for (uint8_t t = 0; t < triggerEntries[j].termCount; t++) {
        if (triggerEntries[j].terms[t].buttonIndex == (uint8_t)buttonIndex &&
            triggerEntries[j].terms[t].triggerType == TRIGGER_TYPE_RELEASE) {
          (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_RELEASE, releaseNow);
          normalActionStarted[buttonIndex] = 0;
          pendingDoubleEventEmitted[buttonIndex] = 0;
          return;
        }
      }
    }
  }

  if (buttonDebouncers[buttonIndex].multiPending) {
    uint8_t maxTapCount = getMaxTapCountForButton((uint8_t)buttonIndex);
    int8_t trigSingle = findSingleTapTrigger((uint8_t)buttonIndex);

    // For awaited composite single terms, resolve immediately on release.
    if (buttonDebouncers[buttonIndex].pressCount == 1 && maxTapCount <= 1) {
      bool hasStandaloneSingleTap = (trigSingle >= 0);
      // Check if composite chain also needs single-tap
      bool hasCompositeSingleTap = false;
      for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
        if (triggerEntries[j].termCount >= 2) {
          for (uint8_t t = 0; t < triggerEntries[j].termCount; t++) {
            if (triggerEntries[j].terms[t].buttonIndex == (uint8_t)buttonIndex &&
                triggerEntries[j].terms[t].triggerType == TRIGGER_TYPE_TAP &&
                triggerEntries[j].terms[t].tapCount == 1) {
              hasCompositeSingleTap = true;
              break;
            }
          }
        }
      }
      if (hasStandaloneSingleTap && !hasCompositeSingleTap) {
        if (globalSettings.thresholdMultiPress == 0) {
          executeTriggerAction(trigSingle);
          buttonDebouncers[buttonIndex].pressCount = 0;
          buttonDebouncers[buttonIndex].multiPending = 0;
          normalActionStarted[buttonIndex] = 0;
          pendingDoubleEventEmitted[buttonIndex] = 0;
        }
        return;
      }
      if (!(hasCompositeSingleTap)) {
      }  // (no fallback normal action: AT BM removed, all actions via AT TG)
      buttonDebouncers[buttonIndex].pressCount = 0;
      buttonDebouncers[buttonIndex].multiPending = 0;
      normalActionStarted[buttonIndex] = 0;
      pendingDoubleEventEmitted[buttonIndex] = 0;
      return;
    }
  }

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

  normalActionStarted[buttonIndex] = 0;

  if (globalSettings.thresholdMultiPress > 0) {
    uint8_t maxTapCount = getMaxTapCountForButton((uint8_t)buttonIndex);
    if (maxTapCount == 0) return;  // No tap triggers for this button
    
    int8_t trigIdx = findTapTrigger((uint8_t)buttonIndex, maxTapCount);
    if (trigIdx < 0 && maxTapCount > 1) {
      // Max count trigger doesn't exist, try lower counts
      for (uint8_t cnt = maxTapCount - 1; cnt >= 1; cnt--) {
        trigIdx = findTapTrigger((uint8_t)buttonIndex, cnt);
        if (trigIdx >= 0) break;
      }
    }
    
    bool hasSingleTap = (findSingleTapTrigger((uint8_t)buttonIndex) >= 0);
    if (!hasSingleTap) return;
    
    // Emit tap event only if there are no higher-count triggers that could still be pending
    if (maxTapCount == 1) {
      (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_TAP,
                   buttonDebouncers[buttonIndex].lastReleaseTime);
    }
  }
}

void processLongPressTriggers()
{
  uint32_t now = millis();
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    if ((buttonStates & (1UL << i)) == 0) continue;   // currently not pressed
    if (buttonDebouncers[i].longPressed) continue;    // already fired during this press

    uint32_t heldFor = now - buttonDebouncers[i].timestamp;

    // ---- Emit LONG events for composite sequence terms at their specific durations ----
    // Each unique threshold is emitted once per press (gated by pendingLongEmittedDuration).
    for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
      if (triggerEntries[j].termCount < 2) continue;
      for (uint8_t t = 0; t < triggerEntries[j].termCount; t++) {
        if (triggerEntries[j].terms[t].buttonIndex != (uint8_t)i) continue;
        if (triggerEntries[j].terms[t].triggerType != TRIGGER_TYPE_LONG) continue;
        uint32_t dur = triggerEntries[j].terms[t].duration;
        if (dur == 0) dur = globalSettings.thresholdLongPress;
        if (dur == 0) break;
        if (heldFor >= dur && dur > pendingLongEmittedDuration[i]) {
          pendingLongEmittedDuration[i] = dur;
          buttonDebouncers[i].pressCount = 0;
          buttonDebouncers[i].multiPending = 0;
          releaseButtonHoldAction(i);
          (void)emitTriggerEvent((uint8_t)i, TRIGGER_TYPE_LONG, now);
        }
        break; // only first long term per trigger entry matters
      }
    }

    // ---- Simple single-term long triggers: multi-tier deferred firing ----
    // Collect the highest exceeded tier; defer if a higher tier still exists.
    int8_t   bestFitIdx      = -1;
    uint32_t bestFitDuration = 0;
    bool     hasHigher       = false;
    for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
      if (triggerEntries[j].termCount != 1) continue;
      if (triggerEntries[j].terms[0].buttonIndex != (uint8_t)i) continue;
      if (triggerEntries[j].terms[0].triggerType != TRIGGER_TYPE_LONG) continue;
      uint32_t dur = triggerEntries[j].terms[0].duration;
      if (dur == 0) dur = globalSettings.thresholdLongPress;
      if (dur == 0) continue;
      if (heldFor >= dur) {
        if (dur > bestFitDuration) {
          bestFitDuration = dur;
          bestFitIdx = j;
        }
      } else {
        hasHigher = true;
      }
    }

    if (bestFitIdx >= 0) {
      pendingLongTrigIdx[i] = bestFitIdx;   // update best tier seen so far
      if (!hasHigher) {
        // Highest tier has been crossed — fire immediately
        buttonDebouncers[i].longPressed = 1;
        pendingLongTrigIdx[i] = -1;
        buttonDebouncers[i].pressCount = 0;
        buttonDebouncers[i].multiPending = 0;
        releaseButtonHoldAction(i);
        executeTriggerAction(bestFitIdx);
      }
      // else: still waiting for a higher tier; fire on release via handleRelease
    }
  }
}

void processMultiPressTriggers()
{
  if (globalSettings.thresholdMultiPress == 0) return;

  uint32_t now = millis();
  for (int i = 0; i < NUMBER_OF_BUTTONS; i++) {
    if (!buttonDebouncers[i].multiPending) continue;

    // If button is still held, continue waiting
    if ((buttonStates & (1UL << i)) != 0) {
      continue;
    }

    // Button is released; check if we're still in multi-press window
    if (buttonDebouncers[i].lastReleaseTime == 0) continue;
    if ((uint32_t)(now - buttonDebouncers[i].lastReleaseTime) <= globalSettings.thresholdMultiPress)
      continue; // still within multi-press window

    // Multi-press window has expired; execute the appropriate trigger for current press count
    uint8_t maxTapCount = getMaxTapCountForButton((uint8_t)i);
    if (maxTapCount == 0) {
      buttonDebouncers[i].multiPending = 0;
      continue;
    }

    // Find trigger for current press count, or highest available if exact count doesn't exist
    int8_t bestTrigIdx = -1;
    uint8_t bestCount = 0;
    for (uint8_t cnt = maxTapCount; cnt >= 1; cnt--) {
      int8_t trigIdx = findTapTrigger((uint8_t)i, cnt);
      if (trigIdx >= 0 && cnt <= buttonDebouncers[i].pressCount) {
        bestTrigIdx = trigIdx;
        bestCount = cnt;
        break;
      }
    }

    if (bestTrigIdx >= 0) {
      releaseButtonHoldAction(i);
      executeTriggerAction(bestTrigIdx);
      if (bestCount == 1) {
        (void)emitTriggerEvent((uint8_t)i, TRIGGER_TYPE_TAP, 
                     buttonDebouncers[i].lastReleaseTime);
      } else {
        (void)emitTriggerEvent((uint8_t)i, TRIGGER_TYPE_TAP, millis());
      }
    } else {
      // Check for composite single-tap in multi-term triggers
      bool hasCompositeSingleTap = false;
      for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
        if (triggerEntries[j].termCount >= 2) {
          for (uint8_t t = 0; t < triggerEntries[j].termCount; t++) {
            if (triggerEntries[j].terms[t].buttonIndex == (uint8_t)i &&
                triggerEntries[j].terms[t].triggerType == TRIGGER_TYPE_TAP &&
                triggerEntries[j].terms[t].tapCount == 1) {
              hasCompositeSingleTap = true;
              break;
            }
          }
          if (hasCompositeSingleTap) break;
        }
      }
      if (hasCompositeSingleTap && buttonDebouncers[i].pressCount == 1) {
        (void)emitTriggerEvent((uint8_t)i, TRIGGER_TYPE_TAP,
                     buttonDebouncers[i].lastReleaseTime);
      }
    }

    buttonDebouncers[i].multiPending = 0;
    buttonDebouncers[i].pressCount = 0;
    normalActionStarted[i] = 0;
    pendingDoubleEventEmitted[i] = 0;
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
      // --- long-press trigger detection (for virtual buttons not tracked by processLongPressTriggers) ---
      if (buttonDebouncers[i].stableState == 1 &&   // button is currently held
          !buttonDebouncers[i].longPressed) {
        // Check if any long-press trigger is configured for this button.
        // If yes, processLongPressTriggers handles timing and deferred firing — skip here.
        // If no trigger exists at any duration, suppress repeated polling via longPressed.
        bool anyLongThreshold = false;
        uint32_t heldFor = (uint32_t)(millis() - buttonDebouncers[i].timestamp);
        for (int j = 0; j < MAX_TRIGGER_COUNT; j++) {
          if (triggerEntries[j].termCount == 0) continue;
          for (uint8_t t = 0; t < triggerEntries[j].termCount; t++) {
            if (triggerEntries[j].terms[t].buttonIndex == (uint8_t)i &&
                triggerEntries[j].terms[t].triggerType == TRIGGER_TYPE_LONG) {
              uint32_t dur = triggerEntries[j].terms[t].duration;
              if (dur == 0) dur = globalSettings.thresholdLongPress;
              if (dur > 0 && heldFor >= dur) { anyLongThreshold = true; }
            }
          }
        }
        if (!anyLongThreshold && globalSettings.thresholdLongPress > 0 &&
            heldFor >= globalSettings.thresholdLongPress &&
            !hasTriggerTerm((uint8_t)i, TRIGGER_TYPE_LONG)) {
          // No trigger configured: suppress repeated checks only
          buttonDebouncers[i].longPressed = 1;
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
    normalActionStarted[i] = 0;
    pendingDoubleEventEmitted[i] = 0;
    pendingLongTrigIdx[i] = -1;
    pendingLongEmittedDuration[i] = 0;
  }
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    sequenceProgress[i] = 0;
    sequenceLastEvent[i] = 0;
    sequenceTermPressCount[i] = 0;
    sequenceTermPressTime[i] = 0;
  }
}
