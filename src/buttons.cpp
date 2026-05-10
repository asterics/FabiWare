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
    if (expected.triggerType != TRIGGER_TYPE_DOUBLE && expected.triggerType != TRIGGER_TYPE_TRIPLE) {
      // Expected button pressed but it's single/long — refresh timeout from this press
      // so subsequent presses don't incorrectly expire the window
      sequenceTermPressCount[i] = 0;
      sequenceLastEvent[i] = now;
      continue;
    }

    uint8_t neededPresses = (expected.triggerType == TRIGGER_TYPE_DOUBLE) ? 2 : 3;
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

    if (triggerEntries[i].terms[p].triggerType == TRIGGER_TYPE_SINGLE &&
        triggerEntries[i].terms[p].buttonIndex == buttonIndex)
      return true;
  }

  return false;
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
  processSequencePressEvent((uint8_t)buttonIndex, buttonDebouncers[buttonIndex].timestamp);
  // Hierarchical handling for non-hold actions:
  // if double/triple are configured, delay firing until timeout so only the
  // most specific matching action (single/double/triple) is executed.
    int8_t trigDouble = findTrigger((uint8_t)buttonIndex, TRIGGER_TYPE_DOUBLE);
    int8_t trigTriple = findTrigger((uint8_t)buttonIndex, TRIGGER_TYPE_TRIPLE);
    bool wantsDoubleTerm = hasTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_DOUBLE);
    bool wantsTripleTerm = hasTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_TRIPLE);
  bool awaitingCompositeSingleTerm = isAwaitingCompositeSingleTerm((uint8_t)buttonIndex);
  if (globalSettings.thresholdMultiPress > 0 &&
      (wantsDoubleTerm || wantsTripleTerm || awaitingCompositeSingleTerm)) {
    normalActionStarted[buttonIndex] = 0;
    uint32_t now = millis();
    if (buttonDebouncers[buttonIndex].multiPending &&
        ((uint32_t)(now - buttonDebouncers[buttonIndex].lastReleaseTime) <= globalSettings.thresholdMultiPress)) {
      if (buttonDebouncers[buttonIndex].pressCount < 3)
        buttonDebouncers[buttonIndex].pressCount++;
    } else {
      buttonDebouncers[buttonIndex].pressCount = 1;
      pendingDoubleEventEmitted[buttonIndex] = 0;
    }
    buttonDebouncers[buttonIndex].multiPending = 1;

    // If no more specific trigger can follow, execute immediately.
    if (buttonDebouncers[buttonIndex].pressCount >= 3 && wantsTripleTerm) {
      releaseButtonHoldAction(buttonIndex);
      if (trigTriple >= 0) executeTriggerAction(trigTriple);
      (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_TRIPLE, millis());
      buttonDebouncers[buttonIndex].pressCount = 0;
      buttonDebouncers[buttonIndex].multiPending = 0;
      pendingDoubleEventEmitted[buttonIndex] = 0;
    }
    else if (buttonDebouncers[buttonIndex].pressCount >= 2 && wantsDoubleTerm) {
      if (wantsTripleTerm) {
        if (!pendingDoubleEventEmitted[buttonIndex]) {
          (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_DOUBLE, millis());
          pendingDoubleEventEmitted[buttonIndex] = 1;
        }
      } else {
        releaseButtonHoldAction(buttonIndex);
        if (trigDouble >= 0) executeTriggerAction(trigDouble);
        (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_DOUBLE, millis());
        buttonDebouncers[buttonIndex].pressCount = 0;
        buttonDebouncers[buttonIndex].multiPending = 0;
        pendingDoubleEventEmitted[buttonIndex] = 0;
      }
    }
    return;
  }

  // No hierarchical multi-press case: execute normal action immediately.
  normalActionStarted[buttonIndex] = 1;
  executeNormalButtonAction(buttonIndex);
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

  if (buttonDebouncers[buttonIndex].multiPending) {
    bool wantsDoubleTerm = hasTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_DOUBLE);
    bool wantsTripleTerm = hasTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_TRIPLE);

    // For awaited composite single terms, resolve immediately on release.
    if (buttonDebouncers[buttonIndex].pressCount == 1 && !wantsDoubleTerm && !wantsTripleTerm) {
      bool completedComposite = false;
      if (hasTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_SINGLE)) {
        completedComposite = emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_SINGLE,
                      buttonDebouncers[buttonIndex].lastReleaseTime);
      }

      if (!(completedComposite && hasCompositeTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_SINGLE))) {
        executeNormalButtonAction(buttonIndex);
      }

      buttonDebouncers[buttonIndex].pressCount = 0;
      buttonDebouncers[buttonIndex].multiPending = 0;
      normalActionStarted[buttonIndex] = 0;
      pendingDoubleEventEmitted[buttonIndex] = 0;
      return;
    }
  }

  if (!normalActionStarted[buttonIndex]) {
    return;
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

  if (globalSettings.thresholdMultiPress > 0 &&
      !hasTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_DOUBLE) &&
      !hasTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_TRIPLE) &&
      hasTriggerTerm((uint8_t)buttonIndex, TRIGGER_TYPE_SINGLE)) {
    (void)emitTriggerEvent((uint8_t)buttonIndex, TRIGGER_TYPE_SINGLE,
                 buttonDebouncers[buttonIndex].lastReleaseTime);
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
      bool wantsLongTerm = hasTriggerTerm((uint8_t)i, TRIGGER_TYPE_LONG);
      buttonDebouncers[i].longPressed = 1;            // fire at most once per press
      if (wantsLongTerm) {
        // A long-press trigger supersedes any pending single/double/triple resolution
        // from this press sequence.
        buttonDebouncers[i].pressCount = 0;
        buttonDebouncers[i].multiPending = 0;
        releaseButtonHoldAction(i);
        if (trigIdx >= 0) executeTriggerAction(trigIdx);
        (void)emitTriggerEvent((uint8_t)i, TRIGGER_TYPE_LONG, millis());
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
      bool compositeSingleTerm = hasCompositeTriggerTerm((uint8_t)i, TRIGGER_TYPE_SINGLE);
      if (buttonDebouncers[i].pressCount == 1 && inHoldMode(i) && !compositeSingleTerm &&
          ((uint32_t)(now - buttonDebouncers[i].timestamp) > globalSettings.thresholdMultiPress)) {
        normalActionStarted[i] = 1;
        executeNormalButtonAction(i);
        buttonDebouncers[i].pressCount = 0;
        buttonDebouncers[i].multiPending = 0;
        pendingDoubleEventEmitted[i] = 0;
      }
      continue;
    }

    // lastReleaseTime==0 means no release has happened yet for this sequence.
    if (buttonDebouncers[i].lastReleaseTime == 0) continue;

    if ((uint32_t)(now - buttonDebouncers[i].lastReleaseTime) <= globalSettings.thresholdMultiPress)
      continue; // still within multi-press window

    int8_t trigDouble = findTrigger((uint8_t)i, TRIGGER_TYPE_DOUBLE);
    int8_t trigTriple = findTrigger((uint8_t)i, TRIGGER_TYPE_TRIPLE);
    bool wantsDoubleTerm = hasTriggerTerm((uint8_t)i, TRIGGER_TYPE_DOUBLE);
    bool wantsTripleTerm = hasTriggerTerm((uint8_t)i, TRIGGER_TYPE_TRIPLE);

    if (buttonDebouncers[i].pressCount >= 3 && wantsTripleTerm) {
      releaseButtonHoldAction(i);
      if (trigTriple >= 0) executeTriggerAction(trigTriple);
      (void)emitTriggerEvent((uint8_t)i, TRIGGER_TYPE_TRIPLE, millis());
      pendingDoubleEventEmitted[i] = 0;
    }
    else if (buttonDebouncers[i].pressCount >= 2 && wantsDoubleTerm) {
      releaseButtonHoldAction(i);
      if (trigDouble >= 0) executeTriggerAction(trigDouble);
      if (!pendingDoubleEventEmitted[i]) {
        (void)emitTriggerEvent((uint8_t)i, TRIGGER_TYPE_DOUBLE, millis());
      }
      pendingDoubleEventEmitted[i] = 0;
    }
    else if (buttonDebouncers[i].pressCount >= 1) {
      bool completedComposite = false;
      if (hasTriggerTerm((uint8_t)i, TRIGGER_TYPE_SINGLE)) {
        completedComposite = emitTriggerEvent((uint8_t)i, TRIGGER_TYPE_SINGLE,
                      buttonDebouncers[i].lastReleaseTime);
      }
      // fall back to the normal single action after timeout, unless the single
      // event completed a composite chain that should supersede this action.
      if (!(completedComposite && hasCompositeTriggerTerm((uint8_t)i, TRIGGER_TYPE_SINGLE))) {
        executeNormalButtonAction(i);
      }
      normalActionStarted[i] = 0;
      pendingDoubleEventEmitted[i] = 0;
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
    normalActionStarted[i] = 0;
    pendingDoubleEventEmitted[i] = 0;
  }
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    sequenceProgress[i] = 0;
    sequenceLastEvent[i] = 0;
    sequenceTermPressCount[i] = 0;
    sequenceTermPressTime[i] = 0;
  }
}
