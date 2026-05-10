/*
  FabiWare - AsTeRICS Foundation
  For more info please visit: https://www.asterics-foundation.org

  Module: triggers.h - header for the complex trigger system (AT TG command)

  The trigger system allows mapping complex input gestures (long press, double press,
  triple press) of any button to additional actions, independent of the button's
  primary action.

  Usage (AT command interface, two-step like AT BM):
    AT TG long(B1)       -> next AT command stored as long-press action for button 1
    AT TG double(sip)    -> next AT command stored as double-press action for sip
    AT TG triple(puff)   -> next AT command stored as triple-press action for puff
    AT TG list           -> list all active trigger definitions
    AT TG clear          -> remove all trigger definitions
    AT TG clear(B1)      -> remove all trigger definitions for button B1

  Supported button names:
    B1..B5 (physical buttons), up, down, left, right,
    sip, puff, strongsip, strongpuff,
    ssup, ssdown, ssleft, ssright  (strongsip + direction)
    spup, spdown, spleft, spright  (strongpuff + direction)

  Trigger thresholds are shared with:
    AT LP <ms>   - long-press threshold  (globalSettings.thresholdLongPress)
    AT MP <ms>   - multi-press threshold (globalSettings.thresholdMultiPress)

  Long-press behavior (additive):
    The button's normal action fires immediately on press (unchanged).
    After thresholdLongPress ms while the button is still held, the
    long-press trigger action fires additionally.

  Multi-press behavior (additive):
    The button's normal action fires on every press (unchanged).
    On the 2nd press within thresholdMultiPress ms the double-press trigger fires.
    On the 3rd press within thresholdMultiPress ms the triple-press trigger fires.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation.
*/

#ifndef _TRIGGERS_H_
#define _TRIGGERS_H_

#include <Arduino.h>

// Trigger type identifiers
#define TRIGGER_TYPE_SINGLE  0  // single press
#define TRIGGER_TYPE_LONG    1  // long press
#define TRIGGER_TYPE_DOUBLE  2  // double press
#define TRIGGER_TYPE_TRIPLE  3  // triple press

#define MAX_TRIGGER_COUNT              10   // maximum triggers per slot
#define MAX_TRIGGER_KEYSTRING_BUFFER  150   // shared keystring storage (bytes)
#define MAX_TRIGGER_TERMS               4   // maximum terms per trigger expression

struct TriggerTerm {
   uint8_t triggerType;
   uint8_t buttonIndex;
};

/**
   TriggerEntry struct
   Associates a (button, trigger-type) pair with an action command.
*/
struct TriggerEntry {
   uint8_t  termCount;    // 0 = unused slot
   struct TriggerTerm terms[MAX_TRIGGER_TERMS];
  uint16_t mode;         // action: CMD_* identifier
  int16_t  value;        // numeric parameter for the action
};

extern struct TriggerEntry triggerEntries[MAX_TRIGGER_COUNT];
extern char triggerKeystringBuffer[MAX_TRIGGER_KEYSTRING_BUFFER];
extern char *triggerKeystrings[MAX_TRIGGER_COUNT];

/**
   @name initTriggers
   @brief Clear all trigger entries and reset the keystring buffer.
   @return none
*/
void initTriggers();

/**
   @name findTrigger
   @brief Search for a trigger entry matching (buttonIndex, triggerType).
   @return index into triggerEntries[], or -1 if not found
*/
int8_t findTrigger(uint8_t buttonIndex, uint8_t triggerType);

/**
   @name addOrReplaceTrigger
   @brief Store or overwrite a trigger entry.
   @param buttonIndex 0-based button index
   @param triggerType TRIGGER_TYPE_LONG / _DOUBLE / _TRIPLE
   @param mode        action command (CMD_*)
   @param value       numeric parameter
   @param keystring   string parameter (may be empty)
   @return index of the entry, or -1 if table is full
*/
int8_t addOrReplaceTrigger(uint8_t buttonIndex, uint8_t triggerType,
                           uint16_t mode, int16_t value, const char *keystring);

/**
   @name addOrReplaceTriggerSequence
   @brief Store or overwrite a trigger entry with 1..MAX_TRIGGER_TERMS terms.
   @param terms      array of trigger terms
   @param termCount  number of terms in the array
   @param mode       action command (CMD_*)
   @param value      numeric parameter
   @param keystring  string parameter (may be empty)
   @return index of the entry, or -1 on error / table full
*/
int8_t addOrReplaceTriggerSequence(const struct TriggerTerm *terms, uint8_t termCount,
                                   uint16_t mode, int16_t value, const char *keystring);

/**
   @name clearTriggers
   @brief Remove trigger entries.
   @param buttonIndex button to clear, or -1 to clear all entries
   @return none
*/
void clearTriggers(int buttonIndex);

/**
   @name getTriggerKeystring
   @brief Return the keystring for trigger entry at trigIdx.
   @return pointer to the keystring (may be empty string, never NULL)
*/
char *getTriggerKeystring(int8_t trigIdx);

/**
   @name listTriggers
   @brief Print all active trigger definitions to Serial.
   @return none
*/
void listTriggers();

/**
   @name printTriggersForSlot
   @brief Emit AT TG commands for all active triggers to the given stream.
          Called by printCurrentSlot() when saving a slot to flash.
   @param S output stream (Serial or File)
   @return none
*/
void printTriggersForSlot(Stream *S);

/**
   @name parseButtonName
   @brief Convert a button name string to a 0-based button index.
   @param name e.g. "B1", "sip", "up"
   @return button index, or -1 if unrecognised
*/
int8_t parseButtonName(const char *name);

/**
   @name buttonIndexName
   @brief Return the canonical name string for a 0-based button index.
   @return e.g. "B1", "sip", "up"  (pointer to a string literal)
*/
const char *buttonIndexName(uint8_t idx);

/**
   @name triggerTypeName
   @brief Return the trigger type as a string.
   @return "long", "double", or "triple"
*/
const char *triggerTypeName(uint8_t type);

/**
   @name hasTriggerTerm
   @brief Check if any trigger entry contains the given (button, triggerType) term.
   @return true if such a term exists
*/
bool hasTriggerTerm(uint8_t buttonIndex, uint8_t triggerType);

/**
   @name hasCompositeTriggerTerm
   @brief Check whether a term appears in any trigger with more than one term.
   @return true if a composite trigger contains this term
*/
bool hasCompositeTriggerTerm(uint8_t buttonIndex, uint8_t triggerType);

/**
   @name isTriggerSupportedButton
   @brief Check whether a button index can be used with AT TG triggers.
   @param idx 0-based button index
   @return true if trigger assignment is supported
*/
bool isTriggerSupportedButton(uint8_t idx);

#endif
