/*
  FabiWare - AsTeRICS Foundation
  For more info please visit: https://www.asterics-foundation.org

  Module: triggers.cpp - implementation of the complex trigger system (AT TG command)

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation.
*/

#include "FabiWare.h"
#include "triggers.h"
#include "buttons.h"
#include "commands.h"
#include "parser.h"

struct TriggerEntry triggerEntries[MAX_TRIGGER_COUNT];
char  triggerKeystringBuffer[MAX_TRIGGER_KEYSTRING_BUFFER];
char *triggerKeystrings[MAX_TRIGGER_COUNT];

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/** Rebuild keystring pointers after modifying the entry table. */
static void rebuildKeystringBuffer()
{
  char   tmp[MAX_TRIGGER_KEYSTRING_BUFFER];
  char  *tmpPtrs[MAX_TRIGGER_COUNT];
  uint16_t pos = 1;   // pos 0 holds a permanent empty string sentinel
  tmp[0] = 0;

  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount > 0) {
      const char *ks = triggerKeystrings[i];
      uint16_t len = (ks && *ks) ? strlen(ks) : 0;
      if (pos + len + 1 < MAX_TRIGGER_KEYSTRING_BUFFER) {
        tmpPtrs[i] = tmp + pos;
        if (len) memcpy(tmp + pos, ks, len);
        tmp[pos + len] = 0;
        pos += len + 1;
      } else {
        tmpPtrs[i] = tmp;  // fallback: empty string
      }
    } else {
      tmpPtrs[i] = tmp;    // unused: point to empty sentinel
    }
  }

  memcpy(triggerKeystringBuffer, tmp, MAX_TRIGGER_KEYSTRING_BUFFER);
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    triggerKeystrings[i] = triggerKeystringBuffer + (tmpPtrs[i] - tmp);
  }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void initTriggers()
{
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    triggerEntries[i].termCount = 0;
    memset(triggerEntries[i].terms, 0, sizeof(triggerEntries[i].terms));
    triggerEntries[i].mode        = 0;
    triggerEntries[i].value       = 0;
  }
  memset(triggerKeystringBuffer, 0, MAX_TRIGGER_KEYSTRING_BUFFER);
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    triggerKeystrings[i] = triggerKeystringBuffer;  // all point to empty sentinel
  }
}

int8_t findTrigger(uint8_t buttonIndex, uint8_t triggerType)
{
  if (!isTriggerSupportedButton(buttonIndex)) return -1;

  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount == 1 &&
        triggerEntries[i].terms[0].buttonIndex == buttonIndex &&
        triggerEntries[i].terms[0].triggerType == triggerType)
      return (int8_t)i;
  }
  return -1;
}

static bool sameTriggerSpec(const struct TriggerTerm *a, uint8_t ac,
                            const struct TriggerTerm *b, uint8_t bc)
{
  if (ac != bc) return false;
  for (uint8_t i = 0; i < ac; i++) {
    if (a[i].buttonIndex != b[i].buttonIndex ||
        a[i].triggerType != b[i].triggerType)
      return false;
  }
  return true;
}

int8_t addOrReplaceTriggerSequence(const struct TriggerTerm *terms, uint8_t termCount,
                                   uint16_t mode, int16_t value, const char *keystring)
{
  if (!terms || termCount == 0 || termCount > MAX_TRIGGER_TERMS) return -1;
  for (uint8_t i = 0; i < termCount; i++) {
    if (!isTriggerSupportedButton(terms[i].buttonIndex)) return -1;
  }

  if (!keystring) keystring = "";

  int8_t idx = -1;
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount == 0) continue;
    if (sameTriggerSpec(triggerEntries[i].terms, triggerEntries[i].termCount, terms, termCount)) {
      idx = (int8_t)i;
      break;
    }
  }

  if (idx < 0) {
    for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
      if (triggerEntries[i].termCount == 0) { idx = (int8_t)i; break; }
    }
  }

  if (idx < 0) return -1;

  triggerEntries[idx].termCount = termCount;
  memset(triggerEntries[idx].terms, 0, sizeof(triggerEntries[idx].terms));
  for (uint8_t i = 0; i < termCount; i++) {
    triggerEntries[idx].terms[i] = terms[i];
  }
  triggerEntries[idx].mode  = mode;
  triggerEntries[idx].value = value;

  triggerKeystrings[idx] = (char *)keystring;
  rebuildKeystringBuffer();

  return idx;
}

int8_t addOrReplaceTrigger(uint8_t buttonIndex, uint8_t triggerType,
                           uint16_t mode, int16_t value, const char *keystring)
{
  struct TriggerTerm term;
  term.buttonIndex = buttonIndex;
  term.triggerType = triggerType;
  return addOrReplaceTriggerSequence(&term, 1, mode, value, keystring);
}

void clearTriggers(int buttonIndex)
{
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    bool clearEntry = (buttonIndex < 0);
    if (!clearEntry && triggerEntries[i].termCount > 0) {
      for (uint8_t t = 0; t < triggerEntries[i].termCount; t++) {
        if ((int)triggerEntries[i].terms[t].buttonIndex == buttonIndex) {
          clearEntry = true;
          break;
        }
      }
    }
    if (clearEntry) {
      triggerEntries[i].termCount = 0;
      memset(triggerEntries[i].terms, 0, sizeof(triggerEntries[i].terms));
      triggerKeystrings[i] = triggerKeystringBuffer;  // point to empty sentinel
    }
  }
  rebuildKeystringBuffer();
}

char *getTriggerKeystring(int8_t trigIdx)
{
  if (trigIdx < 0 || trigIdx >= MAX_TRIGGER_COUNT)
    return triggerKeystringBuffer;  // empty sentinel
  return triggerKeystrings[trigIdx];
}

bool isTriggerSupportedButton(uint8_t idx)
{
  // Keep button names available in the parser, but disable trigger definitions
  // for strong sip/puff + direction combinations for now.
  switch (idx) {
    case STRONGSIP_UP_BUTTON:
    case STRONGSIP_DOWN_BUTTON:
    case STRONGSIP_LEFT_BUTTON:
    case STRONGSIP_RIGHT_BUTTON:
    case STRONGPUFF_UP_BUTTON:
    case STRONGPUFF_DOWN_BUTTON:
    case STRONGPUFF_LEFT_BUTTON:
    case STRONGPUFF_RIGHT_BUTTON:
      return false;
    default:
      return true;
  }
}

// ---------------------------------------------------------------------------
// Button-name / trigger-type string conversion
// ---------------------------------------------------------------------------

const char *triggerTypeName(uint8_t type)
{
  switch (type) {
    case TRIGGER_TYPE_SINGLE: return "single";
    case TRIGGER_TYPE_LONG:   return "long";
    case TRIGGER_TYPE_DOUBLE: return "double";
    case TRIGGER_TYPE_TRIPLE: return "triple";
    default:                  return "?";
  }
}

bool hasTriggerTerm(uint8_t buttonIndex, uint8_t triggerType)
{
  if (!isTriggerSupportedButton(buttonIndex)) return false;
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount == 0) continue;
    for (uint8_t t = 0; t < triggerEntries[i].termCount; t++) {
      if (triggerEntries[i].terms[t].buttonIndex == buttonIndex &&
          triggerEntries[i].terms[t].triggerType == triggerType)
        return true;
    }
  }
  return false;
}

bool hasCompositeTriggerTerm(uint8_t buttonIndex, uint8_t triggerType)
{
  if (!isTriggerSupportedButton(buttonIndex)) return false;
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount < 2) continue;
    for (uint8_t t = 0; t < triggerEntries[i].termCount; t++) {
      if (triggerEntries[i].terms[t].buttonIndex == buttonIndex &&
          triggerEntries[i].terms[t].triggerType == triggerType)
        return true;
    }
  }
  return false;
}

const char *buttonIndexName(uint8_t idx)
{
  // Physical buttons B1..B(NUMBER_OF_PHYSICAL_BUTTONS)
  if (idx < NUMBER_OF_PHYSICAL_BUTTONS) {
    static char buf[4];
    buf[0] = 'B';
    buf[1] = '0' + idx + 1;
    buf[2] = 0;
    return buf;
  }
  // Virtual buttons
  switch (idx) {
    case UP_BUTTON:              return "up";
    case DOWN_BUTTON:            return "down";
    case LEFT_BUTTON:            return "left";
    case RIGHT_BUTTON:           return "right";
    case SIP_BUTTON:             return "sip";
    case PUFF_BUTTON:            return "puff";
    case STRONGSIP_BUTTON:       return "strongsip";
    case STRONGPUFF_BUTTON:      return "strongpuff";
    case STRONGSIP_UP_BUTTON:    return "ssup";
    case STRONGSIP_DOWN_BUTTON:  return "ssdown";
    case STRONGSIP_LEFT_BUTTON:  return "ssleft";
    case STRONGSIP_RIGHT_BUTTON: return "ssright";
    case STRONGPUFF_UP_BUTTON:   return "spup";
    case STRONGPUFF_DOWN_BUTTON: return "spdown";
    case STRONGPUFF_LEFT_BUTTON: return "spleft";
    case STRONGPUFF_RIGHT_BUTTON:return "spright";
    default: {
      static char unk[6];
      snprintf(unk, sizeof(unk), "B%u", idx);
      return unk;
    }
  }
}

int8_t parseButtonName(const char *name)
{
  if (!name || !*name) return -1;

  // Physical buttons: B1..B9 (case-insensitive leading 'B')
  if ((name[0] == 'B' || name[0] == 'b') && name[1] >= '1' && name[1] <= '9' && name[2] == 0) {
    int8_t n = name[1] - '1';
    if (n < NUMBER_OF_PHYSICAL_BUTTONS) return n;
    return -1;
  }

  // Virtual buttons by name
  if (strcasecmp(name, "up")          == 0) return (int8_t)UP_BUTTON;
  if (strcasecmp(name, "down")        == 0) return (int8_t)DOWN_BUTTON;
  if (strcasecmp(name, "left")        == 0) return (int8_t)LEFT_BUTTON;
  if (strcasecmp(name, "right")       == 0) return (int8_t)RIGHT_BUTTON;
  if (strcasecmp(name, "sip")         == 0) return (int8_t)SIP_BUTTON;
  if (strcasecmp(name, "puff")        == 0) return (int8_t)PUFF_BUTTON;
  if (strcasecmp(name, "strongsip")   == 0) return (int8_t)STRONGSIP_BUTTON;
  if (strcasecmp(name, "strongpuff")  == 0) return (int8_t)STRONGPUFF_BUTTON;
  if (strcasecmp(name, "ssup")        == 0) return (int8_t)STRONGSIP_UP_BUTTON;
  if (strcasecmp(name, "ssdown")      == 0) return (int8_t)STRONGSIP_DOWN_BUTTON;
  if (strcasecmp(name, "ssleft")      == 0) return (int8_t)STRONGSIP_LEFT_BUTTON;
  if (strcasecmp(name, "ssright")     == 0) return (int8_t)STRONGSIP_RIGHT_BUTTON;
  if (strcasecmp(name, "spup")        == 0) return (int8_t)STRONGPUFF_UP_BUTTON;
  if (strcasecmp(name, "spdown")      == 0) return (int8_t)STRONGPUFF_DOWN_BUTTON;
  if (strcasecmp(name, "spleft")      == 0) return (int8_t)STRONGPUFF_LEFT_BUTTON;
  if (strcasecmp(name, "spright")     == 0) return (int8_t)STRONGPUFF_RIGHT_BUTTON;

  return -1;
}

// ---------------------------------------------------------------------------
// Listing and slot serialisation
// ---------------------------------------------------------------------------

void listTriggers()
{
  bool any = false;
  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount == 0) continue;
    any = true;
    Serial.print("TG: ");
    for (uint8_t t = 0; t < triggerEntries[i].termCount; t++) {
      if (t) Serial.print("+");
      Serial.print(triggerTypeName(triggerEntries[i].terms[t].triggerType));
      Serial.print("(");
      Serial.print(buttonIndexName(triggerEntries[i].terms[t].buttonIndex));
      Serial.print(")");
    }
    Serial.print(" -> AT ");
    int actCmd = triggerEntries[i].mode;
    char cmdStr[4];
    if (actCmd >= 0 && actCmd < NUM_COMMANDS) {
      strcpy_FM(cmdStr, (uint_farptr_t_FM)atCommands[actCmd].atCmd);
      Serial.print(cmdStr);
      switch (pgm_read_byte_near(&(atCommands[actCmd].partype))) {
        case PARTYPE_UINT:
        case PARTYPE_INT:    Serial.print(" "); Serial.print(triggerEntries[i].value); break;
        case PARTYPE_STRING: Serial.print(" "); Serial.print(getTriggerKeystring(i));  break;
        default: break;
      }
    }
    Serial.println("");
  }
  if (!any) Serial.println("TG: (none)");
  Serial.println("OK");
}

void printTriggersForSlot(Stream *S)
{
  // Always emit a clear command first so loading the slot resets old triggers.
  S->println("AT TG clear");

  for (int i = 0; i < MAX_TRIGGER_COUNT; i++) {
    if (triggerEntries[i].termCount == 0) continue;
    int actCmd = triggerEntries[i].mode;
    if (actCmd < 0 || actCmd >= NUM_COMMANDS) continue;

    // Step 1: trigger spec
    S->print("AT TG ");
    for (uint8_t t = 0; t < triggerEntries[i].termCount; t++) {
      if (t) S->print("+");
      S->print(triggerTypeName(triggerEntries[i].terms[t].triggerType));
      S->print("(");
      S->print(buttonIndexName(triggerEntries[i].terms[t].buttonIndex));
      S->print(")");
    }
    S->println("");

    // Step 2: action command
    char cmdStr[4];
    strcpy_FM(cmdStr, (uint_farptr_t_FM)atCommands[actCmd].atCmd);
    S->print("AT ");
    S->print(cmdStr);
    switch (pgm_read_byte_near(&(atCommands[actCmd].partype))) {
      case PARTYPE_UINT:
      case PARTYPE_INT:    S->print(" "); S->print(triggerEntries[i].value);         break;
      case PARTYPE_STRING: S->print(" "); S->print(getTriggerKeystring((int8_t)i));  break;
      default: break;
    }
    S->println("");
  }
}
