/*
  FabiWare - AsTeRICS Foundation
  For more info please visit: https://www.asterics-foundation.org

  Module: modes.cpp: implementation of stick operation and special modes

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; See the GNU General Public License:
  http://www.gnu.org/licenses/gpl-3.0.en.html

*/

#include "FabiWare.h"
#include "modes.h"
#include "gpio.h"
#include "tone.h"
#include "sensors.h"

/**
   static variables for mode handling
 * */
// Note: Gesture handling (strongSipPuffState) removed in Phase 3

/**
   forward declarations of module-internal functions
*/
void handleMovement();

void handleUserInteraction()
{
  static uint8_t pressureRising = 0, pressureFalling = 0;
  static int previousPressure = 512;
  static int waitStable = 0;
  static int checkPairing = 0;
  static uint8_t puffState = SIP_PUFF_STATE_IDLE, sipState = SIP_PUFF_STATE_IDLE;
  static uint8_t strongPuffState = SIP_PUFF_STATE_IDLE, strongSipState = SIP_PUFF_STATE_IDLE;
  static uint8_t puffCount = 0, sipCount = 0;
  static uint8_t strongPuffCount = 0, strongSipCount = 0;

  // handle button press and release actions
  for (int i = 0; i < NUMBER_OF_PHYSICAL_BUTTONS; i++) { // update button press / release events
    // update and check physical buttons
    digitalRead(input_map[i]) == LOW ? currentState.buttonStates |= (1<<i) : currentState.buttonStates &= ~(1<<i);
    if  ((currentState.buttonStates & (1<<i)) != (currentState.oldButtonStates & (1<<i))) {
      //if the GPIO is locked for serial printing, don't use it.
      #ifdef DEBUG_LOCK_GPIO    
        if(input_map[i] != DEBUG_LOCK_GPIO) {
          if (currentState.buttonStates & (1<<i)) handlePress(i); 
          else  handleRelease(i);
        }
      #else
        if (currentState.buttonStates & (1<<i)) handlePress(i); 
        else  handleRelease(i);
      #endif
    }

    // check I2C buttons (override physical buttons)
    if ((currentState.I2CButtonStates & (1<<i)) != (currentState.oldI2CButtonStates & (1<<i))) {
      if (currentState.I2CButtonStates & (1<<i)) handlePress(i); 
      else handleRelease(i); // button i was released
    }
  }
  currentState.oldButtonStates = currentState.buttonStates;
  currentState.oldI2CButtonStates = currentState.I2CButtonStates;

  // Long-press trigger evaluation must run independent of the concrete input path
  // (physical GPIO, I2C buttons, sip/puff state machine, etc.).
  processLongPressTriggers();
  processMultiPressTriggers();

  #ifndef FLIPMOUSE
  // SDA&SCL as GPIOs
  if(currentState.useI2CasGPIO) {
    handleButton(SIP_BUTTON, digitalRead(PIN_WIRE1_SDA_) == LOW ? 1 : 0);
    handleButton(PUFF_BUTTON, digitalRead(PIN_WIRE1_SCL_) == LOW ? 1 : 0);
  }
  #endif


  #ifdef FLIPMOUSE
    // check "long-press" of internal button unpairing all BT hosts
    if (digitalRead(input_map[0]) == LOW) {
      checkPairing++;
      if (checkPairing == 800) {   // approx 4 sec.
        makeTone(TONE_BT_PAIRING, 0);
        unpairAllBT();
        checkPairing = 0;
      }
    } else checkPairing = 0;
  #endif  

  // check sip/puff activities
  if (currentState.pressure > previousPressure) pressureRising = 1; else pressureRising = 0;
  if (currentState.pressure < previousPressure) pressureFalling = 1; else pressureFalling = 0;
  previousPressure = currentState.pressure;

  // handle normal sip and puff actions
  switch (puffState)  {
      case SIP_PUFF_STATE_IDLE:
        if (currentState.pressure > slotSettings.tp)   // handle single puff actions
        {
          makeTone(TONE_INDICATE_PUFF, 0);
          puffState = SIP_PUFF_STATE_STARTED; puffCount = 0;
        }
        break;

      case SIP_PUFF_STATE_STARTED:
        if (!pressureRising)
        {
          if (puffCount++ > SIP_PUFF_SETTLE_TIME)
          {
            puffCount = MIN_HOLD_TIME;
            handlePress(PUFF_BUTTON);
            puffState = SIP_PUFF_STATE_PRESSED;
          }
        } else if (puffCount) puffCount--;
        break;

      case SIP_PUFF_STATE_PRESSED:
        if (puffCount) puffCount--;
        if ((currentState.pressure < slotSettings.tp) && (!puffCount)) {
          handleRelease(PUFF_BUTTON);
          puffState = 0;
        }
    }

    switch (sipState)  {
      case SIP_PUFF_STATE_IDLE:
        if (currentState.pressure < slotSettings.ts)   // handle single sip actions
        {
          makeTone(TONE_INDICATE_SIP, 0);
          sipState = SIP_PUFF_STATE_STARTED; sipCount = 0;
        }
        break;

      case SIP_PUFF_STATE_STARTED:
        if (!pressureFalling)
        {
          if (sipCount++ > SIP_PUFF_SETTLE_TIME)
          {
            sipCount = MIN_HOLD_TIME;
            handlePress(SIP_BUTTON);
            sipState = SIP_PUFF_STATE_PRESSED;
          }
        } else if (sipCount) sipCount--;
        break;

      case SIP_PUFF_STATE_PRESSED:
        if (sipCount) sipCount--;
        if ((currentState.pressure > slotSettings.ts) && (!sipCount)) {
          handleRelease(SIP_BUTTON);
          sipState = 0;
        }
    }

    // handle strong puff actions
    switch (strongPuffState) {
      case SIP_PUFF_STATE_IDLE:
        if (currentState.pressure > slotSettings.sp)
        {
          strongPuffState = SIP_PUFF_STATE_STARTED;
          strongPuffCount = 0;
        }
        break;

      case SIP_PUFF_STATE_STARTED:
        if (!pressureRising)
        {
          if (strongPuffCount++ > SIP_PUFF_SETTLE_TIME)
          {
            strongPuffCount = MIN_HOLD_TIME;
            handlePress(STRONGPUFF_BUTTON);
            strongPuffState = SIP_PUFF_STATE_PRESSED;
          }
        } else if (strongPuffCount) strongPuffCount--;
        break;

      case SIP_PUFF_STATE_PRESSED:
        if (strongPuffCount) strongPuffCount--;
        if ((currentState.pressure < slotSettings.sp) && (!strongPuffCount)) {
          handleRelease(STRONGPUFF_BUTTON);
          strongPuffState = SIP_PUFF_STATE_IDLE;
        }
        break;
    }

    // handle strong sip actions
    switch (strongSipState) {
      case SIP_PUFF_STATE_IDLE:
        if (currentState.pressure < slotSettings.ss)
        {
          strongSipState = SIP_PUFF_STATE_STARTED;
          strongSipCount = 0;
        }
        break;

      case SIP_PUFF_STATE_STARTED:
        if (!pressureFalling)
        {
          if (strongSipCount++ > SIP_PUFF_SETTLE_TIME)
          {
            strongSipCount = MIN_HOLD_TIME;
            handlePress(STRONGSIP_BUTTON);
            strongSipState = SIP_PUFF_STATE_PRESSED;
          }
        } else if (strongSipCount) strongSipCount--;
        break;

      case SIP_PUFF_STATE_PRESSED:
        if (strongSipCount) strongSipCount--;
        if ((currentState.pressure > slotSettings.ss) && (!strongSipCount)) {
          handleRelease(STRONGSIP_BUTTON);
          strongSipState = SIP_PUFF_STATE_IDLE;
        }
        break;
    }

    // now handle stick movements!
    handleMovement();
}

/**
   @name getAccelFactor
   @brief calculates acceleration for mouse pointer movements 
          according to sensordata and acceleration settings 
   @return float value of current acceleration factor
*/
float getAccelFactor() {
  static float accelFactor=0;
  static int xo = 0, yo = 0;
  static float accelMaxForce = 0, lastAngle = 0;

  if (currentState.force == 0) {
    accelFactor = 0;
    accelMaxForce = 0;
    lastAngle = 0;
  }
  else {
    if (currentState.force > accelMaxForce) accelMaxForce = currentState.force;
    if (currentState.force > accelMaxForce * 0.8f) {
      if (accelFactor < 1.0f)
        accelFactor += ((float)slotSettings.ac / 5000000.0f);
    }
    else if (accelMaxForce > 0) accelMaxForce *= 0.99f;

    if (currentState.force < accelMaxForce * 0.6f)  accelFactor *= 0.995f;
    if (currentState.force < accelMaxForce * 0.4f)  accelFactor *= 0.99f;

    float dampingFactor = fabsf(currentState.x - xo) + fabsf(currentState.y - yo);
    accelFactor *= (1.0f - dampingFactor / 1000.0f);
    lastAngle = currentState.angle;
    xo = currentState.x; yo = currentState.y;
  }
  (void)lastAngle; //avoid compiler warnings on unused variable. TODO: necessary value?
  return(accelFactor);
}

/**
   @name acceleratedMouseMove
   @brief performs accelerated mouse pointer movement
   @param accelFactor current acceleration factor
   @return none
*/
void acceleratedMouseMove(float accelFactor) {
  static float accumXpos = 0;
  static float accumYpos = 0;
  float moveValX, moveValY;

  if (getForceSensorType() != FORCE_FABI_GENERIC)
  {
    moveValX = currentState.x * (float)slotSettings.ax * accelFactor;
    moveValY = currentState.y * (float)slotSettings.ay * accelFactor;
  }
  else {
    moveValX = currentState.xRaw * (float)slotSettings.ax / 50;
    moveValY = currentState.yRaw * (float)slotSettings.ay / 50;
  }

  float actSpeed =  sqrtf (moveValX * moveValX + moveValY * moveValY);
  float max_speed = (float)slotSettings.ms / 3.0f;

  if (actSpeed > max_speed) {
    moveValX *= (max_speed / actSpeed);
    moveValY *= (max_speed / actSpeed);
    accelFactor *= 0.98f;
  }

  accumXpos += moveValX;
  accumYpos += moveValY;

  int xMove = (int)accumXpos;
  int yMove = (int)accumYpos;
  
  if ((xMove != 0) || (yMove != 0)) {
    mouseMove(xMove, yMove);
  }

  accumXpos -= xMove;
  accumYpos -= yMove;
}

/**
   @name scaleJoystickAxis
   @brief scales/crops coordinate values to joystick coordinates (0-1023, centered around 512)
   @param val x/y coordinate value to be scaled
   @return integer value for joystick coordinate
*/
int scaleJoystickAxis (float val) {
  int axis = 512 + (int) (val / 50);
  if (axis < 0) axis = 0; 
  else if (axis > 1023) axis = 1023;
  return (axis);
}

/**
   @name handleMovement
   @brief performs an action or movement according to the current sensorData and mode of operation
   @return none
*/
void handleMovement() 
{
  NB_DELAY_START(mouseAxis, MOUSE_MINIMUM_SEND_INTERVAL)
    int moveX=0, moveY=0;
    
    // handle accelerated mouse cursor movement induced by button actions
    if (currentState.autoMoveX == 0) {
      currentState.autoMoveXTimestamp = 0;
    } else {
      if (currentState.autoMoveXTimestamp == 0) 
        currentState.autoMoveXTimestamp = millis();
      else {
        float f= (float)(millis() - currentState.autoMoveXTimestamp) / (float)MOUSE_AUTOMOVE_ACCELERATION_TIME;
        if (f > 1.0f) f = 1.0f;
        moveX=currentState.autoMoveX * f;
      }
    }
    if (currentState.autoMoveY == 0) {
      currentState.autoMoveYTimestamp = 0;
    } else {
      if (currentState.autoMoveYTimestamp == 0) 
        currentState.autoMoveYTimestamp = millis();
      else {
        float f= (float)(millis() - currentState.autoMoveYTimestamp) / (float)MOUSE_AUTOMOVE_ACCELERATION_TIME;
        if (f > 1.0f) f = 1.0f;
        moveY=currentState.autoMoveY * f;
      }
    }
    if ((moveX != 0) || (moveY != 0))  mouseMove(moveX, moveY);  // perform mouse movement
  NB_DELAY_END

  if (globalSettings.thresholdAutoDwell && currentState.mouseMoveTimestamp) {
    if (millis() - currentState.mouseMoveTimestamp >= globalSettings.thresholdAutoDwell) {
      #ifdef DEBUG_OUTPUT
          Serial.println("Autodwell Click");
      #endif
      mousePress(MOUSE_LEFT);
      currentState.clickReleaseTimestamp = millis() + DEFAULT_CLICK_TIME;
      currentState.mouseMoveTimestamp = 0;
    }
  }
  if ((currentState.clickReleaseTimestamp) && (millis() > currentState.clickReleaseTimestamp)) {
    mouseRelease(MOUSE_LEFT);
    currentState.clickReleaseTimestamp = 0;
  }

  #ifdef RP2350
    if (currentState.x || currentState.y) {   // if there is any movement
      userActivity(); // keep system from going into dormant mode (see lpwFuncs.h)
    }
  #endif

  switch (slotSettings.stickMode) {  

    case STICKMODE_MOUSE:   // handle mouse stick mode
      acceleratedMouseMove(getAccelFactor());
      break; 
     
    case STICKMODE_ALTERNATIVE:  // handle alternative actions stick mode
      handleButton(UP_BUTTON,  currentState.y < 0 ? 1 : 0);
      handleButton(DOWN_BUTTON,  currentState.y > 0 ? 1 : 0);
      handleButton(LEFT_BUTTON,  currentState.x < 0 ? 1 : 0);
      handleButton(RIGHT_BUTTON,  currentState.x > 0 ? 1 : 0);
      break;
      
    case STICKMODE_JOYSTICK_1:
      { NB_DELAY_START(gamepadAxis, GAMEPAD_MINIMUM_SEND_INTERVAL)
        joystickAxis(0,scaleJoystickAxis((float)currentState.x * slotSettings.ax));
        joystickAxis(1,scaleJoystickAxis((float)currentState.y * slotSettings.ay));
      NB_DELAY_END }
      break;

    case STICKMODE_JOYSTICK_2:
      { NB_DELAY_START(gamepadAxis, GAMEPAD_MINIMUM_SEND_INTERVAL)
        joystickAxis(2,scaleJoystickAxis((float)currentState.x * slotSettings.ax));
        joystickAxis(3,scaleJoystickAxis((float)currentState.y * slotSettings.ay));
      NB_DELAY_END }
      break;

    case STICKMODE_JOYSTICK_3:
      { NB_DELAY_START(gamepadAxis, GAMEPAD_MINIMUM_SEND_INTERVAL)
        joystickAxis(4,scaleJoystickAxis((float)currentState.x * slotSettings.ax));
        joystickAxis(5,scaleJoystickAxis((float)currentState.y * slotSettings.ay));
      NB_DELAY_END }
      break;
  }
}
