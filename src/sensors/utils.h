#pragma once
#include <Arduino.h>

// Versatile median filter state
struct MedianState {
  int *buffer;   // pointer to circular buffer of length bufsize
  int bufsize;   // number of elements in buffer
  int pos;       // current write position [0..bufsize-1]
};

// New API: calculate median with external state
int calculateMedian(int value, MedianState *state);

// Check if an ADC pin is floating
int isAnalogPinFloating (int pin);

