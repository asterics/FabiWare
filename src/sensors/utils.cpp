#include "utils.h"
#include "sensors.h"
#include <Arduino.h>
#include <string.h>

static void swap(int *a, int *b) {
  int temp = *a;
  *a = *b;
  *b = temp;
}

static void quickSort(int values[], int left, int right) {
  if (left >= right) {
    return;
  }

  int pivot = values[left];
  int i = left + 1;
  int j = right;

  while (1) {
    while (i <= j && values[i] <= pivot) i++;
    while (j >= i && values[j] >= pivot) j--;
    if (i > j)
      break;

    swap(&values[i], &values[j]);
  }
  swap(&values[left], &values[j]);

  quickSort(values, left, j - 1);
  quickSort(values, j + 1, right);
}

int calculateMedian(int value, MedianState *state) {
  if (!state || !state->buffer || state->bufsize <= 0) return value;
  int n = state->bufsize;
  int *buf = state->buffer;
  int pos = state->pos;

  buf[pos] = value;
  pos++;
  if (pos >= n) pos = 0;
  state->pos = pos;

  // copy into temp array for sorting
  int *values = (int*)alloca(sizeof(int) * n);
  memcpy(values, buf, sizeof(int) * n);

  quickSort(values, 0, n - 1);

  if (n % 2 == 0) {
    int midIndex1 = n / 2 - 1;
    int midIndex2 = n / 2;
    return (values[midIndex1] + values[midIndex2]) / 2;
  } else {
    int midIndex = n / 2;
    return values[midIndex];
  }
}


int isAnalogPinFloating (int pin) {
  int x,y;
  pinMode(pin, INPUT_PULLUP);
  analogRead (pin);
  x= adc_read();
  
  pinMode(pin, INPUT_PULLDOWN);
  analogRead (pin);
  y= adc_read();

  #ifdef DEBUG_OUTPUT_SENSORS
    Serial.printf("Testing Pin Float: %d, %d\n",x,y);
  #endif

  if (x-y < PINFLOAT_DIFFERENCE_THRESHOLD) return(false);
  return(true);
}
