#include "ToFFilter.h"

ToFFilter::ToFFilter(float filterAlpha) {
  alpha = filterAlpha;
  windowIndex = 0;
  smoothedDistance = -1.0; // Flag to indicate uninitialized data
  memset(rangeWindow, 0, sizeof(rangeWindow));
}

void ToFFilter::quickSort(int arr[], int left, int right) {
  int i = left, j = right;
  int pivot = arr[(left + right) / 2];

  while (i <= j) {
    while (arr[i] < pivot) i++;
    while (arr[j] > pivot) j--;
    if (i <= j) {
      int tmp = arr[i];
      arr[i] = arr[j];
      arr[j] = tmp;
      i++;
      j--;
    }
  }

  if (left < j)  quickSort(arr, left, j);
  if (i < right) quickSort(arr, i, right);
}

int ToFFilter::getMedian() {
  int tempArray[WINDOW_SIZE];
  memcpy(tempArray, rangeWindow, WINDOW_SIZE * sizeof(int));

  // Execute quicksort on the temporary array copy
  quickSort(tempArray, 0, WINDOW_SIZE - 1);
  
  return tempArray[WINDOW_SIZE / 2];
}

int ToFFilter::update(int rawReading) {
  // Ignore sensor hardware error code spikes (typically 8190+)
  if (rawReading >= 8190) {
    return (smoothedDistance < 0) ? 0 : (int)smoothedDistance;
  }

  // 1. Update the moving window array
  rangeWindow[windowIndex] = rawReading;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE;

  // 2. Extract the median using Quicksort to drop spikes
  int medianDistance = getMedian();

  // 3. Apply Exponential Moving Average (EMA) to smooth out continuous jitter
  if (smoothedDistance < 0) {
    smoothedDistance = medianDistance; // Seed the initial value
  } else {
    smoothedDistance = (alpha * medianDistance) + ((1.0 - alpha) * smoothedDistance);
  }

  return (int)smoothedDistance;
}
