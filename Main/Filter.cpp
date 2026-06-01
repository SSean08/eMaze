#include "ToFFilter.h"

ToFFilter::ToFFilter(float filterAlpha) {
  alpha = filterAlpha;
  windowIndex = 0;
  smoothedDistance = -1.0; // Flag to indicate uninitialized data
  memset(rangeWindow, 0, sizeof(rangeWindow));
}

int ToFFilter::getMedian() {
  int tempArray[WINDOW_SIZE];
  memcpy(tempArray, rangeWindow, WINDOW_SIZE * sizeof(int));

  // Simple bubble sort
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    for (int j = 0; j < WINDOW_SIZE - i - 1; j++) {
      if (tempArray[j] > tempArray[j + 1]) {
        int temp = tempArray[j];
        tempArray[j] = tempArray[j + 1];
        tempArray[j + 1] = temp;
      }
    }
  }
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

  // 2. Extract the median to drop single-sample spikes
  int medianDistance = getMedian();

  // 3. Apply Exponential Moving Average (EMA) to smooth out continuous jitter
  if (smoothedDistance < 0) {
    smoothedDistance = medianDistance; // Seed the initial value
  } else {
    smoothedDistance = (alpha * medianDistance) + ((1.0 - alpha) * smoothedDistance);
  }

  return (int)smoothedDistance;
}
