#ifndef TOF_FILTER_H
#define TOF_FILTER_H

#include <Arduino.h>

class ToFFilter {
  private:
    static const int WINDOW_SIZE = 5; // Must be an odd number
    int rangeWindow[WINDOW_SIZE];
    int windowIndex;
    float smoothedDistance;
    float alpha;

    int getMedian(); // Internal helper function

  public:
    // Constructor: Sets the response speed alpha (0.01 to 1.0)
    ToFFilter(float filterAlpha = 0.2);

    // Processes a new raw value and returns the fully filtered result
    int update(int rawReading);
};

#endif
