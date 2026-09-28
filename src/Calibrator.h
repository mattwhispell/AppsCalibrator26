#pragma once
#include <Arduino.h>

// Serial-driven calibration wizard: rest -> full press -> sweeps -> rest again, then prints
// recommended Pedal Box thresholds and what the car would have done with the current vs new values.
class Calibrator {
public:
    static void begin(uint8_t taskPriority);

private:
    static void taskLoop(void *pvParameters);
};
