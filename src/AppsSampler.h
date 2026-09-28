#pragma once
#include <Arduino.h>
#include "CalMath.h"

// Reads both APPS exactly like the Pedal Box IO task: same pins, read order, 20 ms period, 5-sample rolling average.
// Every tick is pushed to a queue as a CalMath::Sample.
class AppsSampler {
public:
    static void begin(uint8_t taskPriority);

    static QueueHandle_t queue() { return sampleQueue; }
    static uint32_t droppedSamples() { return dropped; }

private:
    static void taskLoop(void *pvParameters);

    static QueueHandle_t sampleQueue;
    static volatile uint32_t dropped;   // Queue full (consumer busy printing)
};
