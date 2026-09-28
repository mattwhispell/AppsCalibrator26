#include <Arduino.h>
#include "AppsSampler.h"
#include "Calibrator.h"

// APPS calibration firmware for the Pedal Box board.
// Sends nothing on CAN - reflash PedalboxRTOS26 when done.

void setup() {
    Serial.begin(115200);

    // Start tasks at (Priority)
    AppsSampler::begin(3);      // Same priority as the Pedal Box IO task
    Calibrator::begin(2);
}

void loop() {
    vTaskDelete(NULL);
}
