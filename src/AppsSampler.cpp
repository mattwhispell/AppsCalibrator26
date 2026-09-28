#include "AppsSampler.h"
#include "Config.h"

QueueHandle_t AppsSampler::sampleQueue = NULL;
volatile uint32_t AppsSampler::dropped = 0;

void AppsSampler::begin(uint8_t taskPriority) {
    // Same pin setup and default ADC config (12-bit, 11 dB) as the Pedal Box
    pinMode(Config::Pins::apps1Pin, INPUT);
    pinMode(Config::Pins::apps2Pin, INPUT);

    sampleQueue = xQueueCreate(64, sizeof(CalMath::Sample));
    xTaskCreatePinnedToCore(AppsSampler::taskLoop, "APPS_SAMPLE", 3072, NULL, taskPriority, NULL, 1);
}

void AppsSampler::taskLoop(void *pvParameters) {
    CalMath::RollingAverage average;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(Config::Pedalbox::SAMPLE_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // Pedal Box order: APPS1 then APPS2
        int raw1 = analogRead(Config::Pins::apps1Pin);
        int raw2 = analogRead(Config::Pins::apps2Pin);

        CalMath::Sample s = average.push(raw1, raw2);
        if (xQueueSend(sampleQueue, &s, 0) != pdTRUE) dropped++;
    }
}
