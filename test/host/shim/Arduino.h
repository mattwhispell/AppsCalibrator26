#pragma once
// Minimal Arduino / FreeRTOS stand-ins so Calibrator.cpp compiles on the host (test_wizard.cpp).
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <ctype.h>

#define INPUT        0x01
#define INPUT_PULLUP 0x05
#define LOW          0
#define HIGH         1

typedef void    *QueueHandle_t;
typedef uint32_t TickType_t;
typedef int      BaseType_t;
#define pdTRUE  1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

struct FakeSerial {
    void begin(unsigned long) {}
    void print(const char *s) { fputs(s, stdout); }
    int  printf(const char *fmt, ...) {
        va_list a;
        va_start(a, fmt);
        int n = vprintf(fmt, a);
        va_end(a);
        return n;
    }
    int available() { return 0; }
    int read() { return -1; }
};
extern FakeSerial Serial;

extern uint32_t fakeMillis;
inline uint32_t millis() { return fakeMillis; }
inline void pinMode(uint8_t, uint8_t) {}
inline int  digitalRead(uint8_t) { return HIGH; }
inline void neopixelWrite(uint8_t, uint8_t, uint8_t, uint8_t) {}

inline BaseType_t xQueueReceive(QueueHandle_t, void *, TickType_t) { return pdFALSE; }
inline BaseType_t xTaskCreatePinnedToCore(void (*)(void *), const char *, uint32_t, void *, int, void *, int) { return pdTRUE; }
