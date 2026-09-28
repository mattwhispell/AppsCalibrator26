#pragma once
#include <stdint.h>

// No Arduino includes: CalMath and its host test (test/host) share these constants.
namespace Config {

    // Pin Definitions - must match PedalboxRTOS26/src/Config.h
    namespace Pins {
        constexpr uint8_t apps1Pin          = 7;
        constexpr uint8_t apps2Pin          = 6;
        constexpr uint8_t togglePin         = 21;   // Driver-ready button (active LOW), doubles as ENTER
        constexpr uint8_t NEO_ONBOARD       = 38;
    }

    // How PedalboxRTOS26 turns the APPS signals into a torque request - mirrored so predictions match the car
    namespace Pedalbox {
        constexpr uint32_t SAMPLE_PERIOD_MS = 20;   // IO_LOOP_DELAY_MS
        constexpr int      AVG_SAMPLES      = 5;    // appsNum rolling average
        constexpr int      IMPLAUS_GAP_PCT  = 10;   // |apps1 - apps2| > 10 -> implausible
        constexpr int      RANGE_MARGIN     = 200;  // Averaged raw past max - 200 or min + 200 -> implausible
        constexpr uint32_t ECU_LATCH_MS     = 50;   // ECU APPS_IMPLAUS_GAP_MS: implausible longer than this latches torque off

        // Thresholds currently in PedalboxRTOS26 (used for the before/after comparison)
        constexpr double apps1MinThresh     = 2750.0;
        constexpr double apps1MaxThresh     = 1820.0;
        constexpr double apps2MinThresh     = 2220.0;
        constexpr double apps2MaxThresh     = 1390.0;
    }

    // Calibration procedure and recommendation tuning
    namespace Cal {
        constexpr uint32_t HOLD_CAPTURE_MS   = 3000;   // Rest / full-press capture length
        constexpr uint32_t REST_SETTLE_MS    = 1000;   // Wait after the last sweep before re-capturing rest
        constexpr uint32_t SWEEP_MAX_MS      = 60000;  // Sweep buffer length (3000 samples, 24 KB)
        constexpr int      SWEEPS_REQUIRED   = 3;
        constexpr int      SWEEP_HIGH_PCT    = 85;     // A sweep counts once the pedal passes this...
        constexpr int      SWEEP_LOW_PCT     = 10;     // ...and comes back under this

        constexpr float    REST_DEADBAND_PCT = 2.0f;   // Minimum dead band after rest (% of travel)
        constexpr float    FULL_DEADBAND_PCT = 3.0f;   // Minimum margin before the stop so 100% is always reachable
        constexpr float    NOISE_SIGMA_K     = 4.0f;   // Dead bands also swallow this many sigma of averaged noise
        constexpr int      MIN_SPAN_COUNTS   = 300;    // Less travel than this = sensor / wiring / mounting fault
        constexpr int      MAX_HOLD_WOBBLE   = 80;     // Averaged p-p counts allowed during a hold capture
        constexpr int      RAIL_MARGIN       = 100;    // ESP32 ADC is non-linear near 0 and 4095
        constexpr float    TRACK_WARN_PCT    = 5.0f;   // Calibrated gap above this = marginal
        constexpr int      LEAD_WARN_PCT     = 3;      // One sensor this far along while the other reads 0 = free play
    }

    namespace Ui {
        constexpr uint32_t LIVE_REFRESH_MS   = 100;
        constexpr int      BAR_WIDTH         = 20;
    }
}
