#pragma once
#include <stdint.h>
#include "Config.h"

// Pure calibration math - no Arduino dependencies so it can be tested on the host (test/host).
namespace CalMath {

    // One Pedal Box IO tick: the raw reads plus the rolling sums the Pedal Box would hold at that moment
    struct Sample {
        uint16_t raw1, raw2;
        uint16_t sum1, sum2;
        float avg1() const { return (float)sum1 / Config::Pedalbox::AVG_SAMPLES; }
        float avg2() const { return (float)sum2 / Config::Pedalbox::AVG_SAMPLES; }
    };

    // Same rolling average as PedalboxRTOS26 IoManager::appsHandler (buffer starts zeroed)
    class RollingAverage {
    public:
        Sample push(int raw1, int raw2);
    private:
        int buf1[Config::Pedalbox::AVG_SAMPLES] = {0};
        int buf2[Config::Pedalbox::AVG_SAMPLES] = {0};
        int idx = 0;
    };

    struct Thresholds {
        double min1, max1, min2, max2;   // Pedal Box naming: min = raw at 0%, max = raw at 100%
    };

    // Running stats of the averaged signal during a hold (Welford)
    struct HoldStats {
        int    n    = 0;
        double mean = 0, m2 = 0;
        float  lo   = 0, hi = 0;

        void  reset() { *this = HoldStats(); }
        void  add(float v);
        float sigma() const;
        float wobble() const { return n ? hi - lo : 0; }
    };

    // Everything captured for one sensor
    struct CaptureSet {
        HoldStats rest;         // Foot off, before the sweeps
        HoldStats full;         // Held on the stop
        HoldStats restAfter;    // Foot off, after the sweeps (return repeatability)
    };

    struct SensorFit {
        bool  valid;            // Enough travel to calibrate
        int   dir;              // -1 raw falls when pressed (current wiring), +1 rises
        float rest;             // Rest position used: the rest capture furthest toward "pressed"
        float full;             // Full-press hold mean
        float span;             // |full - rest|
        float restShift;        // |rest - restAfter|
        float restWorst;        // Most-pressed averaged value seen while released
        float fullWorst;        // Least-pressed averaged value seen while on the stop
        float sigmaRest, sigmaFull;
        float restNoise;        // Counts the rest dead band has to swallow
        float fullNoise;        // Counts the full margin has to swallow
    };

    struct Recommendation {
        bool       valid;
        SensorFit  s1, s2;
        float      restDeadPct;     // Applied to both sensors so they leave 0% together
        float      fullDeadPct;     // Applied to both sensors so they reach 100% together
        Thresholds t;
    };

    // What the Pedal Box + ECU would have done over a recorded sweep
    struct TrackReport {
        int n;
        int worstGap;           // Max |apps1 - apps2| in percentage points
        int worstGapSigned;     // apps1 - apps2 at that point (+ = APPS1 ahead)
        int worstAtPct;         // Pedal travel where it happened
        int overLimit;          // Samples over the 10-pt limit
        int rangeFaults;        // Samples failing the +/-200 raw range check
        int latchEvents;        // Implausible runs longer than ECU_LATCH_MS (ECU zeroes torque until APPS < 5%)
        int longestRunMs;
        int lead1;              // Max APPS1 % while APPS2 still reads 0
        int lead2;              // Max APPS2 % while APPS1 still reads 0
    };

    // Exact Pedal Box conversion, including its float/double mix, truncation and clamping
    int  pedalboxPercent(float avg, double minT, double maxT);
    bool pedalboxInRange(float avg1, float avg2, const Thresholds &t);
    bool pedalboxPlausible(float avg1, float avg2, const Thresholds &t, int &p1, int &p2);

    SensorFit      fitSensor(const CaptureSet &c);
    Recommendation recommend(const CaptureSet &c1, const CaptureSet &c2);
    TrackReport    track(const Sample *s, int n, const Thresholds &t);
}
