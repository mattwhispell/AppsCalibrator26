// Host test for CalMath - runs the calibration procedure against simulated pedals.
// Build + run (from AppsCalibrator26/):
//   g++ -std=c++11 -Wall -Isrc test/host/test_calmath.cpp src/CalMath.cpp -o test_calmath && ./test_calmath
#include <stdio.h>
#include <math.h>
#include <random>
#include <vector>
#include "CalMath.h"

using namespace CalMath;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL: "); printf(__VA_ARGS__); printf("  [%s]\n", #cond); } } while (0)

// One sensor: raw at rest / on the stop, free play (% of pedal before it moves), Gaussian noise on each read
struct SensorModel {
    double rest, full, lashPct, noise;
};

struct PedalSim {
    SensorModel m1, m2;
    std::mt19937 rng{1234};
    RollingAverage avg;

    int read(const SensorModel &m, double pedalPct) {
        std::normal_distribution<double> n(0.0, m.noise);
        double eff = pedalPct <= m.lashPct ? 0.0 : (pedalPct - m.lashPct) / (100.0 - m.lashPct);
        double v = m.rest + eff * (m.full - m.rest) + n(rng);
        return (int)lround(fmin(4095.0, fmax(0.0, v)));
    }
    Sample tick(double pedalPct) { return avg.push(read(m1, pedalPct), read(m2, pedalPct)); }

    // Mirrors Calibrator: skip AVG_SAMPLES to flush the rolling average, then capture
    void hold(double pedalPct, HoldStats &h1, HoldStats &h2) {
        const int samples = Config::Cal::HOLD_CAPTURE_MS / Config::Pedalbox::SAMPLE_PERIOD_MS;
        for (int i = 0; i < Config::Pedalbox::AVG_SAMPLES; i++) tick(pedalPct);
        for (int i = 0; i < samples; i++) {
            Sample s = tick(pedalPct);
            h1.add(s.avg1());
            h2.add(s.avg2());
        }
    }
    void ramp(double from, double to, int ms, std::vector<Sample> *rec) {
        int n = ms / Config::Pedalbox::SAMPLE_PERIOD_MS;
        for (int i = 1; i <= n; i++) {
            Sample s = tick(from + (to - from) * i / n);
            if (rec) rec->push_back(s);
        }
    }
};

struct Result {
    Recommendation rec;
    TrackReport cur, neu;
};

static Result runProcedure(PedalSim &p) {
    CaptureSet c1, c2;
    std::vector<Sample> sweep;
    p.ramp(0, 0, 1000, nullptr);                        // Idle at rest
    p.hold(0, c1.rest, c2.rest);                        // STEP 1
    p.ramp(0, 100, 800, nullptr);
    p.hold(100, c1.full, c2.full);                      // STEP 2
    p.ramp(100, 0, 1000, &sweep);                       // STEP 3: release, then 3 sweeps
    for (int i = 0; i < Config::Cal::SWEEPS_REQUIRED; i++) {
        p.ramp(0, 0, 500, &sweep);
        p.ramp(0, 100, 2000, &sweep);
        p.ramp(100, 100, 300, &sweep);
        p.ramp(100, 0, 2000, &sweep);
    }
    p.ramp(0, 0, Config::Cal::REST_SETTLE_MS, nullptr); // STEP 4
    p.hold(0, c1.restAfter, c2.restAfter);

    const Thresholds current = { Config::Pedalbox::apps1MinThresh, Config::Pedalbox::apps1MaxThresh,
                                 Config::Pedalbox::apps2MinThresh, Config::Pedalbox::apps2MaxThresh };
    Result r;
    r.rec = recommend(c1, c2);
    r.cur = track(sweep.data(), (int)sweep.size(), current);
    r.neu = r.rec.valid ? track(sweep.data(), (int)sweep.size(), r.rec.t) : TrackReport();
    return r;
}

static void print(const char *name, const Result &r) {
    printf("\n%s\n", name);
    printf("  recommend: valid=%d dead %.1f%%/%.1f%%  min1 %.0f max1 %.0f  min2 %.0f max2 %.0f\n",
           r.rec.valid, r.rec.restDeadPct, r.rec.fullDeadPct, r.rec.t.min1, r.rec.t.max1, r.rec.t.min2, r.rec.t.max2);
    const TrackReport *t[2] = { &r.cur, &r.neu };
    const char *label[2] = { "current", "new    " };
    for (int i = 0; i < 2; i++)
        printf("  %s: worst %2d pts @ %2d%%  over %3d  latches %d  range %3d  lead1 %2d  lead2 %2d\n", label[i],
               t[i]->worstGap, t[i]->worstAtPct, t[i]->overLimit, t[i]->latchEvents, t[i]->rangeFaults, t[i]->lead1, t[i]->lead2);
}

int main() {
    // Pedal Box conversion matches IoManager::appsHandler
    CHECK(pedalboxPercent(2750, 2750, 1820) == 0,   "0%% at min");
    CHECK(pedalboxPercent(1820, 2750, 1820) == 100, "100%% at max");
    CHECK(pedalboxPercent(2285, 2750, 1820) == 50,  "50%% midpoint");
    CHECK(pedalboxPercent(2900, 2750, 1820) == 0,   "clamped low");
    CHECK(pedalboxPercent(1500, 2750, 1820) == 100, "clamped high");
    CHECK(pedalboxPercent(2741, 2750, 1820) == 0,   "truncates 0.97%% to 0");

    // A: data 35 - APPS2 rests ~76 counts past apps2MinThresh (~9% dead zone), sensors themselves linear
    {
        PedalSim p;
        p.m1 = { 2752, 1815, 0, 3 };
        p.m2 = { 2296, 1380, 0, 3 };
        Result r = runProcedure(p);
        print("A: miscalibrated APPS2 rest (data 35)", r);
        CHECK(r.rec.valid, "A valid");
        CHECK(r.cur.worstGap >= 8, "A current config shows the ~9 pt gap");
        CHECK(r.cur.lead1 >= 7, "A current config: APPS1 leads while APPS2 reads 0");
        CHECK(r.neu.worstGap <= 3, "A calibrated gap small");
        CHECK(r.neu.overLimit == 0 && r.neu.latchEvents == 0 && r.neu.rangeFaults == 0, "A calibrated has no faults");
        CHECK(fabs(r.rec.t.min2 - (2296 - 0.02 * 916)) < 3, "A min2 ~2%% past rest");
        CHECK(fabs(r.rec.t.max1 - (1815 + 0.03 * 937)) < 3, "A max1 ~3%% before stop");
    }

    // B: free play - APPS2 doesn't move for the first 10% of pedal. Calibration can't fix it.
    {
        PedalSim p;
        p.m1 = { 2752, 1815, 0, 3 };
        p.m2 = { 2296, 1380, 10, 3 };
        Result r = runProcedure(p);
        print("B: 10% free play on APPS2", r);
        CHECK(r.rec.valid, "B valid");
        CHECK(r.neu.worstGap >= 8, "B gap survives calibration");
        CHECK(r.neu.lead1 >= 7, "B lead1 flags free play");
    }

    // C: APPS2 wired so it rises when pressed - Pedal Box range check would reject it
    {
        PedalSim p;
        p.m1 = { 2752, 1815, 0, 3 };
        p.m2 = { 1000, 1900, 0, 3 };
        Result r = runProcedure(p);
        print("C: APPS2 rises when pressed", r);
        CHECK(r.rec.s2.dir == 1, "C direction detected");
        CHECK(r.neu.worstGap <= 3, "C percentages still track");
        CHECK(r.neu.rangeFaults > 0, "C range check faults");
    }

    // D: noisy APPS1 - dead band grows past the 2% minimum
    {
        PedalSim p;
        p.m1 = { 2752, 1815, 0, 25 };
        p.m2 = { 2296, 1380, 0, 3 };
        Result r = runProcedure(p);
        print("D: noisy APPS1", r);
        CHECK(r.rec.restDeadPct > Config::Cal::REST_DEADBAND_PCT, "D dead band widened for noise");
    }

    // E: APPS1 barely moves - cannot calibrate
    {
        PedalSim p;
        p.m1 = { 2752, 2600, 0, 3 };
        p.m2 = { 2296, 1380, 0, 3 };
        Result r = runProcedure(p);
        print("E: APPS1 barely moves", r);
        CHECK(!r.rec.valid && !r.rec.s1.valid && r.rec.s2.valid, "E flags APPS1");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
