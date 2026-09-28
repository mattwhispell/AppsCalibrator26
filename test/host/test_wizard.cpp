// Host test for the Calibrator wizard: drives the real state machine with a simulated pedal.
// Build + run (from AppsCalibrator26/):
//   g++ -std=c++11 -Wall -Itest/host/shim -Isrc test/host/test_wizard.cpp src/CalMath.cpp -o test_wizard && ./test_wizard
#include <random>
#include "../../src/Calibrator.cpp"     // Same TU so the wizard's internals are reachable

FakeSerial Serial;
uint32_t fakeMillis = 1000;
QueueHandle_t AppsSampler::sampleQueue = nullptr;
volatile uint32_t AppsSampler::dropped = 0;

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { failures++; printf("\n  FAIL: %s  [%s]\n", msg, #cond); } } while (0)

struct Sensor { double rest, full, lashPct, noise; };

struct Pedal {
    Sensor m1, m2;
    std::mt19937 rng{99};
    RollingAverage avg;
    double pos = 0;

    int read(const Sensor &m) {
        std::normal_distribution<double> n(0.0, m.noise);
        double eff = pos <= m.lashPct ? 0.0 : (pos - m.lashPct) / (100.0 - m.lashPct);
        return (int)lround(m.rest + eff * (m.full - m.rest) + n(rng));
    }
    // Advance time, feeding the wizard one 20 ms sample at a time
    void hold(double pct, int ms) { ramp(pct, pct, ms); }
    void ramp(double from, double to, int ms) {
        int n = ms / (int)PERIOD_MS;
        for (int i = 1; i <= n; i++) {
            pos = from + (to - from) * i / n;
            fakeMillis += PERIOD_MS;
            onSample(avg.push(read(m1), read(m2)));
        }
    }
};

static void runWizard(Pedal &p, bool wobbleDuringRest) {
    enterStep(STEP_IDLE);
    p.hold(0, 500);
    onEnter();                              // -> STEP 1 prompt
    p.hold(0, 300);
    onEnter();                              // -> capture rest
    if (wobbleDuringRest) {
        p.ramp(0, 20, 400);                 // Foot bumps the pedal mid-capture
        p.ramp(20, 0, 400);
        p.hold(0, 3000);
        CHECK(step == STEP_REST_WAIT, "wobble sends you back to STEP 1");
        onEnter();
    }
    p.hold(0, 3300);
    CHECK(step == STEP_FULL_WAIT, "rest capture -> STEP 2");
    p.ramp(0, 100, 800);
    p.hold(100, 300);
    onEnter();                              // -> capture full
    p.hold(100, 3300);
    CHECK(step == STEP_SWEEP, "full capture -> STEP 3");
    p.ramp(100, 0, 1000);
    for (int i = 0; i < Config::Cal::SWEEPS_REQUIRED && step == STEP_SWEEP; i++) {
        p.hold(0, 500);
        p.ramp(0, 100, 2000);
        p.hold(100, 300);
        p.ramp(100, 0, 2000);
    }
    CHECK(step == STEP_SETTLE, "3 sweeps -> STEP 4");
    p.hold(0, 5000);
    CHECK(step == STEP_DONE && haveResult, "rest re-capture -> report");
}

int main() {
    printf("########## A: data 35 miscalibration, plus a bumped rest capture ##########\n");
    {
        Pedal p;
        p.m1 = { 2752, 1815, 0, 3 };
        p.m2 = { 2296, 1380, 0, 3 };
        runWizard(p, true);
        CHECK(verdict() == V_PASS, "A passes once calibrated");
        CHECK(sweepsDone == 3, "A counted 3 sweeps");
    }

    printf("\n########## B: 10%% free play on APPS2 ##########\n");
    {
        Pedal p;
        p.m1 = { 2752, 1815, 0, 3 };
        p.m2 = { 2296, 1380, 10, 3 };
        runWizard(p, false);
        CHECK(verdict() == V_MARGINAL || verdict() == V_FAIL, "B is flagged mechanical");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
