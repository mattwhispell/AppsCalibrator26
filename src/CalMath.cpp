#include "CalMath.h"
#include <math.h>
#include <stdlib.h>

namespace CalMath {

Sample RollingAverage::push(int raw1, int raw2) {
    buf1[idx] = raw1;
    buf2[idx] = raw2;
    if (++idx >= Config::Pedalbox::AVG_SAMPLES) idx = 0;

    int sum1 = 0, sum2 = 0;
    for (int i = 0; i < Config::Pedalbox::AVG_SAMPLES; i++) {
        sum1 += buf1[i];
        sum2 += buf2[i];
    }
    return Sample{ (uint16_t)raw1, (uint16_t)raw2, (uint16_t)sum1, (uint16_t)sum2 };
}

void HoldStats::add(float v) {
    n++;
    double delta = v - mean;
    mean += delta / n;
    m2 += delta * (v - mean);
    if (n == 1 || v < lo) lo = v;
    if (n == 1 || v > hi) hi = v;
}

float HoldStats::sigma() const {
    return n > 1 ? (float)sqrt(m2 / (n - 1)) : 0.0f;
}

int pedalboxPercent(float avg, double minT, double maxT) {
    // Same types as IoManager::appsHandler: double thresholds, float average, float range
    float range = minT - maxT;
    if (range == 0.0f) return 0;

    int p = (int)(((minT - avg) / range) * 100.0f);
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return p;
}

bool pedalboxInRange(float avg1, float avg2, const Thresholds &t) {
    // Written for sensors that fall when pressed, exactly like the Pedal Box
    const int m = Config::Pedalbox::RANGE_MARGIN;
    return !(avg1 < t.max1 - m || avg2 < t.max2 - m ||
             avg1 > t.min1 + m || avg2 > t.min2 + m);
}

bool pedalboxPlausible(float avg1, float avg2, const Thresholds &t, int &p1, int &p2) {
    p1 = pedalboxPercent(avg1, t.min1, t.max1);
    p2 = pedalboxPercent(avg2, t.min2, t.max2);
    return abs(p1 - p2) <= Config::Pedalbox::IMPLAUS_GAP_PCT && pedalboxInRange(avg1, avg2, t);
}

SensorFit fitSensor(const CaptureSet &c) {
    SensorFit f = {};
    const bool haveAfter = c.restAfter.n > 0;

    f.dir = (c.full.mean < c.rest.mean) ? -1 : 1;

    // Use the rest capture sitting furthest toward "pressed" so the pedal reads 0% at either
    float r1 = c.rest.mean;
    float r2 = haveAfter ? (float)c.restAfter.mean : r1;
    f.rest      = (f.dir < 0) ? fminf(r1, r2) : fmaxf(r1, r2);
    f.restShift = fabsf(r1 - r2);
    f.full      = c.full.mean;
    f.span      = fabsf(f.full - f.rest);

    // Worst excursions actually seen
    f.restWorst = (f.dir < 0) ? c.rest.lo : c.rest.hi;
    if (haveAfter) f.restWorst = (f.dir < 0) ? fminf(f.restWorst, c.restAfter.lo) : fmaxf(f.restWorst, c.restAfter.hi);
    f.fullWorst = (f.dir < 0) ? c.full.hi : c.full.lo;

    f.sigmaRest = fmaxf(c.rest.sigma(), haveAfter ? c.restAfter.sigma() : 0.0f);
    f.sigmaFull = c.full.sigma();

    // Dead bands cover K sigma of noise or the worst excursion seen, whichever is bigger
    float restExcursion = fmaxf(0.0f, f.dir * (f.restWorst - f.rest));
    float fullShortfall = fmaxf(0.0f, f.dir * (f.full - f.fullWorst));
    f.restNoise = fmaxf(Config::Cal::NOISE_SIGMA_K * f.sigmaRest, restExcursion);
    f.fullNoise = fmaxf(Config::Cal::NOISE_SIGMA_K * f.sigmaFull, fullShortfall);

    f.valid = f.span >= Config::Cal::MIN_SPAN_COUNTS;
    return f;
}

// 0% point: dead band past rest, rounded toward "pressed" so it's never smaller than asked
static double zeroPoint(const SensorFit &f, float pct) {
    double v = f.rest + f.dir * (pct / 100.0) * f.span;
    return f.dir < 0 ? floor(v) : ceil(v);
}

// 100% point: margin before the stop, rounded toward rest so 100% stays reachable
static double fullPoint(const SensorFit &f, float pct) {
    double v = f.full - f.dir * (pct / 100.0) * f.span;
    return f.dir < 0 ? ceil(v) : floor(v);
}

Recommendation recommend(const CaptureSet &c1, const CaptureSet &c2) {
    Recommendation r = {};
    r.s1 = fitSensor(c1);
    r.s2 = fitSensor(c2);
    r.valid = r.s1.valid && r.s2.valid;
    if (!r.valid) return r;

    // Same % on both sensors so they leave 0% and reach 100% at the same pedal position
    r.restDeadPct = fmaxf(Config::Cal::REST_DEADBAND_PCT,
                          fmaxf(100.0f * r.s1.restNoise / r.s1.span, 100.0f * r.s2.restNoise / r.s2.span));
    r.fullDeadPct = fmaxf(Config::Cal::FULL_DEADBAND_PCT,
                          fmaxf(100.0f * r.s1.fullNoise / r.s1.span, 100.0f * r.s2.fullNoise / r.s2.span));

    r.t.min1 = zeroPoint(r.s1, r.restDeadPct);
    r.t.max1 = fullPoint(r.s1, r.fullDeadPct);
    r.t.min2 = zeroPoint(r.s2, r.restDeadPct);
    r.t.max2 = fullPoint(r.s2, r.fullDeadPct);
    return r;
}

TrackReport track(const Sample *s, int n, const Thresholds &t) {
    TrackReport r = {};
    r.n = n;
    int  run = 0;           // Consecutive implausible samples
    bool latched = false;   // Already counted this run as an ECU latch

    for (int i = 0; i < n; i++) {
        float a1 = s[i].avg1(), a2 = s[i].avg2();
        int p1, p2;
        bool ok = pedalboxPlausible(a1, a2, t, p1, p2);

        int gap = p1 - p2;
        if (abs(gap) > r.worstGap) {
            r.worstGap       = abs(gap);
            r.worstGapSigned = gap;
            r.worstAtPct     = (p1 + p2) / 2;
        }
        if (abs(gap) > Config::Pedalbox::IMPLAUS_GAP_PCT) r.overLimit++;
        if (!pedalboxInRange(a1, a2, t)) r.rangeFaults++;

        // ECU latches once the time since the last plausible frame passes ECU_LATCH_MS
        if (!ok) {
            run++;
            int ms = run * (int)Config::Pedalbox::SAMPLE_PERIOD_MS;
            if (ms > r.longestRunMs) r.longestRunMs = ms;
            if (!latched && ms > (int)Config::Pedalbox::ECU_LATCH_MS) {
                r.latchEvents++;
                latched = true;
            }
        } else {
            run = 0;
            latched = false;
        }

        if (p2 == 0 && p1 > r.lead1) r.lead1 = p1;
        if (p1 == 0 && p2 > r.lead2) r.lead2 = p2;
    }
    return r;
}

}
