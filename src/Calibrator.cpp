#include "Calibrator.h"
#include "AppsSampler.h"
#include "CalMath.h"
#include "Config.h"
#include <stdarg.h>

using namespace CalMath;

namespace {
    enum Step : uint8_t {
        STEP_IDLE,          // Live view, waiting for ENTER
        STEP_REST_WAIT,     // 1/4 Foot off, waiting for ENTER
        STEP_REST,          //     Capturing rest
        STEP_FULL_WAIT,     // 2/4 Pedal on the stop, waiting for ENTER
        STEP_FULL,          //     Capturing full press
        STEP_SWEEP,         // 3/4 Recording press/release sweeps
        STEP_SETTLE,        // 4/4 Foot off, letting the pedal settle
        STEP_REST_AFTER,    //     Capturing rest again (return repeatability)
        STEP_DONE           // Report printed
    };

    enum Verdict : uint8_t { V_PASS, V_MARGINAL, V_FAIL, V_INVALID };

    constexpr uint32_t PERIOD_MS      = Config::Pedalbox::SAMPLE_PERIOD_MS;
    constexpr int      AVG_N          = Config::Pedalbox::AVG_SAMPLES;
    constexpr int      HOLD_SAMPLES   = Config::Cal::HOLD_CAPTURE_MS / PERIOD_MS;
    constexpr int      SETTLE_SAMPLES = Config::Cal::REST_SETTLE_MS / PERIOD_MS;
    constexpr int      SWEEP_MAX      = Config::Cal::SWEEP_MAX_MS / PERIOD_MS;
    constexpr int      LIVE_EVERY     = Config::Ui::LIVE_REFRESH_MS / PERIOD_MS;
    constexpr int      BLINK_EVERY    = 500 / PERIOD_MS;

    const Thresholds CURRENT_CFG = {
        Config::Pedalbox::apps1MinThresh, Config::Pedalbox::apps1MaxThresh,
        Config::Pedalbox::apps2MinThresh, Config::Pedalbox::apps2MaxThresh
    };

    Step step = STEP_IDLE;

    // Hold captures
    CaptureSet cap1, cap2;
    HoldStats *hold1 = nullptr, *hold2 = nullptr;
    int holdCount = 0, settleCount = 0;

    // Sweep recording
    Sample   sweepBuf[SWEEP_MAX];
    int      sweepLen = 0, sweepsDone = 0;
    bool     sweepArmed = false, sweepHigh = false;
    uint32_t dropsAtSweepStart = 0;
    bool     sweepDropped = false;

    // Results
    bool           haveResult = false;
    Recommendation rec;
    TrackReport    curTrack, newTrack;

    // UI
    bool     liveView = true, liveLineOpen = false;
    int      liveTick = 0, blinkTick = 0;
    uint32_t lastEnterMs = 0;

    void enterStep(Step next);
    void finishSweep();

    // ---------- Output helpers ----------

    // Print a message, first ending any live line that's being redrawn with \r
    void say(const char *fmt, ...) {
        if (liveLineOpen) {
            Serial.print("\r\n");
            liveLineOpen = false;
        }
        char buf[200];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        Serial.print(buf);
    }

    void setLed(uint8_t r, uint8_t g, uint8_t b) {
        neopixelWrite(Config::Pins::NEO_ONBOARD, r, g, b);
    }

    void bar(char *out, int pct) {
        int filled = pct * Config::Ui::BAR_WIDTH / 100;
        for (int i = 0; i < Config::Ui::BAR_WIDTH; i++) out[i] = i < filled ? '#' : '.';
        out[Config::Ui::BAR_WIDTH] = '\0';
    }

    int roundi(float v) { return (int)lroundf(v); }

    // Rough % from the rest / full captures, only for sweep counting and the progress bars
    int provisionalPct(float avg, const CaptureSet &c) {
        float span = c.full.mean - c.rest.mean;
        if (span == 0.0f) return 0;
        int p = (int)(100.0f * (avg - c.rest.mean) / span);
        return p < 0 ? 0 : (p > 100 ? 100 : p);
    }

    void printHelp() {
        say("  ENTER / pedal box button = start / next step\n");
        say("  x = abort   l = live view on/off   p = print last report   d = dump last sweep CSV   h = help\n");
    }

    void printLive(const Sample &s) {
        int c1, c2;
        bool cOk = pedalboxPlausible(s.avg1(), s.avg2(), CURRENT_CFG, c1, c2);
        Serial.printf("\r  raw %4d %4d | current cfg %3d%% %3d%% %-4s", roundi(s.avg1()), roundi(s.avg2()), c1, c2, cOk ? "OK" : "IMPL");
        if (haveResult && rec.valid) {
            int n1, n2;
            bool nOk = pedalboxPlausible(s.avg1(), s.avg2(), rec.t, n1, n2);
            Serial.printf(" | new %3d%% %3d%% %-4s", n1, n2, nOk ? "OK" : "IMPL");
        }
        Serial.print("  ");
        liveLineOpen = true;
    }

    void printSweepLive(int p1, int p2) {
        char b1[Config::Ui::BAR_WIDTH + 1], b2[Config::Ui::BAR_WIDTH + 1];
        bar(b1, p1);
        bar(b2, p2);
        Serial.printf("\r  APPS1 [%s] %3d%%   APPS2 [%s] %3d%%   sweeps %d/%d  ",
                      b1, p1, b2, p2, sweepsDone, Config::Cal::SWEEPS_REQUIRED);
        liveLineOpen = true;
    }

    // ---------- Result ----------

    Verdict verdict() {
        if (!rec.valid) return V_INVALID;
        if (newTrack.rangeFaults > 0 || newTrack.overLimit > 0 || newTrack.latchEvents > 0) return V_FAIL;
        if (newTrack.worstGap > Config::Cal::TRACK_WARN_PCT) return V_MARGINAL;
        return V_PASS;
    }

    void showVerdictLed() {
        switch (verdict()) {
            case V_PASS:     setLed(0, 40, 0);  break;
            case V_MARGINAL: setLed(30, 20, 0); break;
            default:         setLed(40, 0, 0);  break;
        }
    }

    // One row of the current-vs-new table
    void row(const char *label, const char *cur, const char *neu) {
        say("  %-40s %-14s %s\n", label, cur, neu);
    }

    void trackRows() {
        const bool n = rec.valid;
        const SensorFit &f1 = rec.s1, &f2 = rec.s2;
        char a[24], b[24];

        row("Pedal Box over your sweeps", "current cfg", n ? "new values" : "");

        snprintf(a, sizeof a, "%d pts @ %d%%", curTrack.worstGap, curTrack.worstAtPct);
        snprintf(b, sizeof b, "%d pts @ %d%%", newTrack.worstGap, newTrack.worstAtPct);
        row("Worst APPS1-APPS2 gap (limit 10)", a, n ? b : "-");

        snprintf(a, sizeof a, "%d", curTrack.overLimit);
        snprintf(b, sizeof b, "%d", newTrack.overLimit);
        row("Samples over the 10-pt limit", a, n ? b : "-");

        snprintf(a, sizeof a, "%d", curTrack.latchEvents);
        snprintf(b, sizeof b, "%d", newTrack.latchEvents);
        row("Implausible >50 ms (ECU cuts torque)", a, n ? b : "-");

        snprintf(a, sizeof a, "%d", curTrack.rangeFaults);
        snprintf(b, sizeof b, "%d", newTrack.rangeFaults);
        row("Range-check faults (+/-200 counts)", a, n ? b : "-");

        snprintf(a, sizeof a, "%d%%", curTrack.lead1);
        snprintf(b, sizeof b, "%d%%", newTrack.lead1);
        row("APPS1 travel before APPS2 moves", a, n ? b : "-");

        snprintf(a, sizeof a, "%d%%", curTrack.lead2);
        snprintf(b, sizeof b, "%d%%", newTrack.lead2);
        row("APPS2 travel before APPS1 moves", a, n ? b : "-");

        snprintf(a, sizeof a, "%d%% / %d%%", pedalboxPercent(f1.restWorst, CURRENT_CFG.min1, CURRENT_CFG.max1),
                                              pedalboxPercent(f2.restWorst, CURRENT_CFG.min2, CURRENT_CFG.max2));
        snprintf(b, sizeof b, "%d%% / %d%%", pedalboxPercent(f1.restWorst, rec.t.min1, rec.t.max1),
                                              pedalboxPercent(f2.restWorst, rec.t.min2, rec.t.max2));
        row("Released pedal reads (APPS1 / APPS2)", a, n ? b : "-");

        snprintf(a, sizeof a, "%d%% / %d%%", pedalboxPercent(f1.fullWorst, CURRENT_CFG.min1, CURRENT_CFG.max1),
                                              pedalboxPercent(f2.fullWorst, CURRENT_CFG.min2, CURRENT_CFG.max2));
        snprintf(b, sizeof b, "%d%% / %d%%", pedalboxPercent(f1.fullWorst, rec.t.min1, rec.t.max1),
                                              pedalboxPercent(f2.fullWorst, rec.t.min2, rec.t.max2));
        row("Pedal on the stop reads (APPS1 / APPS2)", a, n ? b : "-");
    }

    void sensorWarnings(int id, const SensorFit &f) {
        const int rail = Config::Cal::RAIL_MARGIN;
        if (f.rest < rail || f.rest > 4095 - rail)
            say("  - APPS%d rests at %d counts, within %d of the ADC rail: non-linear there, and looks like a wiring fault.\n", id, roundi(f.rest), rail);
        if (f.full < rail || f.full > 4095 - rail)
            say("  - APPS%d reads %d counts on the stop, within %d of the ADC rail: non-linear there, and looks like a wiring fault.\n", id, roundi(f.full), rail);
        if (f.dir > 0)
            say("  - APPS%d RISES when pressed. The Pedal Box range check assumes it falls, so the car will flag it.\n", id);
        if (f.valid && f.restShift > 0.01f * f.span)
            say("  - APPS%d rest moved %d counts between captures: the pedal isn't returning to the same spot (spring / stop).\n", id, roundi(f.restShift));
    }

    void printReport() {
        const SensorFit &f1 = rec.s1, &f2 = rec.s2;
        char a[24], b[24];

        say("\n==================== APPS CALIBRATION RESULT ====================\n");
        snprintf(a, sizeof a, "APPS1 (GPIO%d)", Config::Pins::apps1Pin);
        snprintf(b, sizeof b, "APPS2 (GPIO%d)", Config::Pins::apps2Pin);
        say("  %-26s %-20s %s\n", "", a, b);
        say("  %-26s %-20d %d\n", "Rest (released)", roundi(f1.rest), roundi(f2.rest));
        say("  %-26s %-20d %d\n", "Full (on the stop)", roundi(f1.full), roundi(f2.full));
        say("  %-26s %-20d %d\n", "Travel (counts)", roundi(f1.span), roundi(f2.span));
        snprintf(a, sizeof a, "%.1f / %.1f", f1.sigmaRest, f1.sigmaFull);
        snprintf(b, sizeof b, "%.1f / %.1f", f2.sigmaRest, f2.sigmaFull);
        say("  %-26s %-20s %s\n", "Noise sigma (rest / full)", a, b);
        say("  %-26s %-20d %d\n", "Rest repeatability", roundi(f1.restShift), roundi(f2.restShift));
        say("  %-26s %-20s %s\n", "Direction", f1.dir < 0 ? "falls when pressed" : "RISES when pressed",
                                                f2.dir < 0 ? "falls when pressed" : "RISES when pressed");
        say("  Sweeps: %d, %u.%u s of data\n", sweepsDone, (unsigned)(sweepLen * PERIOD_MS / 1000), (unsigned)(sweepLen * PERIOD_MS % 1000 / 100));

        if (rec.valid) {
            say("\n  Dead band: %.1f%% after rest, %.1f%% before the stop (same %% on both sensors)\n", rec.restDeadPct, rec.fullDeadPct);
            say("\n  Paste into PedalboxRTOS26/src/Config.h, namespace Apps:\n");
            say("        constexpr double apps1MinThresh = %.1f;\n", rec.t.min1);
            say("        constexpr double apps1MaxThresh = %.1f;\n", rec.t.max1);
            say("        constexpr double apps2MinThresh = %.1f;\n", rec.t.min2);
            say("        constexpr double apps2MaxThresh = %.1f;\n", rec.t.max2);
        }

        say("\n");
        trackRows();

        say("\n");
        const Verdict v = verdict();
        switch (v) {
            case V_PASS:
                say("  RESULT: PASS - calibrated, the sensors stay within %d pt%s of each other.\n",
                    newTrack.worstGap, newTrack.worstGap == 1 ? "" : "s");
                if (curTrack.latchEvents > 0)
                    say("  Your current thresholds were the problem (%d-pt gap, %d torque cuts over these sweeps). Flash the new values.\n",
                        curTrack.worstGap, curTrack.latchEvents);
                else if (curTrack.worstGap > Config::Cal::TRACK_WARN_PCT)
                    say("  Your current thresholds were the problem (%d-pt gap, limit 10). Flash the new values.\n", curTrack.worstGap);
                break;
            case V_MARGINAL:
                say("  RESULT: MARGINAL - even calibrated, the sensors drift %d pts apart at %d%% travel (limit 10).\n", newTrack.worstGap, newTrack.worstAtPct);
                say("  Calibration only sets the ends; a gap in the middle is mechanical.\n");
                break;
            case V_FAIL:
                say("  RESULT: FAIL - even calibrated, the Pedal Box would flag this pedal (%d-pt gap, %d range faults).\n",
                    newTrack.worstGap, newTrack.rangeFaults);
                break;
            case V_INVALID: {
                const SensorFit &bad = f1.valid ? f2 : f1;
                say("  RESULT: CANNOT CALIBRATE - APPS%d only moves %d counts (need %d). Check its wiring, supply and mounting.\n",
                    f1.valid ? 2 : 1, roundi(bad.span), Config::Cal::MIN_SPAN_COUNTS);
                break;
            }
        }

        // Mechanical hints: one sensor moving while the other still reads 0 is free play, not calibration
        if (rec.valid && newTrack.lead1 >= Config::Cal::LEAD_WARN_PCT)
            say("  - APPS1 reaches %d%% before APPS2 moves: free play in APPS2's linkage, or APPS2 not turning with the pedal.\n", newTrack.lead1);
        if (rec.valid && newTrack.lead2 >= Config::Cal::LEAD_WARN_PCT)
            say("  - APPS2 reaches %d%% before APPS1 moves: free play in APPS1's linkage, or APPS1 not turning with the pedal.\n", newTrack.lead2);
        sensorWarnings(1, f1);
        sensorWarnings(2, f2);
        if (sweepDropped)
            say("  - Samples were dropped during the sweep, so the timing-based numbers may be slightly off.\n");

        say("=================================================================\n");
    }

    void dumpCsv() {
        say("\nidx,t_ms,raw1,raw2,avg1,avg2,cur_pct1,cur_pct2,cur_ok,new_pct1,new_pct2,new_ok\n");
        for (int i = 0; i < sweepLen; i++) {
            const Sample &s = sweepBuf[i];
            int c1, c2, n1 = 0, n2 = 0;
            bool cOk = pedalboxPlausible(s.avg1(), s.avg2(), CURRENT_CFG, c1, c2);
            bool nOk = rec.valid && pedalboxPlausible(s.avg1(), s.avg2(), rec.t, n1, n2);
            Serial.printf("%d,%u,%u,%u,%.1f,%.1f,%d,%d,%d,%d,%d,%d\n", i, (unsigned)(i * PERIOD_MS),
                          (unsigned)s.raw1, (unsigned)s.raw2, s.avg1(), s.avg2(), c1, c2, cOk, n1, n2, nOk);
        }
        say("end of dump (%d rows)\n", sweepLen);
    }

    // ---------- Steps ----------

    void startHold(HoldStats *h1, HoldStats *h2) {
        hold1 = h1;
        hold2 = h2;
        h1->reset();
        h2->reset();
        holdCount = 0;
    }

    void finishHold() {
        const int wob1 = roundi(hold1->wobble()), wob2 = roundi(hold2->wobble());
        if (wob1 > Config::Cal::MAX_HOLD_WOBBLE || wob2 > Config::Cal::MAX_HOLD_WOBBLE) {
            say("  Signal moved during the capture (APPS1 %d, APPS2 %d counts p-p, max %d). Again (x = abort):\n",
                wob1, wob2, Config::Cal::MAX_HOLD_WOBBLE);
            enterStep(step == STEP_REST ? STEP_REST_WAIT : step == STEP_FULL ? STEP_FULL_WAIT : STEP_SETTLE);
            return;
        }
        say("  APPS1 %4d   APPS2 %4d   (noise sigma %.1f / %.1f counts)\n",
            roundi(hold1->mean), roundi(hold2->mean), hold1->sigma(), hold2->sigma());

        if (step == STEP_REST) {
            enterStep(STEP_FULL_WAIT);
        } else if (step == STEP_FULL) {
            const int d1 = roundi(fabsf(cap1.full.mean - cap1.rest.mean));
            const int d2 = roundi(fabsf(cap2.full.mean - cap2.rest.mean));
            if (d1 < Config::Cal::MIN_SPAN_COUNTS || d2 < Config::Cal::MIN_SPAN_COUNTS) {
                const int bad = d1 < Config::Cal::MIN_SPAN_COUNTS ? 1 : 2;
                say("  APPS%d only moved %d counts from rest (need %d). Is the pedal on the stop? Is APPS%d wired?\n",
                    bad, bad == 1 ? d1 : d2, Config::Cal::MIN_SPAN_COUNTS, bad);
                enterStep(STEP_FULL_WAIT);
                return;
            }
            enterStep(STEP_SWEEP);
        } else {
            rec      = recommend(cap1, cap2);
            curTrack = track(sweepBuf, sweepLen, CURRENT_CFG);
            newTrack = rec.valid ? track(sweepBuf, sweepLen, rec.t) : TrackReport();
            haveResult = true;
            printReport();
            enterStep(STEP_DONE);
        }
    }

    void finishSweep() {
        if (sweepsDone == 0) {
            say("  No full press-and-release recorded. Starting the sweep again.\n");
            enterStep(STEP_SWEEP);
            return;
        }
        sweepDropped = AppsSampler::droppedSamples() != dropsAtSweepStart;
        say("  Recorded %d sweep(s).\n", sweepsDone);
        enterStep(STEP_SETTLE);
    }

    void enterStep(Step next) {
        step = next;
        liveTick = 0;

        switch (next) {
            case STEP_IDLE:
                blinkTick = 0;
                say("\nReady. Press ENTER (or the pedal box button) to start calibrating.\n");
                printHelp();
                break;

            case STEP_REST_WAIT:
                setLed(30, 20, 0);
                say("\nSTEP 1/4  Take your foot OFF the pedal and let it sit on its rest stop.\n");
                say("          Press ENTER to capture.\n");
                break;

            case STEP_REST:
                setLed(0, 0, 40);
                haveResult = false;
                cap1 = CaptureSet();
                cap2 = CaptureSet();
                startHold(&cap1.rest, &cap2.rest);
                say("  Capturing rest for %u s, keep still...\n", (unsigned)(Config::Cal::HOLD_CAPTURE_MS / 1000));
                break;

            case STEP_FULL_WAIT:
                setLed(30, 20, 0);
                say("\nSTEP 2/4  Press the pedal all the way to its stop and HOLD it there.\n");
                say("          Press ENTER while holding.\n");
                break;

            case STEP_FULL:
                setLed(0, 0, 40);
                startHold(&cap1.full, &cap2.full);
                say("  Capturing full press for %u s, keep holding...\n", (unsigned)(Config::Cal::HOLD_CAPTURE_MS / 1000));
                break;

            case STEP_SWEEP:
                setLed(30, 0, 30);
                sweepLen = 0;
                sweepsDone = 0;
                sweepArmed = false;
                sweepHigh = false;
                dropsAtSweepStart = AppsSampler::droppedSamples();
                say("\nSTEP 3/4  Release, then slowly press the pedal to the stop and back %d times\n", Config::Cal::SWEEPS_REQUIRED);
                say("          (about 2 s down, 2 s up). Finishes on its own; ENTER stops early.\n");
                break;

            case STEP_SETTLE:
                setLed(30, 20, 0);
                settleCount = 0;
                say("\nSTEP 4/4  Foot OFF the pedal. Re-checking rest in %u s...\n", (unsigned)(Config::Cal::REST_SETTLE_MS / 1000));
                break;

            case STEP_REST_AFTER:
                setLed(0, 0, 40);
                startHold(&cap1.restAfter, &cap2.restAfter);
                say("  Capturing rest for %u s, keep still...\n", (unsigned)(Config::Cal::HOLD_CAPTURE_MS / 1000));
                break;

            case STEP_DONE:
                showVerdictLed();
                say("\nENTER = calibrate again   l = live view (current vs new)   p = print report   d = dump sweep CSV\n");
                break;
        }
    }

    // ---------- Inputs ----------

    void onSample(const Sample &s) {
        switch (step) {
            case STEP_REST:
            case STEP_FULL:
            case STEP_REST_AFTER:
                // Let the rolling average flush anything from before ENTER
                if (++holdCount <= AVG_N) break;
                hold1->add(s.avg1());
                hold2->add(s.avg2());
                if (holdCount >= AVG_N + HOLD_SAMPLES) finishHold();
                break;

            case STEP_SWEEP: {
                sweepBuf[sweepLen++] = s;

                // Count a sweep once the pedal goes low -> past HIGH -> back under LOW
                int p1 = provisionalPct(s.avg1(), cap1), p2 = provisionalPct(s.avg2(), cap2);
                int p = (p1 + p2) / 2;
                if (!sweepArmed)      { if (p < Config::Cal::SWEEP_LOW_PCT) sweepArmed = true; }
                else if (!sweepHigh)  { if (p > Config::Cal::SWEEP_HIGH_PCT) sweepHigh = true; }
                else if (p < Config::Cal::SWEEP_LOW_PCT) { sweepHigh = false; sweepsDone++; }

                if (++liveTick >= LIVE_EVERY) {
                    liveTick = 0;
                    printSweepLive(p1, p2);
                }
                if (sweepsDone >= Config::Cal::SWEEPS_REQUIRED || sweepLen >= SWEEP_MAX) finishSweep();
                break;
            }

            case STEP_SETTLE:
                if (++settleCount >= SETTLE_SAMPLES) enterStep(STEP_REST_AFTER);
                break;

            default:    // IDLE, waits, DONE
                if (step == STEP_IDLE) {
                    if (blinkTick == 0) setLed(0, 30, 0);
                    else if (blinkTick == BLINK_EVERY) setLed(0, 0, 0);
                    if (++blinkTick >= 2 * BLINK_EVERY) blinkTick = 0;
                }
                if (liveView && ++liveTick >= LIVE_EVERY) {
                    liveTick = 0;
                    printLive(s);
                }
                break;
        }
    }

    void onEnter() {
        switch (step) {
            case STEP_IDLE:
            case STEP_DONE:      enterStep(STEP_REST_WAIT); break;
            case STEP_REST_WAIT: enterStep(STEP_REST);      break;
            case STEP_FULL_WAIT: enterStep(STEP_FULL);      break;
            case STEP_SWEEP:
                if (sweepsDone > 0) finishSweep();
                else say("  Need at least one full press and release first.\n");
                break;
            default: break;
        }
    }

    void onKey(char c) {
        const bool idle = step == STEP_IDLE || step == STEP_DONE;
        switch (c) {
            case 'x':
                if (!idle) {
                    say("  Aborted.\n");
                    enterStep(STEP_IDLE);
                }
                break;
            case 'l':
                liveView = !liveView;
                say("  Live view %s\n", liveView ? "on" : "off");
                break;
            case 'p':
                if (idle && haveResult) printReport();
                break;
            case 'd':
                if (idle && haveResult) dumpCsv();
                break;
            case 'h':
            case '?':
                printHelp();
                break;
        }
    }

    // Debounced falling edge on the driver-ready button (active LOW)
    bool buttonPressed() {
        static uint8_t lowCount = 0;
        static bool    handled = false;
        if (digitalRead(Config::Pins::togglePin) == LOW) {
            if (lowCount < 3) lowCount++;
        } else {
            lowCount = 0;
            handled = false;
        }
        if (lowCount >= 3 && !handled) {
            handled = true;
            return true;
        }
        return false;
    }
}

void Calibrator::begin(uint8_t taskPriority) {
    pinMode(Config::Pins::togglePin, INPUT_PULLUP);
    xTaskCreatePinnedToCore(Calibrator::taskLoop, "CALIBRATOR", 6144, NULL, taskPriority, NULL, 0);
}

void Calibrator::taskLoop(void *pvParameters) {
    say("\n\n=== AppsCalibrator26 ===\n");
    say("Reads APPS1 GPIO%d / APPS2 GPIO%d like PedalboxRTOS26 (%u ms, %d-sample average).\n",
        Config::Pins::apps1Pin, Config::Pins::apps2Pin, (unsigned)PERIOD_MS, AVG_N);
    say("Nothing is sent on CAN. Calibrate with the tractive system OFF (LV only).\n");
    say("Reflash PedalboxRTOS26 when you're done.\n");
    enterStep(STEP_IDLE);

    Sample s;
    for (;;) {
        if (xQueueReceive(AppsSampler::queue(), &s, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (buttonPressed()) onEnter();
            onSample(s);
        }

        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\r' || c == '\n') {
                // Terminals send CR+LF for one ENTER
                if (millis() - lastEnterMs > 100) onEnter();
                lastEnterMs = millis();
            } else {
                onKey((char)tolower(c));
            }
        }
    }
}
