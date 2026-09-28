# AppsCalibrator26

Standalone firmware for the Pedal Box board that calibrates the two APPS. You sweep the pedal, and it prints
the `apps1/2 Min/MaxThresh` values to paste into `PedalboxRTOS26/src/Config.h`, plus a before/after of what
the car would have done over your sweeps.

It reads the pedal exactly like PedalboxRTOS26: same pins (APPS1 GPIO7, APPS2 GPIO6), read order,
20 ms period, 5-sample rolling average, % conversion, 10-pt plausibility check, ±200-count range check. It also
models the ECU's 50 ms implausibility latch. The predictions are what the car will actually do.

**Safety:** this firmware sends nothing on CAN. Flash and calibrate with the tractive system OFF (LV only), then
reflash PedalboxRTOS26.

## Use

```
pio run -t upload
pio device monitor
```

The wizard runs on the serial monitor. ENTER (or the pedal box driver-ready button) advances:

1. **Rest:** foot off, 3 s capture.
2. **Full:** hold the pedal on its stop, 3 s capture.
3. **Sweeps:** slowly press to the stop and back 3 times (~2 s each way). Live bars show both sensors.
4. **Rest again:** automatic, 3 s capture. This checks the pedal returns to the same spot.

Then it prints the report. Keys: `x` abort, `l` live view (after a run this shows current vs new % side by side,
so you can press the pedal and watch them track), `p` reprint report, `d` dump the sweep as CSV, `h` help.

## How the values are chosen

- **0% point (`MinThresh`):** rest plus a dead band. **100% point (`MaxThresh`):** the stop minus a margin.
- Dead bands are at least 2% after rest and 3% before the stop. They grow to cover 4σ of noise, or the worst
  excursion seen, if that's bigger.
- **The same % dead band is used on both sensors**, so they leave 0% and reach 100% at the same pedal position.
  A mismatched rest point is exactly what caused `data 35.csv`: APPS2 sat about 9% into a dead zone, so it
  trailed APPS1 by 8-9 pts and tripped the 10-pt limit.
- Values are rounded so the dead band is never smaller than intended and 100% stays reachable.

## Reading the result

| Result | Meaning |
|---|---|
| PASS | Calibrated, the sensors stay within 5 pts. If the current config column shows a big gap, the old thresholds were the problem. |
| MARGINAL | Calibrated, the gap is still 6-10 pts somewhere mid-travel. That's mechanical; calibration only fixes the ends. |
| FAIL | Even calibrated, the Pedal Box would flag it (gap > 10, range faults, or a sensor that rises when pressed). |
| CANNOT CALIBRATE | A sensor moves < 300 counts. Check its wiring, supply, and mounting. |

"APPSx travel before APPSy moves" is the free-play signature: one sensor reaches N% while the other still reads 0.
If it's still there after calibrating, look at that sensor's linkage.

After you apply new values to the Pedal Box, update `Config::Pedalbox` thresholds in `src/Config.h` here too,
so the "current cfg" column matches the car.

## Host test

The math (`src/CalMath.*`) has no Arduino dependencies. `test/host/test_calmath.cpp` runs the whole procedure
against simulated pedals: the data 35 miscalibration, 10% free play, a reversed sensor, a noisy sensor, and a
dead sensor.

`test/host/test_wizard.cpp` runs the real `Calibrator.cpp` state machine through a tiny Arduino shim
(`test/host/shim/`) and prints the exact report you'd see on the car.

```
g++ -std=c++11 -Wall -Isrc test/host/test_calmath.cpp src/CalMath.cpp -o test_calmath && ./test_calmath
g++ -std=c++11 -Wall -Itest/host/shim -Isrc test/host/test_wizard.cpp src/CalMath.cpp -o test_wizard && ./test_wizard
```

With MSYS2 on Windows, put `C:\msys64\ucrt64\bin` on PATH first, or the link step fails silently.
