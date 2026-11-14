# Subsystem 2 — PID tuning report (wheel speed control)

**Requirement:** NFR5 — steady-state wheel speed within ±10 % of the set-point,
settling ≤ 300 ms after a step.  **Code:** `app_program/subsystem2/motion_core.c`
(`pid_wheel()`), gains in `motion_cfg.h`.  **Test:** S2-M05.

**Status:** controller and tuning history written; **step-response
measurements to be taken** (section 5). Fields marked *to fill* are
completed from the car runs.

## 1. Controller

Each wheel has its own controller, run by MotionTask every 10 ms (core 1,
priority 6). Output: PWM duty in per mille (±1000).

```
e    = set-point − measured speed                       (mm/s)
duty = KFF·sp + sign(sp)·MIN_DUTY + KP·e + KI·Σ(e·dt)    (clamped to ±1000)
```

| Part | Value | Purpose |
|---|---|---|
| Feed-forward `MOTION_KFF_MILLI` | 1.2 ‰ per mm/s | most of the duty for the target speed at once |
| Static-friction offset `MOTION_MIN_DUTY` | 23 ‰ | duty a wheel needs before it turns at all |
| P `MOTION_KP_MILLI` | 1.04 ‰ per mm/s | corrects the present speed error |
| I `MOTION_KI_MILLI` | 6.0 ‰ per mm of accumulated error | removes the steady-state error (load, battery, the two motors' difference) |
| D | none | see below |
| Integral limit `MOTION_I_MAX` | 400 ‰ | the I term alone can never exceed 40 % duty |
| Anti-windup | conditional integration | the integral only grows while the output is not saturated, or when the error pulls it back out |
| Minimum target `MOTION_MIN_TARGET_MM_S` | 10 mm/s | below it the wheel is braked and the integral cleared |
| Direction change | integral cleared | a reversal starts clean |

**Speed measurement:** encoder counts every 10 ms (PIO0 quadrature, 2560
counts per wheel turn, 0.083 mm per count), converted to mm/s and filtered
`v += (v_raw − v) / 2` (time constant about 15 ms).

**Set-points are ramped**, not stepped: every MOVE, TURN and DRIVE changes
the set-point at `MOTION_ACCEL_MM_S2` (800 mm/s²), so a 300 mm/s command
reaches its target after 375 ms. The step response is therefore measured
as how closely the wheel follows that ramp and how quickly it settles once
the ramp ends (section 4).

**Why no D term:** the speed is measured by counting encoder edges in a
10 ms window, so it moves in steps of 8.3 mm/s. A derivative of that signal
is mostly noise, which a D term would turn into duty jitter. The
feed-forward already supplies the fast part of the response, and the ramp
removes the sudden set-point changes a D term would otherwise help with.
The line follower (`LINE_KD`, Buddy 3) found the same on 1 Nov: its D term
reacted to sensor noise and was set to 0.

## 2. Measured constants used by the controller

| Constant | Value | How it was found |
|---|---|---|
| Counts per wheel turn | 2560 | 10 hand turns of each wheel, 1 Nov (about 2530 counted); `MOTION_COUNTS_PER_REV` |
| Wheel circumference | 213.6 mm | driving against a tape measure, 1 Nov; `MOTION_WHEEL_CIRC_UM` |
| Track width | 110 mm | turns, 1 Nov; `MOTION_TRACK_MM` |
| Motor and encoder direction | per wheel | 1 Nov; `MOTION_MOT_SIGN_*`, `MOTION_ENC_SIGN_*` |

## 3. Tuning history

| Date | KFF | KP | KI | MIN_DUTY | What was seen | Change |
|---|---|---|---|---|---|---|
| 25 Oct | 2.0 | 1.5 | 8.0 | 120 | first values, before the car ran; geometry still placeholders (1400 counts per turn, 204 mm circumference, 130 mm track) | — |
| 1 Nov | 2.0 | 1.5 | 8.0 | 120 | geometry measured on the car (2560 counts per turn, 213.6 mm, 110 mm) | constants corrected |
| 1 Nov | 2.0 → **1.2** | 1.5 → **1.04** | 8.0 → **6.0** | 120 → **23** | a 50 mm/s target drove the wheels to about 100 mm/s and back to 0 (dashboard wheel-speed graph): too much open-loop power | feed-forward and offset lowered; P and I lowered with them; wheel braked below 10 mm/s; integral reset on a direction change |
| *to fill* | | | | | step responses below | *keep, or retune and repeat* |

## 4. Method (S2-M05)

1. Build with `APP_BENCH 1` (`app_program/app_config.h`) and flash.
2. Car on the floor with at least 1.2 m clear in front, battery charged
   (note the voltage if a meter is available).
3. In the dashboard, under *Manual moves*, press **Speed step 100 mm/s**.
   The car drives 1 m. MotionTask records each wheel's set-point,
   measured speed and duty every 10 ms for the first 1.5 s of the move
   (`motion_rec.h`) and then sends them; *Speed log: 150 / 150 samples*
   appears after about 2 s.
4. Press **Download CSV**. Repeat 3 times each at 100, 200 and 300 mm/s.
5. Run `python3 testing/subsystem2/step_analysis.py speed_log_*.csv`. It
   prints the table for section 5 and saves a graph per run (needs
   matplotlib).

Definitions (as in `step_analysis.py`):

- **ramp end** — when the set-point reaches its final value;
- **rise** — 10 % to 90 % of the target;
- **overshoot** — (peak − target) / target;
- **settle** — time after the ramp end until the speed stays within ±10 %
  of the target (the NFR5 band) for the rest of the log; NFR5: ≤ 300 ms;
- **steady error** — mean speed over the last 500 ms against the target;
  NFR5: within ±10 %;
- **max duty** — 1000 means the output saturated.

## 5. Results — *to fill*

| Run | Target (mm/s) | Wheel | Rise (ms) | Overshoot (%) | Settle (ms, ≤ 300) | Steady error (%, ±10) | Max duty (‰) |
|---|---|---|---|---|---|---|---|
| 1 | 100 | L / R | | | | | |
| 2 | 100 | L / R | | | | | |
| 3 | 100 | L / R | | | | | |
| 4 | 200 | L / R | | | | | |
| 5 | 200 | L / R | | | | | |
| 6 | 200 | L / R | | | | | |
| 7 | 300 | L / R | | | | | |
| 8 | 300 | L / R | | | | | |
| 9 | 300 | L / R | | | | | |

Graphs: *to add* (one per target speed, from `step_analysis.py`).

Conditions: date, battery, floor — *to fill*.

## 6. Conclusion — *to fill*

State whether NFR5 is met at each speed. If a run does not settle within
300 ms or the steady error is outside ±10 %, record it here with the
likely cause (for example duty saturation at 300 mm/s, or the
static-friction offset at 100 mm/s), then either retune one gain at a time
(add a row to section 3 and repeat section 5) or report the shortfall.
