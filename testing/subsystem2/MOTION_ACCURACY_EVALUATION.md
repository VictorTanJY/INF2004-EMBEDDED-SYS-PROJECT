# Subsystem 2 — motion accuracy evaluation

**Requirements:** NFR4 — straight moves within ±2 % of the commanded
distance over 1 m, 90° turns within ±5°, 180° turns within ±8°;
NFR5 — lateral drift ≤ 30 mm over a 1 m straight move without a line.
**Tests:** S2-M03 (distance, drift), S2-M04 (turns).
**Code:** `motion_core.c` (MOVE and TURN run closed-loop on the encoders),
constants in `motion_cfg.h`.

**Status:** method written; **runs to be done**. Fields marked *to fill*
are completed from the car runs.

## 1. Set-up

- Firmware built with `APP_BENCH 1` (`app_program/app_config.h`).
- Dashboard *Manual moves*: **Forward 1 m** / **Back 1 m** (MOVE at
  200 mm/s), **Left 90°**, **Right 90°**, **U-turn 180°** (TURN at
  90 °/s). Each press is one move; the car must be in IDLE.
- Floor: the track mat or the lab floor, the same for every run (note
  which). Battery charged; note the voltage if a meter is available.
- Constants in use: 2560 counts per wheel turn, 213.6 mm wheel
  circumference, 110 mm track width (measured 1 Nov).

## 2. How to measure

**Distance and drift (S2-M03).** Lay a straight tape line on the floor.
Put a mark on the car's side level with the front axle, and line it and
the car's centre up with the start of the tape. After the move, measure

- the distance along the tape from the start to the mark (to 1 mm);
- the drift: how far the car's centre is from the tape line (to 1 mm,
  + = left).

**Turns (S2-M04).** Draw two crossing lines at 90° (a set square or a
floor tile edge), and a third for 180° if needed. Line the car's side up
with one line, turn, and measure the angle between the car's side and the
line with a protractor (to 1°, + = turned too far).

**What the car thinks it did.** The telemetry's `enc: [left, right]`
before and after each move gives the car's own estimate, which shows
whether an error comes from the constants or from wheel slip:

- distance (mm) = (Δleft + Δright) / 2 × 213.628 / 2560
- angle (°) = (Δright − Δleft) × 213.628 / 2560 / 110 × 180 / π
  (+ = left; it will be close to the command, since the turn stops when
  this reaches the target)

Write both numbers in the tables. Save the dashboard log of the session
as evidence.

## 3. Results — *to fill*

### 3.1 Forward 1 m (10 runs) — limit ±20 mm; drift ≤ 30 mm

| Run | Measured (mm) | Error (mm) | Drift (mm) | Car's estimate (mm) |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |
| 5 | | | | |
| 6 | | | | |
| 7 | | | | |
| 8 | | | | |
| 9 | | | | |
| 10 | | | | |

### 3.2 Back 1 m (5 runs) — limit ±20 mm; drift ≤ 30 mm

| Run | Measured (mm) | Error (mm) | Drift (mm) | Car's estimate (mm) |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |
| 5 | | | | |

### 3.3 Left 90° (10 runs) — limit ±5°

| Run | Measured (°) | Error (°) | Car's estimate (°) |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 4 | | | |
| 5 | | | |
| 6 | | | |
| 7 | | | |
| 8 | | | |
| 9 | | | |
| 10 | | | |

### 3.4 Right 90° (10 runs) — limit ±5°

| Run | Measured (°) | Error (°) | Car's estimate (°) |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 4 | | | |
| 5 | | | |
| 6 | | | |
| 7 | | | |
| 8 | | | |
| 9 | | | |
| 10 | | | |

### 3.5 U-turn 180° (10 runs) — limit ±8°

| Run | Measured (°) | Error (°) | Car's estimate (°) |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 4 | | | |
| 5 | | | |
| 6 | | | |
| 7 | | | |
| 8 | | | |
| 9 | | | |
| 10 | | | |

## 4. Summary — *to fill*

For each test: mean error (accuracy), standard deviation (repeatability),
worst case, and whether the limit is met in every run.

| Test | Runs | Mean error | Std dev | Worst case | Limit | Met? |
|---|---|---|---|---|---|---|
| Forward 1 m | 10 | | | | ±20 mm | |
| Back 1 m | 5 | | | | ±20 mm | |
| Drift over 1 m | 15 | | | | ≤ 30 mm | |
| Left 90° | 10 | | | | ±5° | |
| Right 90° | 10 | | | | ±5° | |
| U-turn 180° | 10 | | | | ±8° | |

## 5. Conclusion — *to fill*

If a mean error is outside its limit while the car's own estimate is on
target, the constant is wrong: correct it and repeat that test.

- **Distance:** new circumference = 213.628 mm × measured ÷ car's estimate.
- **Turns:** the car turns until its own estimate reaches the target, so
  a turn that ends past the target means the track width is set too
  large: new track width = 110 mm × commanded ÷ measured, using the
  mean of the 180° turns.

If the spread is large while the mean is close, the error is slip or the
start position; note it here. A test outside its limit after correction
is reported as not met, with the cause.
