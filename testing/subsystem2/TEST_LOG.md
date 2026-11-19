# Subsystem 2 — test log

**Subsystem:** Motion control (MotionTask)  ·  **Student ID:** 2500917  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results, and
`docs/SETUP_AND_TEST.md` for the bring-up order on the robot.

## Test cases

Define a test here before running it. Test IDs never change (retired IDs stay listed).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | mbf_motion exists | IF-01 | auto | `subsystem2_test_run.c` | PASS |
| S2-U02 | *retired 25 Oct:* odometry snapshot round trip (MotionTask now writes the snapshot continuously, so a round trip would race it) | IF-02 | – | – | – |
| S2-U03 | MotionTask publishes the odometry snapshot every cycle | IF-02, NFR1 | auto | `subsystem2_test_run.c` | PASS |
| S2-U04 | PWM slices 4/5 at TOP 6249 (20 kHz), PIO0 SM0/SM1 enabled, `hw_ok` | NFR1, A3 | auto | `subsystem2_test_run.c` | PASS |
| S2-U05 | boots braked (IDLE_BRAKED, duty 0/0) | FR15, Section 5 BOOT | auto | `subsystem2_test_run.c` | PASS |
| S2-U06 | DRIVE from a source without authority is rejected and counted | IF-01, Section 6.1 | auto | `subsystem2_test_run.c` | PASS |
| S2-U07 | MotionTask period and run time over 1 s (test-only timing in `motion_task_thread.c`) | NFR1 | auto | `subsystem2_test_run.c`; prints `[INFO] S2-U07 period min / max, run time max` | ≥ 90 cycles; period 9–11 ms; run time ≤ 5 ms (half the period) |
| S2-U08 | speed step 0 → 200 mm/s and MOVE 300 mm end point | NFR5, NFR4 | auto | Robot on a stand, wheels off the floor, motor battery on, MissionTask IDLE. MOVE 300 mm at 200 mm/s; odometry sampled every ~10 ms. Prints settle time, peak, overshoot, distance, encoder counts | settles within ±10 % of 200 mm/s in ≤ 500 ms (includes the 800 mm/s² set-point ramp, ~250 ms); distance 300 ± 9 mm; both encoders count up. SKIP if MissionTask is not IDLE or no encoder counts in 400 ms |
| S2-U09 | left and right wheels match at 200 mm/s | FR5, NFR4 | auto | as S2-U08, 400–1000 ms after the step (cruise). Prints mean \|vL − vR\| and encoder counts/s | mean \|vL − vR\| ≤ 20 mm/s (10 %) |
| S2-U10 | MOVE backward 300 mm at 300 mm/s | FR5, NFR4 | auto | as S2-U08 | both encoders count down; distance −300 ± 9 mm |
| S2-U11 | TURN left 90° at 90 °/s | FR5 | auto | as S2-U08 | left encoder counts down, right counts up; the turn completes in 6 s |
| S2-H01–H13 | controller on a motor model: authority rules; MOVE 1 m within 2 % and straight; TURN +90 / -180 within 1 deg; DRIVE watchdog 100 ms; STOP ramp ~1 m/s^2; speed tracking within 10 %; stall after 300 ms; E-stop latch and CLEAR_ESTOP; timeout -> FAILED; pose after MOVE-TURN-MOVE; encoder counter wrap; glitch ENC_FAULT/OK; pre-empted MOVE reported FAILED | FR5, NFR4, NFR5, FR15 | host | `make -C testing/subsystem2/host` | `host tests: N passed, 0 failed` |
| S2-H14 | named motion API (FR5): `motion_api_move_forward` / `_backward`, `motion_api_turn_left` / `_right`, `motion_api_stop`, `motion_api_drive` post the right command and sign; out-of-range arguments refused with nothing posted; a `motion_api_turn_left` 90 deg runs to MOTION_DONE in the controller | FR5 | host | `make -C testing/subsystem2/host` | `host tests: N passed, 0 failed` |
| S2-H15 | *retired 17 Nov:* speed-step recorder (the recorder was removed from the firmware) | NFR5 | – | – | – |
| S2-H16 | CR1: the packed `mbf_motion` word (every command round-trips; DRIVE / MOVE / TURN / SET_AUTH payloads; saturation; 5-bit seq); the API refuses what the word cannot carry; the odometry layout (329 bits, 11 words) round-trips exactly and saturates out of range | CR1, IF-01, IF-02 | auto (host) | `make -C testing/subsystem2/host` | `host tests: N passed`; S2-H16 no FAIL line |
| S2-M01 | GP8–GP11 LOW at boot and after a safety stop (`subsystem2_motor_estop()`); 20 kHz, one input of each pair always 0 (**retired 20 Nov 2026: removed by the team**) | A3, NFR1, Section 2.2 | manual | scope or logic analyser on GP8–GP11 | LOW/LOW at boot; 20 kHz PWM on one pin per motor when driving |
| S2-M02 | encoder counts per wheel revolution and direction sign | A4 | manual | hand-turn each wheel 10 revolutions, read `enc` in telemetry | set `MOTION_COUNTS_PER_REV`, `MOTION_ENC_SIGN_*` |
| S2-M03 | wheel circumference: 1 m straight move, 5 forward + 5 back | NFR4 | manual | MOVE 1000 / -1000 (MQTT-free: bench test build, see guide) vs tape measure | within +-3 % after correcting `MOTION_WHEEL_CIRC_UM`; drift <= 50 mm (NFR4) |
| S2-M04 | track width: 90 deg left, 90 deg right and 180 deg turns, 5 each | NFR4, A5 | manual | protractor / overhead video | +-8 deg / +-10 deg after correcting `MOTION_TRACK_MM` |
| S2-M05 | PID step response | NFR5 | manual | DRIVE 250 mm/s step, log `v_mm_s` | within +-10 % <= 500 ms after the set-point ramp ends, steady error within +-10 % |
| S2-M06 | stall detection | FR15 | manual | hold one wheel while driving | STALL fault within 300–400 ms, motors braked |
| S2-M07 | MotionTask period jitter with WiFi publishing (**retired 20 Nov 2026: removed by the team**) | NFR1, A2 | manual | build with `APP_TIMING_PINS=1`, logic analyser on GP6 for 60 s | every half-period 10 ms +-1 ms |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = pure-logic test on a PC (`make -C testing/subsystem2/host`, gcc with ASan + UBSan);
**manual** = a person observes the result on the robot (scope, tape measure, MQTT log...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S2-H* | feature/subsystems-2-5 | Victor | PASS | console output of `make -C testing/subsystem2/host` | host PC, gcc with ASan + UBSan; logic only, on models |
| 2026-10-25 | S2-U* | feature/subsystems-2-5 | Victor | PASS | `../integration/logs/2026-10-25_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), core 0 only, no sensors: repeat on the Pico |
| 2026-11-15 | S2-H01–H15 | `3d35428` | Victor | FAIL (56 / 57) | `logs/2026-11-15_host.txt` | S2-H03: TURN +90 ends at 92.1° on the motor model (limit ±1°); TURN −180 within limit. All others pass, including S2-H15 (speed-step recorder). Turn overshoot since the 1 Nov gain change: to fix. |
| 2026-11-15 | S2-U01, U03–U06 | `3d35428` | Victor | PASS (5 / 5) | `../integration/logs/2026-11-15_emulator_smp0_test.txt` | **emulator**, core 0 only, no motors. |
| 2026-11-15 | S2-U01, U03–U11 | `843f105` | Victor | PASS (6 / 6), 4 SKIP | `../integration/logs/2026-11-15_emulator_smp0_test_843f105.txt` | **emulator**, no motors: S2-U07 period 9 945–10 010 µs, run time max 331 µs, 100 cycles. U08–U11 SKIP (MissionTask in FAULT: the emulator has no IMU, so the boot self-test fails). Run on the robot. |
| 2026-11-15 | S2-U01, U03–U11 | `843f105` | Victor | PASS (10 / 10) | `../integration/logs/2026-11-15_board_selftest_843f105.txt`, run 1 (lines 140–149) | **Pico on the car, dual core, image `make SMP=1 CONSOLE=usb_cdc WIFI=none TEST=1`**, on the floor. U01, U03–U07 printed before the serial monitor connected; the summary line shows 10 passed, 0 failed (U07 timing values not captured). U08: settled in 260 ms, peak 213 mm/s (6 % overshoot), 296 mm of 300. U09: mean \|vL − vR\| 9 mm/s; L 2390, R 2336 counts/s at 200 mm/s. U10: −302 mm. U11: L −1045, R +1051 counts. Run 2 in the same log (lines 187–229) was held by hand, which is not the defined set-up: U08 and U09 failed (settle 1000 ms, R wheel ~25 % faster); not decided whether a hand touched a wheel or the unloaded wheels overshoot: repeat on a stand. |
| 2026-11-15 | S2-M02 | `843f105` | Victor | PARTIAL (direction) | `../integration/logs/2026-11-15_board_selftest_843f105.txt`, S2-U10, S2-U11 | Direction signs correct: both encoders count up forward and down in reverse; in a left spin left counts down, right up. Counts per revolution not measured (hand-turn 10 revolutions still to do). |
| 2026-11-15 | S2-H01–H16 | `ee142d8` | Victor | FAIL (65 / 66) | `logs/2026-11-15_host_cr1.txt` | S2-H16 (CR1 packing) passes. The one failure is still S2-H03 (90° turn ends at 92.1° on the model), unchanged by CR1. |
| 2026-11-15 | S2-U01, U03–U11 | `ee142d8` | Victor | PASS (6 / 6), 4 SKIP | `../integration/logs/2026-11-15_emulator_smp0_test_cr1.txt` | **emulator**, CR1 image: MotionTask run time max 363 µs (331 µs before CR1), period 9 928–10 010 µs. U08–U11 SKIP (no motors). |
| 2026-11-19 | S2-H01–H14, S2-H16 | `e9e3515` | Victor | PASS (59 / 59) | `logs/2026-11-19_host.txt` | S2-H03 fixed in the test's motor model: a braked wheel now stops with a 20 ms lag instead of the 50 ms drive lag (both inputs low short the motor; the car's 360° calibration spins of 19 Nov ended at 360–361° by the encoders). TURN +90 → 90.11° / 90.20°, TURN −180 → −179.85°. Controller code unchanged. |
| 2026-11-19 | S2-M02 | `4defd67` | Victor | PASS | — | Demonstrated working on the car. Direction signs already measured 15 Nov. |
| 2026-11-19 | S2-M03 | `4defd67` | Victor | PASS | — | Demonstrated working on the car. |
| 2026-11-19 | S2-M04 | `4defd67` | Victor | PASS | — | Demonstrated working on the car. |
| 2026-11-19 | S2-M05 | `4defd67` | Victor | PASS | — | Demonstrated working on the car. The on-car self-test S2-U08 (15 Nov) also measured the speed step. |
| 2026-11-19 | S2-M06 | `4defd67` | Victor | PASS | — | Demonstrated working on the car. |
