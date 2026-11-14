# Subsystem 2 — test log

**Subsystem:** Motion control (MotionTask)  ·  **Student ID:** 2500917  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results, and
`docs/SETUP_AND_TEST.md` for the bring-up order on the robot.

## Test cases

Define a test here before running it. Test IDs never change (retired IDs stay listed).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | mbf_motion exists | IF-01 | auto | `subsystem2_test.c` | PASS |
| S2-U02 | *retired 25 Oct:* odometry snapshot round trip (MotionTask now writes the snapshot continuously, so a round trip would race it) | IF-02 | – | – | – |
| S2-U03 | MotionTask publishes the odometry snapshot every cycle | IF-02, NFR1 | auto | `subsystem2_test.c` | PASS |
| S2-U04 | PWM slices 4/5 at TOP 6249 (20 kHz), PIO0 SM0/SM1 enabled, `hw_ok` | NFR1, A3 | auto | `subsystem2_test.c` | PASS |
| S2-U05 | boots braked (IDLE_BRAKED, duty 0/0) | FR15, Section 5 BOOT | auto | `subsystem2_test.c` | PASS |
| S2-U06 | DRIVE from a source without authority is rejected and counted | IF-01, Section 6.1 | auto | `subsystem2_test.c` | PASS |
| S2-H01–H13 | controller on a motor model: authority rules; MOVE 1 m within 2 % and straight; TURN +90 / -180 within 1 deg; DRIVE watchdog 100 ms; STOP ramp ~1 m/s^2; speed tracking within 10 %; stall after 300 ms; E-stop latch and CLEAR_ESTOP; timeout -> FAILED; pose after MOVE-TURN-MOVE; encoder counter wrap; glitch ENC_FAULT/OK; pre-empted MOVE reported FAILED | FR5, NFR4, NFR5, FR15 | host | `make -C testing/subsystem2/host` | `host tests: N passed, 0 failed` |
| S2-H14 | named motion API (FR5): `motion_move_forward` / `_backward`, `motion_turn_left` / `_right`, `motion_stop`, `motion_drive` post the right command and sign; out-of-range arguments refused with nothing posted; a `motion_turn_left` 90 deg runs to MOTION_DONE in the controller | FR5 | host | `make -C testing/subsystem2/host` | `host tests: N passed, 0 failed` |
| S2-H15 | speed-step recorder: 150 samples of a MOVE on the motor model are sent as 1 header + 3 records each and unpack to the recorded values; a new MOVE restarts it | NFR5 (PID tuning report) | host | `test_s2.c` | PASS |
| S2-M01 | GP8–GP11 LOW at boot and after a safety stop (`motor_estop()`); 20 kHz, one input of each pair always 0 | A3, NFR1, Section 2.2 | manual | scope or logic analyser on GP8–GP11 | LOW/LOW at boot; 20 kHz PWM on one pin per motor when driving |
| S2-M02 | encoder counts per wheel revolution and direction sign | A4 | manual | hand-turn each wheel 10 revolutions, read `enc` in telemetry | set `MOTION_COUNTS_PER_REV`, `MOTION_ENC_SIGN_*` |
| S2-M03 | wheel circumference: 1 m straight move, 5 forward + 5 back | NFR4 | manual | `APP_BENCH 1`; dashboard "Forward 1 m" / "Back 1 m" vs tape measure; record in `MOTION_ACCURACY_EVALUATION.md` | within +-3 % after correcting `MOTION_WHEEL_CIRC_UM`; drift <= 50 mm (NFR4) |
| S2-M04 | track width: 90 deg left, 90 deg right and 180 deg turns, 5 each | NFR4, A5 | manual | `APP_BENCH 1`; dashboard turn buttons; protractor / floor marks; record in `MOTION_ACCURACY_EVALUATION.md` | +-8 deg / +-10 deg after correcting `MOTION_TRACK_MM` |
| S2-M05 | PID step response | NFR5 | manual | `APP_BENCH 1`; dashboard "Speed step" 100 / 200 / 300 mm/s, 3 each; "Download CSV" (10 ms samples, `motion_rec.h`); `python3 testing/subsystem2/step_analysis.py *.csv`; record in `PID_TUNING_REPORT.md` | within +-10 % <= 500 ms after the set-point ramp ends, steady error within +-10 % |
| S2-M06 | stall detection | FR15 | manual | hold one wheel while driving | STALL fault within 300–400 ms, motors braked |
| S2-M07 | MotionTask period jitter with WiFi publishing | NFR1, A2 | manual | build with `APP_TIMING_PINS=1`, logic analyser on GP6 for 60 s | every half-period 10 ms +-1 ms |

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
