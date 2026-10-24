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
| S2-M01 | GP8–GP11 LOW at boot and after E-stop; 20 kHz, one input of each pair always 0 | A3, NFR1, Section 2.2 | manual | scope or logic analyser on GP8–GP11 | LOW/LOW at boot; 20 kHz PWM on one pin per motor when driving |
| S2-M02 | encoder counts per wheel revolution and direction sign | A4 | manual | hand-turn each wheel 10 revolutions, read `enc` in telemetry | set `MOTION_COUNTS_PER_REV`, `MOTION_ENC_SIGN_*` |
| S2-M03 | wheel circumference: 1 m straight move, 5 runs | NFR4 | manual | MOVE 1000 (MQTT-free: bench test build, see guide) vs tape measure | within +-2 % after correcting `MOTION_WHEEL_CIRC_UM`; drift <= 30 mm (NFR5) |
| S2-M04 | track width: 90 deg and 180 deg turns, 10 each | NFR4, A5 | manual | protractor / overhead video | +-5 deg / +-8 deg after correcting `MOTION_TRACK_MM` |
| S2-M05 | PID step response | NFR5 | manual | DRIVE 250 mm/s step, log `v_mm_s` | settles <= 300 ms, steady error < 10 % |
| S2-M06 | stall detection | FR15 | manual | hold one wheel while driving | STALL fault within 300–400 ms, motors braked |
| S2-M07 | MotionTask period jitter with WiFi publishing | NFR1, A2 | manual | build with `APP_TIMING_PINS=1`, logic analyser on GP6 for 60 s | every half-period 10 ms +-1 ms |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = pure-logic test on a PC (`make -C testing/subsystem2/host`, gcc with ASan + UBSan);
**manual** = a person observes the result on the robot (scope, tape measure, MQTT log...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S2-H* | feature/subsystems-2-5 | Claude (for the team) | PASS | console output of `make -C testing/subsystem2/host` | host PC, gcc with ASan + UBSan; logic only, on models |
| 2026-10-25 | S2-U* | feature/subsystems-2-5 | Claude (for the team) | PASS | `../integration/logs/2026-10-25_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), core 0 only, no sensors: repeat on the Pico |
