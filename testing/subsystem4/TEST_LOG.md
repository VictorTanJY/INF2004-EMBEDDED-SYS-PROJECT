# Subsystem 4 — test log

**Subsystem:** IMU terrain monitoring (ImuTask) and fault supervision (SafetyTask)  ·  **Student ID:** 2503229  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results, and
`docs/SETUP_AND_TEST.md` for the bring-up order on the robot.

## Test cases

Define a test here before running it. Test IDs never change (retired IDs stay listed).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | flg_safety exists | IF-05 | auto | `subsystem4_test.c` | PASS |
| S4-U02 | heartbeat counter advances (MotionTask's) | IF-07 | auto | `subsystem4_test.c` | PASS |
| S4-U03 | *retired 25 Oct:* IMU snapshot round trip (ImuTask now writes it continuously) | IF-08 | – | – | – |
| S4-U04 | ImuTask publishes its snapshot | IF-08 | auto | `subsystem4_test.c` | PASS |
| S4-U05 | 1 us timer runs at 1 MHz against clk_sys (watchdog tick fix) | NFR8 | auto | `subsystem4_test.c` | PASS, `[INFO] timer: 100000 us per 100 ms` |
| S4-U06 | watchdog enabled and fed | NFR8 | auto | `subsystem4_test.c` | PASS |
| S4-H01–H09 | humps 15/20/30 mm x 300 mm on a chassis model within NFR7; 10 mm x 300 mm below the 5 deg threshold (known limit, S4-H01b); no hump under +-1.5 m/s^2 accelerate/brake; long ramp is a ramp; tilt on/off with hysteresis, roll too; impact > 1.5 g; level calibration removes a mounting offset and refuses a moving robot; hard-iron offsets and heading; motion classes; heartbeat timeouts per task and across the ms wrap | FR6–FR8, FR15, NFR7, A9 | host | `make -C testing/subsystem4/host` | `host tests: N passed, 0 failed` |
| S4-M01 | IMU found (IRA_REG_M = 0x48), snapshot `ok` = 1 | A3 | manual | boot, telemetry / snapshot | `ok` 1, i2c_errors not rising |
| S4-M02 | axis orientation and signs | A8 | manual | tilt nose up, then left side up, by hand | pitch and roll go positive; else fix `IMU_AXIS_*` / `IMU_SIGN_*` |
| S4-M03 | pitch noise while driving on flat ground | A8 | manual | log pitch at 0.3 m/s | within +-1 deg; else re-check mounting, consider lowering `IMU_HUMP_ON_CDEG` only after this |
| S4-M04 | hump height, 10 passes each on 10 / 20 / 30 mm humps | FR7, NFR7, A9 | manual | `HUMP_END` data vs measured height | within +-20 % or +-5 mm |
| S4-M05 | tilt > 30 deg -> FAULT; reset refused while still tilted | FR15 | manual | tilt the robot by hand | FAULT; GP21 long press answered with FAULT until level |
| S4-M06 | impact -> both motors braked <= 20 ms | NFR8 | manual | logic analyser GP19 (INT1) -> GP8–GP11 | <= 20 ms |
| S4-M07 | remote E-stop -> brake <= 20 ms with WiFi publishing | NFR8, FR14 | manual | publish estop; analyser on GP8–GP11 + a GPIO toggled on receipt | <= 20 ms |
| S4-M08 | missed heartbeat -> FAULT | FR15 | manual | debug build that suspends one task | HEARTBEAT fault, log code 41 names the task |
| S4-M09 | power: motor start + servo sweep, 10 runs, 5 min idle | A10 | manual | scope V+ and 3V3 | no brown-out resets, no power-bank auto-off |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = pure-logic test on a PC (`make -C testing/subsystem4/host`, gcc with ASan + UBSan);
**manual** = a person observes the result on the robot (scope, tape measure, MQTT log...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S4-H* | feature/subsystems-2-5 | Claude (for the team) | PASS | console output of `make -C testing/subsystem4/host` | host PC, gcc with ASan + UBSan; logic only, on models |
| 2026-10-25 | S4-U* | feature/subsystems-2-5 | Claude (for the team) | PASS | `../integration/logs/2026-10-25_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), core 0 only, no sensors: repeat on the Pico |
