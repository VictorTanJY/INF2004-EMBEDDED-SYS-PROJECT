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
| S4-U07 | IMU reads level; no I2C errors in 1 s | FR6, FR15 | auto | robot on a level surface. Prints pitch, roll, \|a\| − 1 g, I2C errors, calibration bits | \|pitch\|, \|roll\| ≤ 5.0°; 0 I2C errors; not tilted. SKIP if the IMU does not answer |
| S4-U08 | fault flag → both motors braked (STALL bit set by the test) | NFR8, IF-03, IF-05 | auto | `tk_set_flg(flg_safety, STALL)`; `motor_estop()` records the time (test build only). Prints the latency | latency ≤ 20 000 µs; brake latched; GP8–GP11 SIO outputs, LOW; fault latched in SafetyTask |
| S4-U09 | Reset clears the fault, as the dashboard Reset does | FR14, FR15, IF-06 | auto | `mission_post(REMOTE_RESET)`; wait up to 8 s | latch cleared, PWM reconnected (GP8 back on PWM), MotionTask IDLE_BRAKED, mission back to IDLE if it was IDLE |
| S4-H01–H09 | humps 15/20/30 mm x 300 mm on a chassis model within NFR7; 10 mm x 300 mm below the 5 deg threshold (known limit, S4-H01b); no hump under +-1.5 m/s^2 accelerate/brake; long ramp is a ramp; tilt on/off with hysteresis, roll too; impact > 1.5 g; level calibration removes a mounting offset and refuses a moving robot; hard-iron offsets and heading; motion classes; heartbeat timeouts per task and across the ms wrap | FR6–FR8, FR15, NFR7, A9 | host | `make -C testing/subsystem4/host` | `host tests: N passed, 0 failed` |
| S4-H10 | humps under ONE wheel (roll): 10 / 20 / 30 mm x 300 mm, both sides, within NFR7; lifting by hand with the wheels still and a short 10 mm jolt are not humps | FR6, FR7, NFR7 | host | `make -C testing/subsystem4/host` | `host tests: N passed, 0 failed` |
| S4-M01 | IMU found (IRA_REG_M = 0x48), snapshot `ok` = 1 | A3 | manual | boot, telemetry / snapshot | `ok` 1, i2c_errors not rising |
| S4-M02 | axis orientation and signs | A8 | manual | tilt nose up, then right side up, by hand | pitch and roll go positive (roll + = right side up); else fix `IMU_AXIS_*` / `IMU_SIGN_*` |
| S4-M03 | pitch noise while driving on flat ground | A8 | manual | log pitch at 0.3 m/s | within +-1 deg; else re-check mounting, consider lowering `IMU_HUMP_ON_CDEG` only after this |
| S4-M04 | hump height, 5 passes each on test humps of known height | FR7, NFR7, A9 | manual | "Hump finished" peak vs measured height | within +-30 % or +-5 mm, whichever is larger |
| S4-M04b | one-wheel hump height, 5 passes each on 1 cm and 2 cm humps under the right wheel | FR7, NFR7 | manual | line-follow over the hump; read Humps table (peak, max roll) | HUMP_END peak within +-5 mm; if all readings are off by the same factor, scale `IMU_SIDE_LEVER_MM` by real / reported |
| S4-M05 | overturned during a run -> brake | FR15 (revised 14 Nov) | manual | during a run, tilt the robot by hand past 30 deg for > 0.2 s | FAULT "Overturned / tilted" (code 8), motors braked; stays stopped until reset |
| S4-M06 | lifted during a run -> brake, run ended | FR15 (revised 14 Nov) | manual | during a run, lift the robot level (IMU) and also lift it so the IR sensors see no floor | FAULT "Lifted" (code 1), motors braked, run summary published; humps measured in the air not counted |
| S4-M07 | withdrawn 14 Nov: the remote E-stop command was removed (MQTT `stop` and the lift / overturn stops remain) | — | — | — | — |
| S4-M08 | missed heartbeat -> FAULT | FR15 | manual | debug build that suspends one task | HEARTBEAT fault, log code 41 names the task |
| S4-M09 | power: motor start + servo sweep, 10 runs, 5 min idle | A10 | manual | scope V+ and 3V3 | no brown-out resets, no power-bank auto-off |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = pure-logic test on a PC (`make -C testing/subsystem4/host`, gcc with ASan + UBSan);
**manual** = a person observes the result on the robot (scope, tape measure, MQTT log...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S4-H* | feature/subsystems-2-5 | Victor | PASS | console output of `make -C testing/subsystem4/host` | host PC, gcc with ASan + UBSan; logic only, on models |
| 2026-10-25 | S4-U* | feature/subsystems-2-5 | Victor | PASS | `../integration/logs/2026-10-25_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), core 0 only, no sensors: repeat on the Pico |
| 2026-11-14 | S4-M05 | `4d98d83` | Victor | PASS | `../system/logs/2026-11-14_0244_dashboard_lift_overturn.txt` | Tilted by hand during a run: FAULT "Overturned / tilted" (code 8), motors braked, at 02:44:49 and 02:45:28. |
| 2026-11-14 | S4-M06 | `4d98d83`, `cf3e2d7` | Victor | PASS | `../system/logs/2026-11-14_0244_dashboard_lift_overturn.txt`, `../system/logs/2026-11-14_1625_dashboard_lift_no_floor.txt` | Lifted level during a run: IMU (up axis +298 mg) at 02:45:10; IR "no floor" (all three 1000 for 40 mm) at 16:25:35. FAULT code 1, run summary published, highest hump 0 mm. |
| 2026-11-14 | S4-M08 | `cf3e2d7` | Victor | PASS | `../system/logs/2026-11-14_0424_dashboard_heartbeat_timeout.txt`, `../system/logs/2026-11-14_1711_dashboard_barcode_A_heartbeat.txt` | A real missed heartbeat (LineBarcodeTask, mask 2) at 04:24:48, 17:11:31, 17:12:03: "Heartbeat missed" then FAULT code 32, robot stopped. The cause (slow barcode decode) is fixed in `673f9e2`. |
| 2026-11-15 | S4-H01–H10c | `3d35428` | Victor | PASS (66 checks) | `logs/2026-11-15_host.txt` | Includes the lift tests S4-H05 to S4-H05e. |
| 2026-11-15 | S4-U01, U02, U04–U06 | `3d35428` | Victor | PASS (5 / 5) | `../integration/logs/2026-11-15_emulator_smp0_test.txt` | **emulator**: flg_safety, heartbeat, IMU snapshot, 1 µs timer, watchdog enabled and fed. |
| 2026-11-15 | S4-U01, U02, U04–U09 | `843f105` | Victor | PASS (7 / 7), 1 SKIP | `../integration/logs/2026-11-15_emulator_smp0_test_843f105.txt` | **emulator**: fault flag → `motor_estop()` 7 µs (emulated, one core); Reset cleared it at once. S4-U07 SKIP (no IMU). Run on the robot for the dual-core latency. |
