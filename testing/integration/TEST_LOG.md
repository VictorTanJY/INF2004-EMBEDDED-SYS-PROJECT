# Integration — test log

Tests that two or more subsystems work together. There is one test (or
more) for every interface listed in TEAM.md section 5 (IF-01 to IF-12).
Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results. Car logs
referenced below are in `../system/logs/`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | motion commands reach MotionTask only from the source with authority | IF-01, FR5 | auto + manual | (a) S2-U06 on target. (b) A run: line following, then a barcode turn. | (a) DRIVE without LINE authority rejected. (b) Line follower drives in LINE_FOLLOW; MissionTask's turn runs in EXECUTE_NAV. |
| INT-02 | odometry snapshot read by the other tasks | IF-02 | auto + manual | (a) S2-U03 on target. (b) Telemetry during a run. | (a) Snapshot updated every cycle. (b) `enc`, `dist_mm`, `v_mm_s` change with the wheels. |
| INT-03 | SafetyTask brakes the motors through `motor_estop()` | IF-03, FR15 | manual | Lift or tilt the robot during a run. | Wheels stop at once; FAULT; motors stay braked until reset. |
| INT-04 | line snapshot read by mission, obstacle and comms | IF-04 | auto + manual | (a) S3-U02 on target. (b) Line loss during a run. | (a) Snapshot published. (b) Telemetry `line.lost` and LINE_SEARCH follow the loss. |
| INT-05 | fault bits from S2, S4, S5 reach SafetyTask | IF-05, FR15 | auto + manual | (a) S4-U01 on target. (b) The SYS-15 fault tests. | (a) `flg_safety` exists. (b) Each cause gives FAULT with its own code. |
| INT-06 | events from every task reach MissionTask | IF-06 | manual | A run with a barcode, a junction and an obstacle; remote start / stop. | Barcode latched → EXECUTE_NAV; obstacle → OBSTACLE_HANDLING; remote start / stop change the state. |
| INT-07 | heartbeats monitored by SafetyTask | IF-07, NFR1 | auto + manual | (a) S4-U02 on target. (b) A missed heartbeat on the robot. | (a) MotionTask heartbeat advances. (b) "Heartbeat missed" names the task (mask), FAULT code 32. |
| INT-08 | IMU snapshot read by mission, obstacle and comms | IF-08 | auto + manual | (a) S4-U04 on target. (b) Tilt the robot during a run. | (a) Snapshot published. (b) Tilt reaches SafetyTask (FAULT code 8); pitch shown in telemetry. |
| INT-09 | MissionTask requests scans from ObstacleTask | IF-09 | auto + manual | (a) S5-U02, S5-U06 on target. (b) An obstacle ahead during a run. | (a) `mbf_scan` exists, ObstacleTask runs. (b) Stop → sweep → "Bypass plan" logged. |
| INT-10 | mission and obstacle snapshots read by comms | IF-10 | auto + manual | (a) S5-U05 on target. (b) Send the map, then watch status / telemetry. | (a) No map until MQTT sends one. (b) The status shows the map; `nav` shows the latched command. |
| INT-11 | MQTT link to the laptop | IF-11, FR13, FR14 | manual | Dashboard connected: map, start, stop during a run. | Each command acknowledged and acted on; telemetry and events arrive. |
| INT-12 | every task posts telemetry records without blocking | IF-12, NFR10 | auto + manual | (a) S1-U01 on target. (b) Events from S2–S5 during a run. | (a) Post with TMO_POL accepted. (b) Events from every subsystem appear in the dashboard log. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (dashboard log, robot behaviour).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | INT-01, 02, 04, 05, 07–10, 12 (part a) | feature/subsystems-2-5 | Victor | PASS | `logs/2026-10-25_emulator_smp0_test.txt` | On-target self-tests in the emulator (core 0 only, no sensors). |
| 2026-11-14 | INT-03 | `4d98d83`, `cf3e2d7` | Victor | PASS | `../system/logs/2026-11-14_0244_dashboard_lift_overturn.txt`, `../system/logs/2026-11-14_1625_dashboard_lift_no_floor.txt` | Overturn and lift during runs: FAULT, motors braked. |
| 2026-11-14 | INT-05 | `4d98d83`, `cf3e2d7` | Victor | PASS (part b: lift, overturn, heartbeat) | as INT-03, plus `../system/logs/2026-11-14_0424_dashboard_heartbeat_timeout.txt` | Codes 1, 8 and 32 seen. Too close and stall still to test. |
| 2026-11-14 | INT-06 | `cf3e2d7` | Victor | PASS | `../system/logs/2026-11-14_0334_dashboard_map_barcode_F.txt` | Barcode 'F' read → latched → EXECUTE_NAV; remote start → LINE_FOLLOW; remote stop → MISSION_COMPLETE. |
| 2026-11-14 | INT-07 | `cf3e2d7` | Victor | PASS (part b) | `../system/logs/2026-11-14_1711_dashboard_barcode_A_heartbeat.txt` | "Heartbeat missed", value 2 (LineBarcodeTask), then FAULT code 32 at 17:11:31 and 17:12:03. |
| 2026-11-14 | INT-08 | `4d98d83` | Victor | PASS (part b) | `../system/logs/2026-11-14_0244_dashboard_lift_overturn.txt` | Tilt over 30° → FAULT code 8 at 02:44:49 and 02:45:28. |
| 2026-11-14 | INT-10 | `cf3e2d7` | Victor | PASS (part b) | `../system/logs/2026-11-14_0334_dashboard_map_barcode_F.txt` | "Map received", "Map stored" before each run; latched command "F → U-turn". |
| 2026-11-14 | INT-11 | `cf3e2d7` | Victor | PASS | `../system/logs/2026-11-14_0334_dashboard_map_barcode_F.txt`, `../system/logs/2026-11-14_0400_dashboard_start_stop.txt` | Map, remote start and remote stop each acted on; events from S2–S5 arrived. |
| 2026-11-14 | INT-12 | `cf3e2d7` | Victor | PASS (part b) | all `../system/logs/2026-11-14_*` | Events from motion ("Move done"), line (barcode, junction), IMU / safety (lift, motion class), mission (state) and obstacle ("Slowing for an obstacle"). |
| 2026-11-15 | INT-01, 02, 04, 05, 07–10, 12 (part a) | `3d35428` | Victor | PASS | `logs/2026-11-15_emulator_smp0_test.txt` | S2-U06, S2-U03, S3-U02, S4-U01, S4-U02, S4-U04, S5-U02, S5-U05, S5-U06, S1-U01: 27 / 27 checks PASS. |
| 2026-11-15 | INT-01, 02, 04, 05, 07–10, 12 (part a) | `843f105` | Victor | PASS | `logs/2026-11-15_emulator_smp0_test_843f105.txt` | Same checks as above on the extended self-test image: 39 checks, 33 PASS, 0 FAIL, 6 SKIP (hardware not present on the emulator). |
