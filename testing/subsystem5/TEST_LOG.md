# Subsystem 5 — test log

**Subsystem:** Obstacle scanning & avoidance (ObstacleTask) and system state machine (MissionTask)  ·  **Student ID:** 2500443  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results, and
`docs/SETUP_AND_TEST.md` for the bring-up order on the robot.

## Test cases

Define a test here before running it. Test IDs never change (retired IDs stay listed).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | mbf_mission exists | IF-06 | auto | `subsystem5_test.c` | PASS |
| S5-U02 | mbf_scan exists | IF-09 | auto | `subsystem5_test.c` | PASS |
| S5-U03 | flg_obst exists | IF-09 | auto | `subsystem5_test.c` | PASS |
| S5-U04 | *retired 25 Oct:* mission snapshot starts in BOOT (MissionTask now runs and may already be past BOOT) | IF-10 | – | – | – |
| S5-U05 | mission snapshot valid; no barcode map until MQTT sends one (not hard-coded) | IF-10, FR4 (instructor 24 Oct) | auto | `subsystem5_test.c` | PASS |
| S5-U06 | ObstacleTask runs (range_cyc / 100 ms wait) | IF-09 | auto | `subsystem5_test.c` | PASS |
| S5-U07 | echo conversion and median on the target | FR9 | auto | `subsystem5_test.c` | PASS |
| S5-H01–H08 | statechart: BOOT self-test / FAULT; start refused uncalibrated; CALIBRATING phases; MQTT map (refused while running, duplicates, '*'); barcode latched until the next junction, LEFT/RIGHT/U-turn/STRAIGHT manoeuvres, a barcode read during a manoeuvre latched for the next junction; obstacle -> profile -> bypass -> recovery; CONTINUE; BLOCKED_TIMEOUT; FAULT reset via SafetyTask; bounded line search; set only in IDLE; hump / slow-down speed caps; mbf_mission drop while running = FAULT | Section 5, FR4, FR9–FR12, FR14, FR15 | host | `make -C testing/subsystem5/host` | `host tests: N passed, 0 failed` |
| S5-H09–H13 | geometry on a 15 deg beam model: 10/15/20 cm boxes at 20 cm, centred and +-4 cm: width within 10 %, <= 30 pings; plan side, wall on one side, blocked, beside the path, nothing; bypass path returns to the line with heading 0, deviation within FR11, >= 50 mm clearance; seek-line window; hump mask 600 mm / 4 s | FR10–FR12, A11 | host | same | same |
| S5-M01 | servo angles 0 / 90 / 180 deg | FR10 | manual | set angles, protractor | set `OBST_SERVO_MIN_US` / `_MAX_US`, `OBST_SERVO_LEFT_IS_HIGH` |
| S5-M02 | HC-SR04P on 3.3 V: 10 / 30 / 100 cm against a ruler | A11 | manual | `d_cm` in telemetry, servo at 90 | within +-1 cm, no lost echoes |
| S5-M03 | approach and stop, 5 straight approaches at cruise speed | FR9, NFR6 | manual | dashboard log + tape measure at rest | slow-down below 40 cm; rests >= 10 cm from the obstacle; else raise `stop_cm` |
| S5-M04 | width estimate: 10 / 15 / 20 cm boxes, left / centre / right | FR10 (instructor 23 Oct: 10–20 cm, accuracy graded) | manual | `w_mm` in telemetry vs ruler | grade A <= 10 %, B <= 20 %, C <= 30 % |
| S5-M05 | bypass, 3 runs per offset (left / centre / right) | FR11 | manual | video + dashboard log | no contact; the side taken matches the logged "Bypass plan"; time from OBSTACLE to RECOVERY_DONE logged (instructor grading: A < 3 s, B < 5 s, C < 7 s) |
| S5-M06 | line reacquisition | FR12 | manual | the S5-M05 runs | >= 2 of 3 runs per offset end in LINE_FOLLOW on the original line |
| S5-M07 | barcode map over MQTT, then a run: barcode -> next junction -> movement | FR3, FR4 | manual | send `map`, check `status.map`; run the junction track | the mapped movement at the next cross-junction, every time |
| S5-M08 | blocked path | Section 5 | manual | box with walls both sides | 2 reverse-and-rescans, then BLOCKED, FAULT after 10 s |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = pure-logic test on a PC (`make -C testing/subsystem5/host`, gcc with ASan + UBSan);
**manual** = a person observes the result on the robot (scope, tape measure, MQTT log...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S5-H* | feature/subsystems-2-5 | Victor | PASS | console output of `make -C testing/subsystem5/host` | host PC, gcc with ASan + UBSan; logic only, on models |
| 2026-10-25 | S5-U* | feature/subsystems-2-5 | Victor | PASS | `../integration/logs/2026-10-25_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), core 0 only, no sensors: repeat on the Pico |
| 2026-11-14 | S5-M07 | `cf3e2d7` | Victor | PARTIAL | `../system/logs/2026-11-14_0334_dashboard_map_barcode_F.txt` | Map sent, F read and latched "F → U-turn at the next junction", EXECUTE_NAV at the next junction (03:34:28, 03:40:58). That junction was detected on the plain line just after the barcode, so the U-turn came early. |
| 2026-11-14 | S5-M03 | `cf3e2d7` | Victor | PARTIAL | `../system/logs/2026-11-14_0351_dashboard_obstacle_slowdown.txt` | "Slowing for an obstacle" at 39.8, 39.6 and 39.6 cm on three approaches; a second line reads −0.1 cm (display value to check). Rest distance not measured yet. |
| 2026-11-15 | S5-H01–H25 | `3d35428` | Victor | PASS (259 + 263 checks) | `logs/2026-11-15_host.txt` | Run twice: normal build and APP_BENCH. |
| 2026-11-15 | S5-U01–U03, U05–U07 | `3d35428` | Victor | PASS (6 / 6) | `../integration/logs/2026-11-15_emulator_smp0_test.txt` | **emulator**: kernel objects, mission snapshot without a map, ObstacleTask running, echo conversion. |
