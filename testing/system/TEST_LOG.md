# System — test log

Tests of the complete robot against its requirements (design review
sections 1.1 and 1.2), run on the final hardware set-up. One test per
requirement: SYS-01 to SYS-16 are FR1 to FR16, SYS-17 to SYS-28 are NFR1
to NFR12. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.
Dashboard logs are saved from the dashboard's event list (copy all, paste
into `logs/YYYY-MM-DD_HHMM_dashboard_<what>.txt`).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | line following | FR1 | manual | Calibrate, send the map, start. Let the robot run over a practice-track section of at least 2 m with straights and curves. 3 runs. | Never leaves the line; the dashboard log stays in LINE_FOLLOW for each run. |
| SYS-02 | junctions, line loss and line search | FR2 | manual | (a) Drive straight over a cross-junction 10 times. (b) 5 times, move the robot off the line during a run. | (a) "Cross-junction" logged at each crossing. (b) "Line lost", then "Line found"; the robot is back on the line. |
| SYS-03 | barcode decoding | FR3 | manual | Barcode sheets A and Z at the specified size (3 mm narrow bars), 20 mm left of the line. 10 driven passes over each at cruise speed. | Each pass logs "Barcode 'X' read" or "Barcode rejected" with a reason; never a different character. Pass rate: SYS-19. |
| SYS-04 | barcode command at the next junction | FR4 | manual | Send the map from the dashboard. Place a barcode before a cross-junction. Each of the four movements 3 times. | "Barcode latched … at the next junction", then EXECUTE_NAV at the junction; the robot turns as mapped and resumes LINE_FOLLOW. |
| SYS-05 | motion API and control | FR5 | host | `make -C testing/subsystem2/host` | S2-H01 to S2-H15 pass. |
| SYS-06 | hump detection | FR6 | manual | (a) 5 passes over a test hump. (b) 5 hard start / stop runs on flat ground. | (a) "Hump finished" logged each time. (b) No hump logged. |
| SYS-07 | hump height and run summary | FR7 | manual | Humps of known height (SYS-23). Then lift the robot during a run. | Each hump reported with its peak; the run summary gives the highest. A lift leaves the highest hump unchanged. |
| SYS-08 | motion classification | FR8 | manual | Start, cruise, turn at a junction, stop, drive over a hump. | "Motion class" lines follow the sequence: stationary, accelerating, cruising, turning, decelerating, climbing / descending. |
| SYS-09 | obstacle slow-down and stop | FR9 | manual | Box on the line ahead. 5 straight approaches. | "Slowing for an obstacle" below 40 cm; at rest before 20 cm (distance: SYS-22). |
| SYS-10 | obstacle sweep and width | FR10 | manual | 15 cm box centred, 5 cm left and 5 cm right of the line, 3 runs each. Tape-measure the box. | Logged width within ±3 cm of the measured width. |
| SYS-11 | bypass | FR11 | manual | The SYS-10 set-up, 3 runs each position. | The side taken matches the logged plan ("Bypass plan"); no contact with the box. |
| SYS-12 | rejoining the line | FR12 | manual | The SYS-11 runs. | At least 2 of the 3 runs at each position end in LINE_FOLLOW on the original line. |
| SYS-13 | telemetry content | FR13 | manual | One complete run with the dashboard (or `mosquitto_sub -v -t 'picocar/is23/#'`) logging. | State, wheel speeds, encoder counts, distance, line status, barcode, hump and obstacle fields all present and changing. |
| SYS-14 | remote commands, heartbeat, reconnection | FR14 | manual | From the dashboard: start, stop, reset, map, set. Then stop and restart the broker. | Each command acknowledged and acted on (start / map / set only in IDLE). Status goes offline (Last Will), then online again after the restart. |
| SYS-15 | safety stops | FR15 | manual | During a run: lift the robot; tilt it past 30°; hand closer than 8 cm in front; hold one wheel. A missed heartbeat is also covered when one occurs. | Each time: motors stop at once, FAULT with the right code (1 lifted, 8 overturned, 2 too close, 4 stall, 32 heartbeat); stays stopped until reset. A lift also publishes the run summary. |
| SYS-16 | calibration and buttons | FR16 | manual | Press start before calibrating; calibrate (GP21); start again. Check GP20 start / stop and a GP21 long press (reset). | "Start refused: not calibrated" before calibration; the run starts after it; buttons act as described. |
| SYS-17 | control timing | NFR1 | auto + manual | (a) On-target self-tests S2-U03, S2-U04 (`make TEST=1`). (b) 10-minute run with telemetry at 5 Hz. | (a) PASS. (b) No "Heartbeat missed" in the log. |
| SYS-18 | IR sampling | NFR2 | auto + manual | (a) On-target self-test S3-U03. (b) The SYS-01 runs. | (a) PASS. (b) No LineBarcodeTask heartbeat fault (missed-task mask 2). |
| SYS-19 | barcode reliability and decode time | NFR3 | manual | The SYS-03 passes. | At least 7 of 10 correct per sheet; no wrong command executed; every "Barcode decode time" ≤ 50 000 µs. |
| SYS-20 | motion accuracy | NFR4 | manual | `testing/subsystem2/MOTION_ACCURACY_EVALUATION.md`, 5 runs each. | 1 m within ±3 % and drift ≤ 50 mm; 90° within ±8°; 180° within ±10°. |
| SYS-21 | speed control | NFR5 | manual | `testing/subsystem2/PID_TUNING_REPORT.md`: speed steps 100 / 200 / 300 mm/s, 3 each; `step_analysis.py`. | Steady speed within ±10 %; within ±10 % ≤ 500 ms after the set-point ramp ends. |
| SYS-22 | obstacle stopping distance | NFR6 | manual | The SYS-09 approaches; tape-measure from the robot's front to the box. | At rest ≥ 10 cm from the box every time. |
| SYS-23 | hump height accuracy | NFR7 | manual | Humps of known height, 5 passes each. | Peak within ±30 % or ±5 mm, whichever is larger. |
| SYS-24 | safety response and watchdog | NFR8 | manual + review | (a) The SYS-15 lift and tilt tests. (b) Code review of the watchdog set-up. | (a) Wheels stop at once, FAULT logged. (b) Watchdog loaded for 200 ms, fed only by SafetyTask; S4-U06 PASS. |
| SYS-25 | communications timing | NFR9 | manual | 10-minute logged run; switch the broker and the access point off and on during a run. | Telemetry every 200 ms, status every 1 s; reconnects (1–16 s back-off, WiFi re-join 5 s); the run continues throughout. |
| SYS-26 | resources | NFR10 | review | Release build: `arm-none-eabi-size`; search for heap calls; check every `mbf_telem` post. | No malloc / free in `app_program`; every telemetry post uses TMO_POL; RAM well inside 264 KB. |
| SYS-27 | robustness | NFR11 | manual | A run over a 40 mm gap and a tape joint; a run with the access point switched off. | The robot keeps following the line; the run continues without WiFi. |
| SYS-28 | software quality | NFR12 | auto | Clean release build; all host tests; on-target self-tests (emulator). | 0 warnings in `app_program`; host tests pass; self-tests pass. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = host tests on a PC; **review** = code or build inspection;
**manual** = a person observes the result on the robot (dashboard log,
tape measure, protractor).

## Results

Add a new row for every run. Never edit or delete old rows.
Car logs of 14 Nov: firmware `4d98d83` before 02:52, `cf3e2d7` from 02:52
to 18:10 (barcode end-speck and decode-time fix `673f9e2` after that).

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-14 | SYS-15 | `4d98d83` | Victor | PASS (part: overturn, lift) | `logs/2026-11-14_0244_dashboard_lift_overturn.txt` | Overturned (code 8) at 02:44:49 and 02:45:28; lifted (code 1, IMU, 298 mg) at 02:45:10 with run summary. Motors braked each time. Too close and stall not yet tested. |
| 2026-11-14 | SYS-15 | `cf3e2d7` | Victor | PASS (part: lift by IR) | `logs/2026-11-14_1625_dashboard_lift_no_floor.txt` | 16:25:35 "No floor under the IR sensors" (all three IR 1000 for 40 mm), Lifted (code 1), run summary. |
| 2026-11-14 | SYS-15 | `cf3e2d7` | Victor | PASS (part: heartbeat) | `logs/2026-11-14_0424_dashboard_heartbeat_timeout.txt`, `logs/2026-11-14_1711_dashboard_barcode_A_heartbeat.txt` | A real missed heartbeat (LineBarcodeTask, mask 2) at 04:24:48, 17:11:31 and 17:12:03: FAULT code 32, robot stopped. Cause: SYS-17. |
| 2026-11-14 | SYS-16 | `cf3e2d7` | Victor | PASS | `logs/2026-11-14_1711_dashboard_barcode_A_heartbeat.txt` | Calibration failed at 17:11:47 (IMU shake 68 > 60); start refused at 17:11:48 and 17:11:49; calibrated at 17:11:57; run started 17:11:59. |
| 2026-11-14 | SYS-14 | `cf3e2d7` | Victor | PASS (part: start, stop, map) | `logs/2026-11-14_0334_dashboard_map_barcode_F.txt`, `logs/2026-11-14_0400_dashboard_start_stop.txt`, `logs/2026-11-14_0424_dashboard_heartbeat_timeout.txt` | Map received and stored, remote start, remote stop → MISSION_COMPLETE with run summary (03:34–04:25). Reset, set and broker restart not yet tested. |
| 2026-11-14 | SYS-04 | `cf3e2d7` | Victor | PARTIAL | `logs/2026-11-14_0334_dashboard_map_barcode_F.txt` | 03:34:28 and 03:40:58: "Barcode 'F' read", latched "F → U-turn at the next junction", EXECUTE_NAV at the next junction. That junction was detected on the plain line just after the barcode (see SYS-02), so the U-turn came early. |
| 2026-11-14 | SYS-02 | `cf3e2d7` | Victor | PARTIAL (line loss) | `logs/2026-11-14_1625_dashboard_lift_no_floor.txt`, `logs/2026-11-14_1711_dashboard_barcode_A_heartbeat.txt` | Line lost → LINE_SEARCH → line found at 16:25:28, 16:25:57 and 17:11:30. "Cross-junction" also logged on plain line during runs (03:34–04:25): to be fixed before the 10 junction crossings. |
| 2026-11-14 | SYS-03 | `cf3e2d7` | Victor | PARTIAL | `logs/2026-11-14_0334_dashboard_map_barcode_F.txt`, `logs/2026-11-14_1711_dashboard_barcode_A_heartbeat.txt`, `logs/2026-11-14_1737_dashboard_barcode_A_best_match.txt` | F decoded at 03:34:28 and 03:40:58 (print size not recorded). With the 1 mm slit, 3 mm A: full code seen at 17:11:31 and 17:12:03 but rejected (end specks, fixed in `673f9e2`, replayed in S3-H20), decoded by best match at 17:37:24. No wrong character in any log. |
| 2026-11-14 | SYS-09 | `cf3e2d7` | Victor | PARTIAL | `logs/2026-11-14_0351_dashboard_obstacle_slowdown.txt` | "Slowing for an obstacle" at 39.8 / 39.6 / 39.6 cm on three approaches. A second "Slowing" line reads −0.1 cm (03:51:48, 03:52:45): display value to check. Stop distance not measured yet (SYS-22). |
| 2026-11-14 | SYS-17 | `cf3e2d7` | Victor | FAIL | `logs/2026-11-14_0424_dashboard_heartbeat_timeout.txt`, `logs/2026-11-14_1711_dashboard_barcode_A_heartbeat.txt`, `logs/2026-11-14_1737_dashboard_barcode_A_best_match.txt` | "Heartbeat missed" (LineBarcodeTask) right after a barcode capture: the best-match decoder ran on up to four readings and took over 50 ms. Fixed in `673f9e2` (one reading, early cut-off, decode time logged). Re-run the 10-minute run. |
| 2026-11-14 | SYS-18 | `cf3e2d7` | Victor | FAIL | as SYS-17 | Same cause and fix as SYS-17. |
| 2026-11-15 | SYS-05 | `3d35428` | Victor | FAIL (56 / 57) | `../subsystem2/logs/2026-11-15_host.txt` | S2-H03: the 90° turn on the motor model ends at 92.1° against a ±1° limit (still within NFR4's ±8°). Every other S2 host test passes, including S2-H14 (named API) and S2-H15 (speed recorder). |
| 2026-11-15 | SYS-17 | `3d35428` | Victor | PASS (part a) | `../integration/logs/2026-11-15_emulator_smp0_test.txt` | S2-U03, S2-U04 PASS in the emulator. Part (b) to re-run on the robot. |
| 2026-11-15 | SYS-18 | `3d35428` | Victor | PASS (part a) | `../integration/logs/2026-11-15_emulator_smp0_test.txt` | S3-U03 PASS (~10 ADC blocks per 100 ms). |
| 2026-11-15 | SYS-24 | `3d35428` | Victor | PASS | `../integration/logs/2026-11-15_emulator_smp0_test.txt`; review of `subsystem4/safety_task.c`, `drv_watchdog.c` | `SAFETY_WD_TIMEOUT_MS` 200, load = ms × 1000 × 2 (RP2040-E1); `drv_watchdog_feed()` called only in `safety_task.c`; S4-U06 PASS. Brake at once in the SYS-15 lift / tilt runs. |
| 2026-11-15 | SYS-26 | `3d35428` | Victor | PASS | `logs/2026-11-15_release_build.txt` | No malloc / calloc / realloc / free in `app_program`; the one `mbf_telem` send (`subsystem1.c`, `comms_telem_post`) uses TMO_POL. RAM 101 900 B (data + bss, including lwIP and the template) of 264 KB. |
| 2026-11-15 | SYS-28 | `3d35428` | Victor | PARTIAL | `logs/2026-11-15_release_build.txt`, `../*/logs/2026-11-15_host.txt`, `../integration/logs/2026-11-15_emulator_smp0_test.txt` | 41 `app_program` files, 0 warnings. Host tests 2 129 passed, 1 failed (S2-H03, see SYS-05; S1 and S5 run twice, normal and APP_BENCH). On-target self-tests 27 / 27 PASS. |
