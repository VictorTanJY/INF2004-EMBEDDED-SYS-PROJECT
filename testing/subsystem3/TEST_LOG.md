# Subsystem 3 — test log

**Subsystem:** IR line following & Code 39 barcode (LineBarcodeTask)  ·  **Student ID:** 2500519  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results, and
`docs/SETUP_AND_TEST.md` for the bring-up order on the robot.

## Test cases

Define a test here before running it. Test IDs never change (retired IDs stay listed).

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | *retired 25 Oct:* line snapshot round trip (LineBarcodeTask now writes it continuously) | IF-04 | – | – | – |
| S3-U02 | LineBarcodeTask publishes its snapshot | IF-04 | auto | `subsystem3_test.c` | PASS |
| S3-U03 | ADC + chained DMA deliver ~10 blocks per 100 ms (1 kHz per channel, 3 channels) | NFR2 | auto | `subsystem3_test.c` | PASS |
| S3-U04 | Code 39 `*F*` decodes on the target CPU | FR3 | auto | `subsystem3_test.c` | PASS |
| S3-U05 | 100 ADC blocks per second, none lost; live IR readings | NFR2 | auto | `subsystem3_test.c`; 1 s window. Prints blocks, lost blocks and the three normalised readings | 95–105 blocks, 0 lost, readings 0–1000 |
| S3-U06 | *not used* | – | – | – | – |
| S3-U07 | barcode `*A*` through the whole `bc_sample` pipeline on the target, timed | FR3, NFR3 | auto | synthetic clean bars (narrow 3 mm, wide 7.5 mm, levels 80 / 920), 100 mm/s, 1 kHz, 1.2 m. Prints result, pass, slowest and mean call time | one result, `A`; slowest `bc_sample` call < 10 ms (one ADC block) |
| S3-H01–H07 | Code 39: full 44-character table; the instructor's sample sheets (A, F, Z at all three printed sizes, W, X, Y; `vectors_slides.h` from BARCODE.pptx) at screen and A4 scale, forwards and reversed; all 43 data characters at 3 narrow widths x 3 ratios x 2 directions with 15 % jitter; corrupted wide bar never decodes as another character; 1 kHz sample-stream capture at 0.15 / 0.30 m/s with spot blur and noise (>= 95 %, 0 wrong); crossing line ignored; long patch rejected | FR3, NFR3, A6 | host | `make -C testing/subsystem3/host` | `host tests: N passed, 0 failed` |
| S3-H08–H12 | tracking: steering sign; junction once on a straight, not on a tight curve; 40 mm gap bridged, 90 mm LOST once, FOUND; drift off one side turns back; calibration contrast >= 3:1 and normalisation | FR1, FR2, FR16, NFR11 | host | same | same |
| S3-M01 | raw ADC levels over black and white, sensor heights, lighting | A7 | manual | calibration spin, `LOG 32–34` (guide step F3) | contrast >= 3:1 on both line sensors; set `LINE_BLACK_IS_HIGH` |
| S3-M02 | calibration spin | FR16 | manual | robot on the line, GP21 tap | event log `CAL_DONE` data 1 from src 3; `calibrated` bit 1 |
| S3-M03 | line following, 3 runs over a 2 m practice-track section (straights and curves) | FR1, NFR11 | manual | start, observe, save the dashboard log | no intervention; stays in LINE_FOLLOW |
| S3-M04 | junctions and line loss: 10 cross-junction crossings, 5 line-loss tests | FR2 | manual | event log | "Cross-junction" at each crossing; LINE_LOST then LINE_FOUND after each loss |
| S3-M05 | barcodes: 10 passes each over the A and Z sheets (3 mm narrow bars) at cruise speed | FR3, NFR3 | manual | event log: "Barcode 'X' read", "Barcode rejected", "Barcode decode time" | >= 7 / 10 correct per sheet, 0 wrong commands, decode time <= 50 000 us |
| S3-M06 | ADC block timing | NFR2 | manual | `APP_TIMING_PINS=1`, GP7 on a logic analyser; `missed_blocks` in the snapshot | 10 ms blocks, 0 missed |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = pure-logic test on a PC (`make -C testing/subsystem3/host`, gcc with ASan + UBSan);
**manual** = a person observes the result on the robot (scope, tape measure, MQTT log...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S3-H* | feature/subsystems-2-5 | Victor | PASS | console output of `make -C testing/subsystem3/host` | host PC, gcc with ASan + UBSan; logic only, on models |
| 2026-10-25 | S3-U* | feature/subsystems-2-5 | Victor | PASS | `../integration/logs/2026-10-25_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), core 0 only, no sensors: repeat on the Pico |
| 2026-11-14 | S3-M05 | `cf3e2d7` | Victor | PARTIAL | `../system/logs/2026-11-14_0334_dashboard_map_barcode_F.txt`, `../system/logs/2026-11-14_1711_dashboard_barcode_A_heartbeat.txt`, `../system/logs/2026-11-14_1737_dashboard_barcode_A_best_match.txt` | F read at 03:34:28 and 03:40:58 (print size not recorded). 3 mm A with the 1 mm slit: whole code seen twice (31 and 33 elements, end specks) but rejected; decoded by best match at 17:37:24. No wrong character. The 10 + 10 passes are still to run with `673f9e2`. |
| 2026-11-14 | S3-M02 | `cf3e2d7` | Victor | FAIL | all `../system/logs/2026-11-14_*` | Calibration spin: "right sensor 0, not measured: turns not corrected" in every calibration of the day; the left sensor saw 1–2 crossings. Right line sensor position / height to check. |
| 2026-11-15 | S3-H01–H21 | `3d35428` | Victor | PASS (157 checks) | `logs/2026-11-15_host.txt` | Includes S3-H20: the two 14 Nov car reads of *A* (end specks) decode as A, 8 / 8. |
| 2026-11-15 | S3-U02–U04 | `3d35428` | Victor | PASS (3 / 3) | `../integration/logs/2026-11-15_emulator_smp0_test.txt` | **emulator**: snapshot, ADC + DMA blocks (~10 per 100 ms), *F* decode on target. |
| 2026-11-15 | S3-U02–U05, U07 | `843f105` | Victor | PASS (5 / 5) | `../integration/logs/2026-11-15_emulator_smp0_test_843f105.txt` | **emulator**: 101 blocks in 1 s, 0 lost (readings 0: no sensors). `*A*` decoded once (pass 0); slowest call 132 µs, mean 4 µs over 12 000 samples (emulated clock). Run on the robot for real readings and timing. |
