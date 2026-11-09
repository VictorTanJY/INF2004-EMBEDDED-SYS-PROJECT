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
| S3-H01–H07 | Code 39: full 44-character table; the instructor's sample sheets (A, F, Z at all three printed sizes, W, X, Y; `vectors_slides.h` from BARCODE.pptx) at screen and A4 scale, forwards and reversed; all 43 data characters at 3 narrow widths x 3 ratios x 2 directions with 15 % jitter; corrupted wide bar never decodes as another character; 1 kHz sample-stream capture at 0.15 / 0.30 m/s with spot blur and noise (>= 95 %, 0 wrong); crossing line ignored; long patch rejected | FR3, NFR3, A6 | host | `make -C testing/subsystem3/host` | `host tests: N passed, 0 failed` |
| S3-H08–H12 | tracking: steering sign; junction once on a straight, not on a tight curve; 40 mm gap bridged, 90 mm LOST once, FOUND; drift off one side turns back; calibration contrast >= 3:1 and normalisation | FR1, FR2, FR16, NFR11 | host | same | same |
| S3-M01 | raw ADC levels over black and white, sensor heights, lighting | A7 | manual | calibration spin, `LOG 32–34` (guide step F3) | contrast >= 3:1 on both line sensors; set `LINE_BLACK_IS_HIGH` |
| S3-M02 | calibration spin | FR16 | manual | robot on the line, GP21 tap | event log `CAL_DONE` data 1 from src 3; `calibrated` bit 1 |
| S3-M03 | line following, 5 laps of the practice track | FR1, NFR11 | manual | start, observe | no intervention |
| S3-M04 | junction / gap track, 10 passes incl. 40 mm gaps and the tightest curve | FR2 | manual | event log | one JUNCTION per crossing, no false JUNCTION / LINE_LOST |
| S3-M05 | barcodes: 20 passes per character (A, F, Z, W, X, Y sheets) at 0.15 and 0.30 m/s, both directions | FR3, NFR3 | manual | event log `BARCODE` data | >= 95 % correct, 0 wrong commands |
| S3-M06 | ADC block timing | NFR2 | manual | `APP_TIMING_PINS=1`, GP7 on a logic analyser; `missed_blocks` in the snapshot | 10 ms blocks, 0 missed |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = pure-logic test on a PC (`make -C testing/subsystem3/host`, gcc with ASan + UBSan);
**manual** = a person observes the result on the robot (scope, tape measure, MQTT log...).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-25 | S3-H* | feature/subsystems-2-5 | Claude (for the team) | PASS | console output of `make -C testing/subsystem3/host` | host PC, gcc with ASan + UBSan; logic only, on models |
| 2026-10-25 | S3-U* | feature/subsystems-2-5 | Claude (for the team) | PASS | `../integration/logs/2026-10-25_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), core 0 only, no sensors: repeat on the Pico |
