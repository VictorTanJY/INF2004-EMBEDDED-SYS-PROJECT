# Subsystem 1 — test log

**Subsystem:** WiFi/MQTT command & telemetry (CommsTask) and buttons (HmiTask)  ·  **Student ID:** 2503553  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | mbf_telem exists and accepts a record without blocking | IF-12, NFR10 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | worst-case telemetry fits the 320 B TX buffer | FR13, NFR9 | auto | `subsystem1_test.c` | PASS |
| S1-U03 | command parser accepts valid / rejects invalid input | FR14, IF-11 | auto | `subsystem1_test.c` | PASS |
| S1-U04 | start accepted only in IDLE | FR14 | auto | `subsystem1_test.c` | PASS |
| S1-U05 | ack JSON format | IF-11 | auto | `subsystem1_test.c` | PASS |
| S1-U06 | debounce ignores a 1-sample glitch; 2 s hold = one LONG, no SHORT | FR16 | auto | `subsystem1_test.c` | PASS (a, b) |
| S1-H01–H17 | JSON writer, buffer bounds at every size, telemetry/event/status/ack exact strings, parser (valid, every rejection reason, every parameter boundary, 20 000 random inputs), offline ring, debounce patterns | FR13, FR14, FR16 | auto (host) | `make -C testing/subsystem1/host` (ASan + UBSan) | `host tests: N passed, 0 failed` |
| S1-M01 | button level: GP20/GP21 read LOW when pressed with the internal pull-up | FR16 | manual | Flash normal image, press each button, watch the `event` topic | one BTN_START_STOP per GP20 press; BTN_CALIBRATE on GP21 tap; BTN_RESET after 2 s hold |
| S1-M02 | 1 Hz retained status, 5 Hz telemetry | NFR9 | manual | `mosquitto_sub -v -t 'picocar/is23/#'` for 60 s | 60 ± 1 status, 300 ± 3 telemetry, seq without gaps |
| S1-M03 | Last Will | FR14 | manual | Power the robot off while subscribed | `{"online":false}` on `status` within ~15 s (1.5 × 10 s keep-alive) |
| S1-M04 | reconnect back-off | FR14, NFR9 | manual | Stop the broker; read the console | retries after 1, 2, 4, 8, 16, 16 s; after restart, connected and delay reset |
| S1-M05 | commands and acks | FR14 | manual | Publish start/stop/estop/reset/set and bad payloads on `cmd` (QoS 1) | one ack per command with the right `err`; a repeated id is re-acked, not re-executed |
| S1-M06 | 10-minute soak at 5 Hz | A1, A14, NFR9 | manual | Run 10 min near the track | no telemetry seq gaps while connected; `drops` stays 0 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-22 | S1-H01–H17 | feature branch | Claude (for Victor) | PASS (537 checks) | console output of `make -C testing/subsystem1/host` | host PC, gcc with ASan + UBSan |
| 2026-10-22 | S1-U01–U06 (+ S2–S5 base checks) | feature branch | Claude (for Victor) | PASS (17/17) | `logs/2026-10-22_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), not a real board: repeat on the Pico |
