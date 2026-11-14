# Subsystem 1 — test log

**Subsystem:** WiFi/MQTT command & telemetry (CommsTask) and buttons (HmiTask)  ·  **Student ID:** 2503553  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | mbf_telem exists and accepts a record without blocking | IF-12, NFR10 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | worst-case telemetry fits the TX buffer (JSON: 320 B until 25 Oct, 384 B after; binary since 15 Nov: one 27-byte frame) | FR13, NFR9, CR1 | auto | `subsystem1_test.c` | PASS |
| S1-U03 | command parser accepts valid / rejects invalid input (binary frames since 15 Nov) | FR14, IF-11, CR1 | auto | `subsystem1_test.c` | PASS |
| S1-U04 | start accepted only in IDLE | FR14 | auto | `subsystem1_test.c` | PASS |
| S1-U05 | ack format (JSON until 15 Nov; now a 3-byte frame: has-id, 8-bit id, 4-bit error) | IF-11, CR1 | auto | `subsystem1_test.c` | PASS |
| S1-U06 | debounce ignores a 1-sample glitch; 2 s hold = one LONG, no SHORT | FR16 | auto | `subsystem1_test.c` | PASS (a, b) |
| S1-U07 | a delayed QoS 1 redelivery is acknowledged but not re-executed; an id reused for another command is rejected | FR14, Version C gap 9 | auto | `subsystem1_test.c` | PASS |
| S1-H01–H18 | Since 15 Nov (CR1, binary frames; JSON before): bit writer LSB-first layout (H01), buffer bounds at every size (H02), telemetry / event / status / ack byte for byte against golden frames, every field's saturation and none code, the packed 8-byte `mbf_telem` record and its time across the 20-bit wrap (H03–H08), parser (valid, every rejection reason, exact length, every parameter boundary, 20 000 random inputs) (H09–H14), offline ring with packed records (H15), debounce patterns (H16–H17), duplicate-id window with 8-bit ids (H18) | FR13, FR14, FR16, CR1 | auto (host) | `make -C testing/subsystem1/host` (ASan + UBSan) | `host tests: N passed, 0 failed` |
| S1-H19–H20 | `map` command: four 6-bit Code 39 symbols, duplicate / symbol 0 / symbol 63 rejected (`bad_map`), IDLE only; status shows the map or none; bench parameters only in an APP_BENCH build; START needs calibration (`not_calibrated`) | FR4 (instructor 24 Oct), FR14 | host | same | same |
| S1-H21 | the laptop codec (`dashboard/picocar_codec.py`) decodes the C encoder's golden frames back to the test inputs, encodes the same command bytes, ignores non-frames (old JSON, wrong version, truncated), unwraps the 16-bit counters | IF-11, CR1 | auto (host) | `make -C testing/subsystem1/host` runs `test_codec.py` | `host tests: N passed, 0 failed` |
| S1-M07 | barcode map | FR4 | manual | dashboard map panel, or `picocar_codec.py cmd <id> map A F Z W > cmd.bin` and `mosquitto_pub -t picocar/is23/cmd -q 1 -f cmd.bin`, in IDLE | ack ok; retained `status` shows `map` |
| S1-M01 | button level: GP20/GP21 read LOW when pressed with the internal pull-up | FR16 | manual | Flash normal image, press each button, watch the `event` topic | one BTN_START_STOP per GP20 press; BTN_CALIBRATE on GP21 tap; BTN_RESET after 2 s hold |
| S1-M02 | 1 Hz retained status, 5 Hz telemetry | NFR9 | manual | `mosquitto_sub -t 'picocar/is23/#' -F '%t %x' \| python3 dashboard/picocar_codec.py` for 60 s | 60 ± 1 status, 300 ± 3 telemetry, seq without gaps (seq wraps at 256) |
| S1-M03 | Last Will | FR14 | manual | Power the robot off while subscribed | the 1-byte offline frame (`23`, decoded `{"online": false}`) on `status` within ~15 s (1.5 × 10 s keep-alive) |
| S1-M04 | reconnect back-off | FR14, NFR9 | manual | Stop the broker; read the console | retries after 1, 2, 4, 8, 16, 16 s; after restart, connected and delay reset |
| S1-M05 | commands and acks | FR14 | manual | Publish start/stop/reset/set frames (`picocar_codec.py cmd ...`) and bad payloads (command code 3, the removed E-stop, is answered `bad_cmd`; any non-frame `bad_frame`) on `cmd` (QoS 1) | one ack per command with the right `err`; a repeated id is re-acked, not re-executed |
| S1-M06 | 10-minute soak at 5 Hz | A1, A14, NFR9 | manual | Run 10 min near the track | no telemetry seq gaps while connected; `drops` stays 0 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-22 | S1-H01–H17 | feature branch | Victor | PASS (537 checks) | console output of `make -C testing/subsystem1/host` | host PC, gcc with ASan + UBSan |
| 2026-10-22 | S1-U01–U06 (+ S2–S5 base checks) | feature branch | Victor | PASS (17/17) | `logs/2026-10-22_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), not a real board: repeat on the Pico |
| 2026-10-23 | S1-U01–U07 (+ S2–S5 base checks) | feature branch | Victor | PASS (18/18) | `logs/2026-10-23_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`), not a real board: repeat on the Pico |
| 2026-10-25 | S1-H01–H20 | feature/subsystems-2-5 | Victor | PASS (611 checks) | console output of `make -C testing/subsystem1/host` | host PC, gcc with ASan + UBSan |
| 2026-10-25 | S1-U01–U07 (+ S2–S5) | feature/subsystems-2-5 | Victor | PASS (27/27) | `../integration/logs/2026-10-25_emulator_smp0_test.txt` | **emulator**; runner fixed 25 Oct to advance the simulated clock (before, `tk_dly_tsk` never returned, so the 22/23 Oct runs only covered checks that do not sleep) |
| 2026-11-14 | S1-M07 | `cf3e2d7` | Victor | PASS | `../system/logs/2026-11-14_0334_dashboard_map_barcode_F.txt` | Map sent from the dashboard before each run: "Map received", "Map stored" (03:34:18, 03:40:48); the latched barcode used it (F → U-turn). |
| 2026-11-14 | S1-M05 | `cf3e2d7` | Victor | PARTIAL | `../system/logs/2026-11-14_0400_dashboard_start_stop.txt`, `../system/logs/2026-11-14_0424_dashboard_heartbeat_timeout.txt` | Remote start and remote stop acted on (03:34–04:25). Reset, set, bad payloads and a repeated id still to test. |
| 2026-11-15 | S1-H01–H20 | `3d35428` | Victor | PASS (657 + 658 checks) | `logs/2026-11-15_host.txt` | Run twice: normal build and APP_BENCH. Host PC, gcc with ASan + UBSan. |
| 2026-11-15 | S1-U01–U07 | `3d35428` | Victor | PASS (8 / 8) | `../integration/logs/2026-11-15_emulator_smp0_test.txt` | **emulator** (rp2040js, `SMP=0 CONSOLE=uart WIFI=none TEST=1`). |
