# PicoCar — setup and test guide

How to go from a bare Pico W to a demo run, one subsystem at a time. Follow
the stages in order: each one assumes the previous one passed. Record every
result in that subsystem's `testing/subsystemN/TEST_LOG.md` (the test IDs
below, e.g. **S2-M02**, are defined there).

**Status of the code (25 Oct 2026).** All five subsystems are implemented
against the Week 6 design review (revised 22 Oct), Tribunal 1 Version C and
the instructor's forum answers of 23–24 Oct. The code has been built (three
build variants, zero warnings), tested on a PC (logic, on models) and run in
the rp2040js emulator (single core, no sensors). **Nothing has run on the
robot yet.** Every tuning value marked `MEASURE` in a `*_cfg.h` file is a
placeholder until you measure it in the stages below.

---

## 0. Before you start

### You need

| Item | Used for |
|---|---|
| The robot, fully assembled but **with the sensors unplugged** | stages are wired one at a time |
| USB cable, a laptop with the Pico SDK 2.2.0 toolchain (README section 1) | build and flash |
| A serial terminal at 115200 (PuTTY, `screen`, VS Code serial monitor) | console and `[TEST]` lines |
| Mosquitto broker + `mosquitto_sub` / `mosquitto_pub` on the laptop, on the lab WiFi | MQTT (stage 2 onwards) |
| Multimeter; logic analyser or scope (any cheap 8-channel analyser works) | pin levels, PWM, timing |
| Tape measure, protractor, ruler, a few boxes 10 / 15 / 20 cm wide | calibration |
| The printed barcode sheets (A, F, Z, W, X, Y) and a strip of track with a junction and a gap | stage 5 |

### Three rules that protect the hardware

1. **Wheels off the ground** (robot on a box) the first time you drive the motors.
2. **Measure before you connect** any sensor output to a Pico pin: it must be
   3.3 V, never 5 V (encoders A4, HC-SR04P A11).
3. **UART-console builds** (`CONSOLE=uart` or `CRASH_REPORT=1`) use GP0/GP1
   for the console: unplug the HC-SR04P for those. The firmware then leaves
   GP0/GP1 alone and the ultrasonic sensor simply does nothing.

### Build switches you will flip during bring-up

| Where | Switch | Default | Set it to |
|---|---|---|---|
| `app_program/app_config.h` | `APP_BENCH` | 0 | **1** for stages 3–6: drive the hardware from the laptop over MQTT |
| `app_program/app_config.h` | `APP_TIMING_PINS` | 0 | **1** when you capture timing on GP6 / GP7 (S2-M07, S3-M06) |
| `app_program/subsystem5/mission_cfg.h` | `MISSION_SELFTEST_MASK` | `0x0F` | **`0x0B`** until the IMU is wired (stage 4). Without the GY-511 the boot self-test fails and the robot stays in FAULT |

Put all three back to their defaults before a demo or a graded run.

### Build and flash

```sh
export PICO_SDK_PATH=/path/to/pico-sdk          # 2.2.0
cp config/wifi_secrets.example.h config/wifi_secrets.h   # then edit SSID, password, broker IP
cd build_make
make -j8 NET=lwip MQTT=1            # normal image:   mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
make -j8 NET=lwip MQTT=1 TEST=1     # + self-tests:   mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test.uf2
```

Hold **BOOTSEL**, plug in USB, copy the `.uf2` onto the drive (README section 2).
The file name always tells you which options it has.

Host tests (no Pico needed), from the repo root:

```sh
for d in shared subsystem1 subsystem2 subsystem3 subsystem4 subsystem5; do make -C testing/$d/host; done
```

---

## Stage 1 — bare board (nothing wired except the motors' own cables)

Goal: the image boots, nothing is driven that should not be, and every
subsystem's self-test passes on the real chip.

1. Set `MISSION_SELFTEST_MASK` to `0x0B` (no IMU yet). Build the **TEST=1**
   image, flash it, open the serial terminal.
2. Check the console shows `all subsystems started`, then the `[TEST]` lines,
   then `=== test run end ===`. Expected: every line `PASS`, and
   `[INFO] timer: 100000 us per 100 ms of CPU clock` (±2 %). **S4-U05** failing
   means the 1 µs timer is not at 1 MHz — stop and investigate, everything
   timed depends on it.
   - `S4-U04` (ImuTask snapshot) passes even without an IMU; the snapshot
     just says `ok = 0`.
3. With a multimeter or analyser (**S2-M01**): GP8, GP9, GP10, GP11 are LOW
   (motor driver brake), GP0 LOW, GP15 shows a 50 Hz pulse of ~1.5 ms (servo
   centre).
4. Unplug USB, power from the power bank, check the robot boots the same.

Record: S1–S5 `U` tests in each TEST_LOG with "hardware" in the notes.

## Stage 2 — laptop link (subsystem 1, Victor)

Goal: MQTT works, because every later stage uses it to see what the robot does.

1. Start the broker on the laptop (`mosquitto -v`, listening on 1883, the IP
   in `wifi_secrets.h`). Flash the **normal** image.
2. Watch everything: `mosquitto_sub -v -t 'picocar/is23/#'`
   - `status` (retained, 1 Hz): `{"online":true,...,"map":null}` (**S1-M02**)
   - `telemetry` (5 Hz), `event` (state changes and events as they happen).
     After boot you should see `STATE` `IDLE`.
3. Send the barcode map the way the instructor will (**S1-M07**, **S5-M07**):
   ```sh
   mosquitto_pub -q 1 -t picocar/is23/cmd -m '{"id":1,"cmd":"map","left":"A","right":"F","straight":"Z","uturn":"W"}'
   ```
   Expect an ack `{"id":1,"ok":true}` on `picocar/is23/ack`, and `status.map`
   showing the four letters. Wrong inputs (a missing movement, a letter used
   twice, `*`) are answered with `"err":"bad_map"`. The map is only accepted
   in IDLE and is lost at reboot — send it again after a power cycle.
4. Run S1-M03 (Last Will), S1-M04 (broker stop/start back-off), S1-M05
   (each command and error), S1-M06 (10-minute soak).
5. Buttons (**S1-M01**): GP20 press → `BTN_START_STOP` event; GP21 tap →
   `BTN_CALIBRATE`; GP21 held 2 s → `BTN_RESET`.

Useful commands (all need `"id"`, use a new id each time):

| Purpose | Payload on `picocar/is23/cmd` |
|---|---|
| start a run | `{"id":2,"cmd":"start"}` (refused with `not_calibrated` until the stage 5 calibration) |
| stop | `{"id":3,"cmd":"stop"}` |
| emergency stop | `{"id":4,"cmd":"estop"}` |
| reset after a fault | `{"id":5,"cmd":"reset"}` |
| run parameter (IDLE) | `{"id":6,"cmd":"set","param":"speed_mm_s","value":200}` — also `slow_cm`, `stop_cm`, `lost_mm`, `junction_mm` |
| bench (APP_BENCH=1, IDLE) | `{"id":7,"cmd":"set","param":"bench_move","value":300}` — also `bench_turn` (0.1 deg), `bench_speed` (mm/s), `bench_servo` (deg) |

## Stage 3 — motors and encoders (subsystem 2, Jedrek)

Robot **on a box, wheels free**. Set `APP_BENCH 1`, flash the normal image.

1. **Encoder levels first (A4).** Plug only the encoder power (3V3, GND) into
   Grove 3 / Grove 4, leave the A/B wires off the Pico. Turn each wheel by
   hand and measure A and B with the multimeter or scope: they must swing
   between 0 and 3.3 V, never 5 V. Then plug the A/B wires in.
2. **Counts per revolution (S2-M02):** mark each wheel, turn it exactly 10
   revolutions forward by hand, read `enc` in telemetry. Set
   `MOTION_COUNTS_PER_REV` (counts / 10) in `subsystem2/motion_cfg.h`. If the
   count goes **down** when the wheel turns forward, flip `MOTION_ENC_SIGN_L/_R`.
3. **Direction:** `bench_move` 200. Both wheels must turn forward. If one runs
   backwards, flip its `MOTION_MOT_SIGN_*`. If a wheel runs away (speed keeps
   rising, then a STALL fault), its encoder sign is wrong.
4. **PWM (S2-M01):** analyser on GP8–GP11 during the move: 20 kHz, only one
   pin of each pair active.
5. Put the robot on the floor. **Distance (S2-M03):** `bench_move` 1000, five
   times, tape measure. Scale `MOTION_WHEEL_CIRC_UM` by measured / commanded.
   Lateral drift over 1 m ≤ 30 mm (NFR5).
6. **Turns (S2-M04):** `bench_turn` 900 and 1800, ten each. Scale
   `MOTION_TRACK_MM` by commanded / measured angle (a robot that turns too far
   has a track width set too small).
7. **Speed loop (S2-M05):** `bench_speed` 250. Analyser on GP4 (left encoder
   A): the pulse rate is the wheel speed against time (telemetry at 5 Hz is
   too slow for this). Tune `MOTION_KFF_MILLI`, `MOTION_KP_MILLI`,
   `MOTION_KI_MILLI`, `MOTION_MIN_DUTY` until it settles within 300 ms with
   < 10 % error. Start with feed-forward only (KP = KI = 0): the speed should
   already be close.
8. **Stall (S2-M06):** during a `bench_move` 2000, hold one wheel: a `FAULT`
   event with data 4 (STALL) after 300–400 ms, motors braked. Reset with GP21
   held 2 s or `{"cmd":"reset"}`.
9. **Timing (S2-M07):** `APP_TIMING_PINS 1`, analyser on GP6 for 60 s while
   telemetry runs: every edge 10 ms ± 1 ms (NFR1). If not, see A2 (pace the
   loop from a TIMER alarm on core 1).

## Stage 4 — IMU and safety (subsystem 4, Purvez)

The IMU comes before the line sensors because a run (`start`) needs **both**
the IR and the IMU calibration.

1. Wire the GY-511 to Grove 2 (GP2 SDA, GP3 SCL) and INT1 to GP19; mount it
   ≥ 5 cm from the motors. Set `MISSION_SELFTEST_MASK` back to **`0x0F`**.
2. **Found (S4-M01):** boot reaches `IDLE`, not FAULT. If it goes to FAULT
   (cause self-test), `LOG 62` with data `0x01040000` names the IMU: check
   the wiring and pull-ups.
3. **Axes (S4-M02):** tilt the nose up by hand: `pitch_deg` in telemetry goes
   positive. Lift the left side: roll goes positive (IMU snapshot). Otherwise
   fix `IMU_AXIS_*` / `IMU_SIGN_*` in `subsystem4/imu_cfg.h`.
4. **Pitch noise (S4-M03):** with `bench_move` over the flat track, pitch
   stays within ±1°.
5. **Safety (S4-M05 to S4-M08):** tilt > 30° → FAULT, and a reset is refused
   while still tilted. A firm tap on the chassis (INT1) brakes the motors. An
   MQTT `estop` brakes them. With the analyser, time detection → GP8–GP11
   LOW: ≤ 20 ms (NFR8). A `FAULT` event's data is the fault bit mask
   (1 impact, 2 too close, 4 stall, 8 tilt, 16 E-stop, 32 heartbeat).
6. **Power (S4-M09, A10):** scope V+ and 3V3 while starting the motors with
   the servo sweeping (`bench_servo`); no resets.
7. Hump height (S4-M04) needs line following: do it at the end of stage 5.

## Stage 5 — IR sensors, line and barcodes (subsystem 3, Boon)

1. Wire the two line sensors (Grove 5 GP26, Grove 6 GP27 — insulate Grove 6's
   GP26 wire) and the barcode sensor (Grove 7 GP28). VBAT jumper open.
   Line sensors 3–8 mm above the track, centres ~18 mm apart, straddling the
   line edges; barcode sensor ~3 mm high, ~45 mm right of the line centre
   (set `LINE_BC_OFFSET_MM`).
2. **Calibration and raw levels (S3-M01, S3-M02, A7, FR16):** robot centred on
   a straight line, GP21 tap. It stands still 1 s (IMU level offsets), spins
   360° at 90°/s (IR black/white, magnetometer), then logs:
   - `LOG 32 / 33 / 34` (left, right, barcode sensor): `data` = highest raw
     ADC value << 16 | lowest. Black / white must differ by ≥ 3:1 on the two
     line sensors; if not, change the sensor height or shade them.
   - `LOG 62`: `data` = calibrated bits << 16 — `0x30000` = IR and IMU both
     good (bit 0 IR, bit 1 IMU).
   The barcode sensor only crosses the line briefly in the spin; its black
   level is refined on every barcode read.
3. **Line following (S3-M03):** `start`. If the robot steers away from the
   line, flip `LINE_STEER_SIGN`; if it seems to see the line where there is
   none, the module reads lower over black: set `LINE_BLACK_IS_HIGH 0`. Tune
   `LINE_KP`, `LINE_KD` (`line_cfg.h`) and `speed_mm_s`. 5 laps without help.
4. **Junctions and gaps (S3-M04):** one `JUNCTION` event per crossing, none on
   curves or 40 mm gaps. `junction_mm` defaults to **12 mm**, not FR2's
   20 mm: a crossing line is only 18 mm wide, so both sensors are fully black
   for less than 20 mm. Tune it with `{"cmd":"set","param":"junction_mm",...}`.
5. **Barcodes (S3-M05, NFR3):** send the map, start, drive over each sheet.
   `event` shows `BARCODE` with the character (data `"A"` etc.). Every
   character A–Z decodes, mapped or not; only mapped ones are latched
   (`LOG 63`, telemetry `nav`), unmapped ones give `LOG 64`. Do 20 passes per
   character at 150 and 300 mm/s, both directions; the sheets come in three
   print sizes and all three must read. Rejected reads give `LOG 31` (data =
   reason: 2 count, 3 ratio, 4 pattern, 5 start/stop, 6 length, 7 too long).
6. **Barcode → junction (S5-M07):** with the map sent, every latched
   character is executed at the next cross-junction (left / right / straight
   / U-turn), then line following resumes.
7. **Timing (S3-M06):** `APP_TIMING_PINS 1`, GP7 toggles every 10 ms block;
   snapshot `missed_blocks` stays 0 (NFR2).
8. **Humps (S4-M04, NFR7):** 10 passes over humps of known height. Each
   gives `HUMP_START`, then `HUMP_END` with the peak in mm; telemetry
   `hump_max_mm`; and the run summary (`LOG 67`, data = highest peak) when
   you stop. Known limit: the climb threshold is +5°, which a 10 mm × 300 mm
   hump does not reach with this chassis (host test S4-H01b). If the course
   has humps that low, lower `IMU_HUMP_ON_CDEG` only after S4-M03 shows the
   noise allows it. The model under-reads by ~10 %: correct it once measured.

## Stage 6 — ultrasonic and servo (subsystem 5, Ryan)

1. Normal image (USB console). Plug the HC-SR04P into Grove 1 (3.3 V).
2. **Servo (S5-M01):** `bench_servo` 0, 90, 180 with a protractor; adjust
   `OBST_SERVO_MIN_US` / `_MAX_US`; if 180 looks right instead of left, set
   `OBST_SERVO_LEFT_IS_HIGH 0` (`subsystem5/mission_cfg.h`).
3. **Range (S5-M02, A11):** `bench_servo` 90; a box at 10 / 30 / 100 cm:
   telemetry `obstacle.d_cm` within ±1 cm, steady. `NO_ECHO` (report-only
   fault bit 256) means no echo at all: wiring or supply.
4. **Stop distance (S5-M03, NFR6):** line-follow toward a box, 10 runs each at
   150 and 300 mm/s (`speed_mm_s`): `SLOW_DOWN` below 40 cm, `OBSTACLE` below
   20 cm, robot at rest ≥ 10 cm from the box. If not, raise `stop_cm`.
5. **Width (S5-M04):** boxes 10 / 15 / 20 cm (the instructor's range) at the
   left, centre and right of the line: compare telemetry `obstacle.w_mm` (the
   `PROFILE_DONE` event data is the same number) with a ruler. Grade targets:
   A ≤ 10 %, B ≤ 20 %, C ≤ 30 %. If it is consistently too wide or narrow,
   adjust `OBST_HALF_BEAM_CDEG` (the beam correction) — this is the main
   calibration of the width estimate; the 9 % worst case in the host test is
   on a model, not this sensor.
6. **Bypass (S5-M05, FR11):** 10 runs per offset, video from above: no
   contact, ≥ 50 mm clearance, back on the line within 0.5 m. Time from the
   `OBSTACLE` event to `RECOVERY_DONE` (event timestamps): A < 3 s, B < 5 s,
   C < 7 s. The scan alone is ~1.6 s (26 pings of 60 ms); faster bypasses
   come from `OBST_BYPASS_SPEED_MM_S`, `OBST_BYPASS_TURN_DEG_S` and
   `OBST_TURN_OUT_DDEG`, once the turns are accurate (stage 3).
7. **Reacquisition (S5-M06)** ≥ 9/10, **blocked path (S5-M08)**: walls on both
   sides → two reverse-and-rescans, BLOCKED, FAULT after 10 s.

## Stage 7 — full runs and the demo

Set `APP_BENCH 0`, `APP_TIMING_PINS 0`, `MISSION_SELFTEST_MASK 0x0F`, rebuild,
flash the normal image.

Demo procedure (instructor, forum 24 Oct):

1. Power on. `status` shows `online:true`, `map:null`; state `IDLE`.
2. The instructor names four characters. Send them:
   `{"id":<n>,"cmd":"map","left":"?","right":"?","straight":"?","uturn":"?"}`
   and check the ack and `status.map`.
3. Put the robot on the line, GP21 tap: calibration spin; wait for `IDLE`.
4. GP20 press or `{"cmd":"start"}`. The robot follows the line; each barcode
   (either direction) is decoded and latched (`nav` in telemetry), and the
   mapped movement is done at the **next cross-junction**; then it carries on.
5. Stop with GP20 or `{"cmd":"stop"}`: `MISSION_COMPLETE`, run summary
   (`LOG 67` = highest hump).
6. After any FAULT: remove the cause, GP21 held 2 s or `{"cmd":"reset"}` →
   `BOOT` → `IDLE`; calibrate again before the next run.

Integration tests T1–T7 (Tribunal 1 Version C) are run at this stage; log
them in `testing/integration/TEST_LOG.md`.

---

## Reference

### Values to measure

| File | Value | Stage / test |
|---|---|---|
| `subsystem2/motion_cfg.h` | `MOTION_COUNTS_PER_REV`, `MOTION_ENC_SIGN_*`, `MOTION_MOT_SIGN_*` | 3 / S2-M02 |
| | `MOTION_WHEEL_CIRC_UM` | 3 / S2-M03 |
| | `MOTION_TRACK_MM` | 3 / S2-M04 |
| | `MOTION_KFF_MILLI`, `_KP_`, `_KI_`, `MOTION_MIN_DUTY` | 3 / S2-M05 |
| `subsystem3/line_cfg.h` | `LINE_BLACK_IS_HIGH`, `LINE_STEER_SIGN`, `LINE_KP`, `LINE_KD`, `LINE_BC_OFFSET_MM` | 5 / S3-M01–M03 |
| MQTT `set` | `junction_mm` (default 12), `lost_mm` (80), `speed_mm_s` (250) | 5 / S3-M04 |
| `subsystem4/imu_cfg.h` | `IMU_AXIS_*`, `IMU_SIGN_*`, `IMU_WHEELBASE_MM`, `IMU_HUMP_ON_CDEG` | 4–5 / S4-M02–M04 |
| `subsystem5/mission_cfg.h` | `OBST_SERVO_MIN_US`, `_MAX_US`, `OBST_SERVO_LEFT_IS_HIGH` | 6 / S5-M01 |
| | `OBST_HALF_BEAM_CDEG`, `OBST_DEPTH_TOL_MM` | 6 / S5-M04 |
| | `OBST_ROBOT_WIDTH_MM`, `OBST_ROBOT_LENGTH_MM`, `MISSION_NAV_CENTRE_MM` | measure the robot |
| MQTT `set` | `stop_cm` (20), `slow_cm` (40) | 6 / S5-M03 |

### Telemetry `LOG` codes (`event` topic, `"type":"LOG"`)

| code | from | meaning (`data`) |
|---|---|---|
| 31 | S3 | barcode rejected (reason, see stage 5) |
| 32–34 | S3 | calibration: raw ADC max << 16 \| min for left, right, barcode sensor |
| 41 | S4 | heartbeat missed (bit mask: 1 Motion, 2 Line, 4 Mission, 8 Obstacle, 16 Imu) |
| 42 | S4 | the last reset was the watchdog |
| 51 | S4 | motion class (0 stationary, 1 accelerating, 2 turning, 3 climbing, 4 descending, 5 impact, 6 cruising, 7 braking) |
| 52 | S4 | a climb > 600 mm was a ramp, not a hump |
| 61 | S5 | start refused: not calibrated |
| 62 | S5 | calibration result: calibrated bits << 16 (`0x30000` = IR + IMU). Boot self-test failure: (`0x100` \| failed bits) << 16, failed bit 0 motion, 1 line, 2 IMU, 3 timer |
| 63 | S5 | barcode latched: movement << 16 \| character |
| 64 | S5 | barcode not in the map (character) |
| 65 / 66 | S5 | map stored / map refused |
| 67 | S5 | run summary: highest hump peak, mm |
| 68 | S5 | parameter set: id << 16 \| value |
| 69 / 70 | S5 | mbf_mission / mbf_motion was full (decision may be lost) |

### Known limits and open points (say these in the report)

- Everything is tested on models and in an emulator, not yet on the robot.
- 10 mm humps over ~300 mm are below the +5° detection threshold (S4-H01b).
- `junction_mm` is 12 mm, not FR2's 20 mm (18 mm crossing line).
- The width estimate's accuracy depends on `OBST_HALF_BEAM_CDEG`, which only
  S5-M04 can set.
- The emulator showed kernel time falling behind the 1 µs timer while the
  UART console printed heavily (lost ticks with interrupts masked). Normal
  builds use the USB console; check A2's jitter capture on the robot.
- The barcode map is held in RAM: send it again after any reset (the
  watchdog included).
