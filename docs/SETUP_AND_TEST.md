# PicoCar — setup and test guide

How to go from an empty laptop and a bare Pico W to a demo run, one
subsystem at a time. Follow the phases in order: each one assumes the
previous one passed. Record every result in that subsystem's
`testing/subsystemN/TEST_LOG.md` (the test IDs below, e.g. **S2-M02**, are
defined there).

**Status of the code (25 Oct 2026).** All five subsystems are implemented
to the **original requirements** (Week 6 design review, revised 22 Oct,
with its bug fixes) and the instructor's forum answers of 23–24 Oct.
Change Requirement 1 (minimum-bit data) is **not** implemented. The code
has been built (all variants, zero warnings), tested on a PC (logic, on
models) and run in the rp2040js emulator (single core, no sensors).
**Nothing has run on the robot yet.** Every tuning value marked `MEASURE`
in a `*_cfg.h` file is a placeholder until you measure it below.

| Phase | What | Who |
|---|---|---|
| A | Laptop: toolchain, code, broker, WiFi credentials | everyone (once per laptop) |
| B | First build and flash, bare board | anyone |
| C | Laptop link (MQTT) | S1 Victor |
| D | Motors and encoders | S2 Jedrek |
| E | IMU and safety | S4 Purvez |
| F | IR sensors, line, barcodes, humps | S3 Boon |
| G | Ultrasonic and servo | S5 Ryan |
| H | Demo configuration | everyone |

---

## What you need

| Item | Used for |
|---|---|
| The robot, assembled. Sensors can stay plugged in if already wired **and** voltage-checked (rule 2 below) | everything |
| USB cable, a laptop | build and flash |
| A serial terminal at 115200 (PuTTY, `screen`, VS Code Serial Monitor) | console and `[TEST]` lines |
| Mosquitto (broker + `mosquitto_sub` / `mosquitto_pub`) | MQTT, phase C onwards |
| A 2.4 GHz WiFi network the laptop and the Pico share (a phone hotspot is the safest) | MQTT |
| Multimeter; logic analyser or scope (any cheap 8-channel analyser works) | pin levels, PWM, timing |
| Tape measure, protractor, ruler, boxes 10 / 15 / 20 cm wide | calibration |
| The printed barcode sheets (A, F, Z, W, X, Y) and track with a junction, a gap and humps | phase F |

### Three rules that protect the hardware

1. **Wheels off the ground** (robot on a box) the first time you drive the motors.
2. **Measure before you connect** any sensor output to a Pico pin for the
   first time: it must be 3.3 V, never 5 V (encoders A4, HC-SR04P A11).
3. **UART-console builds** (`CONSOLE=uart` or `CRASH_REPORT=1`) use GP0/GP1
   for the console: unplug the HC-SR04P for those. The firmware then leaves
   GP0/GP1 alone and the ultrasonic sensor simply does nothing. The normal
   USB-console build does not have this problem.

You do not have to unplug sensors that are already wired and checked. A
sensor that is plugged in does not stop WiFi or telemetry: the robot simply
reports what it sees, and each sensor is tested in its own phase.

### Build switches you will flip during bring-up

| File | Switch | Default | Set it to |
|---|---|---|---|
| `app_program/subsystem5/mission_cfg.h` | `MISSION_SELFTEST_MASK` | `0x0Fu` | **`0x0Bu`** until the IMU is wired (phase E). Without the GY-511 the boot self-test fails and the robot stays in FAULT |
| `app_program/app_config.h` | `APP_BENCH` | 0 | **1** for phases D–G: drive the hardware from the laptop over MQTT |
| `app_program/app_config.h` | `APP_TIMING_PINS` | 0 | **1** when you capture timing on GP6 / GP7 (S2-M07, S3-M06) |

Put all three back to their defaults before a demo or a graded run (phase H).

---

## Phase A — Laptop (once per laptop)

> **Already followed the lab instructions / README sections 1–2 up to
> building and flashing the template `.uf2`?** Then A1–A3 are done: start
> at **A4**. The template build is not the PicoCar build: you still need
> this branch, `wifi_secrets.h` and the `NET=lwip MQTT=1` options below.

**A1. Build tools** (Linux / WSL):
```sh
sudo apt update && sudo apt install -y git make g++ gcc-arm-none-eabi
```
macOS: `xcode-select --install`, then the Arm GNU Toolchain (AArch32
bare-metal, `arm-none-eabi`) from developer.arm.com.

**A2. Pico SDK 2.2.0:**
```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init lib/tinyusb lib/cyw43-driver
```
Without the submodule line, USB and WiFi do not build.

**A3. Paths.** Add to `~/.bashrc` (or `~/.zshrc`), then open a new terminal:
```sh
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
export PATH="$HOME/tools/arm-gnu-toolchain-<ver>/bin:$PATH"   # only for a hand-installed toolchain
```
Check: `arm-none-eabi-gcc --version` and `ls $PICO_SDK_PATH` both print something.

**A4. The PicoCar code:**
```sh
cd ~/pico
git clone https://github.com/VictorTanJY/INF2004-EMBEDDED-SYS-PROJECT.git picocar
cd picocar && git checkout feature/subsystems-2-5
```
(Already cloned: `git fetch && git checkout feature/subsystems-2-5 && git pull`.)

**A5. Host tests (optional, ~1 min, no Pico needed)** from the repo root:
```sh
for d in shared subsystem1 subsystem2 subsystem3 subsystem4 subsystem5; do make -C testing/$d/host; done
```
Every suite must end with all tests passing. If one fails on your laptop,
stop and report it before touching the hardware.

**A6. Mosquitto.**
- Windows: the installer from mosquitto.org. Run it **on Windows, not in
  WSL**: WSL sits behind NAT and the Pico cannot reach a broker inside it.
- macOS: `brew install mosquitto`. Linux: `sudo apt install -y mosquitto mosquitto-clients`.

**A7. Let the broker accept the Pico.** Mosquitto 2.x listens on localhost
only by default — the most common reason the first connection fails.
Create `picocar.conf`:
```
listener 1883 0.0.0.0
allow_anonymous true
```
and start the broker with `mosquitto -c picocar.conf -v`.
- Windows: allow `mosquitto.exe` (TCP 1883) through the firewall on private networks.
- The Pico W only does **2.4 GHz** WiFi, and many campus networks block
  traffic between devices. If the Pico joins the WiFi but never reaches the
  broker, put the laptop and the Pico on a **phone hotspot** (2.4 GHz /
  "maximise compatibility").

**A8. The laptop's IP on that network:** `ipconfig` (Windows), `ip a`
(Linux), `ipconfig getifaddr en0` (macOS). It changes each time you join a
hotspot: check it every session.

**A9. Credentials.** `config/wifi_secrets.h` is git-ignored:
```sh
cp config/wifi_secrets.example.h config/wifi_secrets.h
```
Set `WIFI_SSID`, `WIFI_PASSWORD` and `MQTT_BROKER_IP` (A8). Keep port 1883
and client ID `picocar-is23`; if two Picos ever share a broker, give each a
different client ID, or they keep disconnecting each other. Rebuild after
every change to this file.

---

## Phase B — First build and flash (bare board)

**B1.** Pico W in the Robo Pico. Robot **on a box, wheels off the ground**.
Nothing needs to be in the Grove ports yet (see "What you need").

**B2.** Set `MISSION_SELFTEST_MASK` to **`0x0Bu`** (no IMU yet). Leave
`APP_BENCH 0` and `APP_TIMING_PINS 0` for now.

**B3. Build the test image:**
```sh
cd build_make
make -j8 NET=lwip MQTT=1 TEST=1     # -> mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test.uf2
```
The build must end without warnings. The file name always tells you which
options an image has; the normal image (phase C onwards) is
`make -j8 NET=lwip MQTT=1` → `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`.

**B4. Flash:** hold **BOOTSEL**, plug in USB, release. Copy the `.uf2` onto
the **RPI-RP2** drive (WSL: `explorer.exe .` in `build_make`). The Pico
reboots by itself.

**B5. Console at 115200:** `screen /dev/ttyACM0 115200` (Linux),
`screen /dev/tty.usbmodem* 115200` (macOS), PuTTY / VS Code Serial Monitor
on the new COM port (Windows). If you connected too late, press reset or
re-plug to see the boot.

**B6. Expected:** `all subsystems started`, the `[TEST]` lines, then
`=== test run end ===`. Every line `PASS`, and
`[INFO] timer: 100000 us per 100 ms of CPU clock` (±2 %).
- **S4-U05** failing means the 1 µs timer is not at 1 MHz — stop and
  investigate: everything timed depends on it.
- `S4-U04` (ImuTask snapshot) passes even without an IMU; the snapshot just says `ok = 0`.

**B7. Pins (S2-M01):** GP8, GP9, GP10, GP11 LOW (motor driver brake);
GP15 a 50 Hz pulse of ~1.5 ms (servo centre).

**B8. Battery:** unplug USB, power from the battery / power bank with the
Robo Pico power switch on: it boots the same. USB alone cannot drive the
motors (they run from VIN).

Record: S1–S5 `U` tests in each TEST_LOG with "hardware" in the notes.

---

## Phase C — Laptop link (subsystem 1, Victor)

Goal: MQTT works, because every later phase uses it to see what the robot does.

**C1.** Flash the **normal** image (`make -j8 NET=lwip MQTT=1`).

**C2.** Two terminals on the laptop:
```sh
mosquitto -c picocar.conf -v
mosquitto_sub -h localhost -v -t 'picocar/is23/#'
```
Keep the `mosquitto_sub` window open from now on: it is your view of
everything the robot does.

**C3. Expected within ~10 s of boot:**
- broker log: `New client connected ... as picocar-is23`;
- `picocar/is23/status` (retained, 1 Hz): `{"online":true,...,"map":null}` (**S1-M02**);
- `picocar/is23/event`: `STATE` `IDLE`; `picocar/is23/telemetry` at 5 Hz.

**C4. Nothing arrives?** Read the serial console. WiFi join failed → SSID,
password, or a 5 GHz-only network. WiFi up but MQTT never connects → broker
IP, firewall, broker on localhost only (A7), or client isolation (hotspot).

**C5. Barcode map**, the way the instructor will do it (**S1-M07**, **S5-M07**):
```sh
mosquitto_pub -h localhost -q 1 -t picocar/is23/cmd \
  -m '{"id":1,"cmd":"map","left":"A","right":"F","straight":"Z","uturn":"W"}'
```
Expect `{"id":1,"ok":true}` on `picocar/is23/ack` and `status.map` showing
the four letters. A missing movement, a letter used twice or `*` is
answered with `"err":"bad_map"`. The map is only accepted in IDLE and is
**lost at reboot** — send it again after every power cycle.

**C6. Buttons (S1-M01):** GP20 press → `BTN_START_STOP` event; GP21 tap →
`BTN_CALIBRATE`; GP21 held 2 s → `BTN_RESET`.

**C7.** S1-M03 (pull the power: the broker publishes the Last Will,
`online:false`), S1-M04 (stop / start the broker: the Pico reconnects by
itself), S1-M05 (each command and error below), S1-M06 (10-minute soak).

Commands (all need `"id"`; use a new id each time):

| Purpose | Payload on `picocar/is23/cmd` |
|---|---|
| start a run | `{"id":2,"cmd":"start"}` (refused with `not_calibrated` until the phase F calibration) |
| stop | `{"id":3,"cmd":"stop"}` |
| emergency stop | `{"id":4,"cmd":"estop"}` |
| reset after a fault | `{"id":5,"cmd":"reset"}` |
| run parameter (IDLE) | `{"id":6,"cmd":"set","param":"speed_mm_s","value":200}` — also `slow_cm`, `stop_cm`, `lost_mm`, `junction_mm` |
| bench (APP_BENCH=1, IDLE) | `{"id":7,"cmd":"set","param":"bench_move","value":300}` — also `bench_turn` (0.1 deg), `bench_speed` (mm/s), `bench_servo` (deg) |

---

## Phase D — Motors and encoders (subsystem 2, Jedrek)

Robot **on a box, wheels free**.

**D1.** Set `APP_BENCH 1`, rebuild and flash the normal image. Bench
commands only work in IDLE.

**D2. Encoder levels first (A4).** Plug only the encoder power (3V3, GND)
into Grove 3 / Grove 4, A/B wires **off** the Pico. Turn each wheel by hand
and measure A and B: they must swing between 0 and 3.3 V, never 5 V. Then
plug the A/B wires in (GP4/5 left, GP16/17 right).

**D3. Counts per revolution (S2-M02):** mark each wheel, turn it exactly 10
revolutions forward by hand, read `enc` in telemetry. Set
`MOTION_COUNTS_PER_REV` (counts / 10) in `subsystem2/motion_cfg.h`. If the
count goes **down** when the wheel turns forward, flip `MOTION_ENC_SIGN_L/_R`.

**D4. Direction:**
```sh
mosquitto_pub -h localhost -t picocar/is23/cmd -m '{"id":10,"cmd":"set","param":"bench_move","value":200}'
```
Both wheels must turn forward. If one runs backwards, flip its
`MOTION_MOT_SIGN_*`. If a wheel runs away (speed keeps rising, then a STALL
fault), its **encoder** sign is wrong.

**D5. PWM (S2-M01):** analyser on GP8–GP11 during a move: 20 kHz, only one
pin of each pair active.

**D6. Distance (S2-M03), on the floor:** `bench_move` 1000, five times,
tape measure. Scale `MOTION_WHEEL_CIRC_UM` by measured / commanded. Lateral
drift over 1 m ≤ 30 mm (NFR5).

**D7. Turns (S2-M04):** `bench_turn` 900 and 1800, ten each. Scale
`MOTION_TRACK_MM` by commanded / measured angle (a robot that turns too far
has a track width set too small).

**D8. Speed loop (S2-M05):** `bench_speed` 250, analyser on GP4 (left
encoder A): the pulse rate is the wheel speed against time (telemetry at
5 Hz is too slow for this). Start with feed-forward only (KP = KI = 0): the
speed should already be close. Then tune `MOTION_KFF_MILLI`,
`MOTION_KP_MILLI`, `MOTION_KI_MILLI`, `MOTION_MIN_DUTY` until it settles
within 300 ms with < 10 % error.

**D9. Stall (S2-M06):** during a `bench_move` 2000, hold one wheel: a
`FAULT` event with data 4 (STALL) after 300–400 ms, motors braked. Reset
with GP21 held 2 s or `{"cmd":"reset"}`.

**D10. Timing (S2-M07):** `APP_TIMING_PINS 1`, analyser on GP6 for 60 s
while telemetry runs: every edge 10 ms ± 1 ms (NFR1). If not, see A2 (pace
the loop from a TIMER alarm on core 1).

---

## Phase E — IMU and safety (subsystem 4, Purvez)

The IMU comes before the line sensors because a run (`start`) needs
**both** the IR and the IMU calibration.

**E1.** Wire the GY-511 to Grove 2 (GP2 SDA, GP3 SCL) and INT1 to GP19;
mount it ≥ 5 cm from the motors.

**E2.** Set `MISSION_SELFTEST_MASK` back to **`0x0Fu`**, rebuild, flash.

**E3. Found (S4-M01):** boot reaches `IDLE`, not FAULT. If it goes to FAULT
(cause self-test), `LOG 62` with data `0x01040000` names the IMU: check the
wiring and pull-ups.

**E4. Axes (S4-M02):** nose up by hand → `pitch_deg` in telemetry goes
positive. Lift the left side → roll goes positive (IMU snapshot). Otherwise
fix `IMU_AXIS_*` / `IMU_SIGN_*` in `subsystem4/imu_cfg.h`.

**E5. Pitch noise (S4-M03):** with `bench_move` over the flat track, pitch
stays within ±1°.

**E6. Safety (S4-M05 to S4-M08):** tilt > 30° → FAULT, and a reset is
refused while still tilted. A firm tap on the chassis (INT1) brakes the
motors. An MQTT `estop` brakes them. With the analyser, time detection →
GP8–GP11 LOW: **≤ 20 ms** (NFR8). A `FAULT` event's data is the fault bit
mask (1 impact, 2 too close, 4 stall, 8 tilt, 16 E-stop, 32 heartbeat).

**E7. Power (S4-M09, A10):** scope V+ and 3V3 while starting the motors
with the servo sweeping (`bench_servo`): no resets.

Hump height (S4-M04) needs line following: it is step F8.

---

## Phase F — IR sensors, line and barcodes (subsystem 3, Boon)

**F1. Wiring:** left line sensor → Grove 5 (GP26); right line sensor →
Grove 6 (GP27; insulate Grove 6's GP26 wire); barcode sensor → Grove 7
(GP28). VBAT jumper **open**.

**F2. Mounting:** line sensors 3–8 mm above the track, centres ~18 mm
apart, straddling the line edges; barcode sensor ~3 mm high at the **far
left** of the robot (barcodes are always on the left of the line in the
direction of travel). Measure its distance from the robot centre line and
set `LINE_BC_OFFSET_MM` to minus that (default `-45`; + is right).

**F3. Calibration and raw levels (S3-M01, S3-M02, A7, FR16):** robot
centred on a straight line, GP21 tap. It stands still 1 s (IMU level
offsets), spins 360° at 90°/s (IR black / white, magnetometer), then logs:
- `LOG 32 / 33 / 34` (left, right, barcode sensor): `data` = highest raw
  ADC value << 16 | lowest. Black / white must differ by ≥ 3:1 on the two
  line sensors; if not, change the sensor height or shade them.
- `LOG 62`: `data` = calibrated bits << 16 — `0x30000` = IR and IMU both
  good (bit 0 IR, bit 1 IMU).

The barcode sensor only crosses the line briefly in the spin; its black
level is refined on every barcode read.

**F4. Line following (S3-M03):** `start`. If the robot steers away from the
line, flip `LINE_STEER_SIGN`; if it seems to see the line where there is
none, the module reads lower over black: set `LINE_BLACK_IS_HIGH 0`. Tune
`LINE_KP`, `LINE_KD` (`line_cfg.h`) and `speed_mm_s`. 5 laps without help.

**F5. Junctions and gaps (S3-M04):** one `JUNCTION` event per crossing,
none on curves or 40 mm gaps. `junction_mm` defaults to **12 mm**, not
FR2's 20 mm: a crossing line is only 18 mm wide, so both sensors are fully
black for less than 20 mm. Tune it with
`{"cmd":"set","param":"junction_mm",...}`.

**F6. Barcodes (S3-M05, NFR3):** send the map, start, drive over each
sheet. `event` shows `BARCODE` with the character (data `"A"` etc.). Every
character A–Z decodes, mapped or not; only mapped ones are latched
(`LOG 63`, telemetry `nav`), unmapped ones give `LOG 64`. Do 20 passes per
character at 150 and 300 mm/s, both directions; the sheets come in three
print sizes and all three must read. Rejected reads give `LOG 31` (data =
reason: 2 count, 3 ratio, 4 pattern, 5 start/stop, 6 length, 7 too long).

**F7. Barcode → junction (S5-M07):** with the map sent, every latched
character is executed at the next cross-junction (left / right / straight /
U-turn), then line following resumes.

**F8. Timing (S3-M06):** `APP_TIMING_PINS 1`, GP7 toggles every 10 ms
block; snapshot `missed_blocks` stays 0 (NFR2).

**F9. Humps (S4-M04, NFR7):** 10 passes over humps of known height. Each
gives `HUMP_START`, then `HUMP_END` with the peak in mm; telemetry
`hump_max_mm`; and the run summary (`LOG 67`, data = highest peak) when
you stop. Known limit: the climb threshold is +5°, which a 10 mm × 300 mm
hump does not reach with this chassis (host test S4-H01b). If the course
has humps that low, lower `IMU_HUMP_ON_CDEG` only after S4-M03 shows the
noise allows it. The model under-reads by ~10 %: correct it once measured.

---

## Phase G — Ultrasonic and servo (subsystem 5, Ryan)

**G1.** Normal image (USB console, not a UART build). Check the HC-SR04P's
echo output is 3.3 V (A11), then plug it into Grove 1 (GP0 trigger, GP1 echo).

**G2. Servo (S5-M01):** `bench_servo` 0, 90, 180 with a protractor; adjust
`OBST_SERVO_MIN_US` / `_MAX_US`; if 180 looks right instead of left, set
`OBST_SERVO_LEFT_IS_HIGH 0` (`subsystem5/mission_cfg.h`).

**G3. Range (S5-M02, A11):** `bench_servo` 90; a box at 10 / 30 / 100 cm:
telemetry `obstacle.d_cm` within ±1 cm, steady. `NO_ECHO` (report-only
fault bit 256) means no echo at all: wiring or supply.

**G4. Stop distance (S5-M03, NFR6):** line-follow toward a box, 10 runs
each at 150 and 300 mm/s (`speed_mm_s`): `SLOW_DOWN` below 40 cm,
`OBSTACLE` below 20 cm, robot at rest **≥ 10 cm** from the box. If not,
raise `stop_cm`.

**G5. Width (S5-M04):** boxes 10 / 15 / 20 cm (the instructor's range) at
the left, centre and right of the line: compare telemetry `obstacle.w_mm`
(the `PROFILE_DONE` event data is the same number) with a ruler. Grade
targets: A ≤ 10 %, B ≤ 20 %, C ≤ 30 %. If it is consistently too wide or
narrow, adjust `OBST_HALF_BEAM_CDEG` (the beam correction) — this is the
main calibration of the width estimate; the 9 % worst case in the host test
is on a model, not this sensor.

**G6. Bypass (S5-M05, FR11):** 10 runs per offset, video from above: no
contact, ≥ 50 mm clearance, back on the line within 0.5 m. Time from the
`OBSTACLE` event to `RECOVERY_DONE` (event timestamps): A < 3 s, B < 5 s,
C < 7 s. The scan alone is ~1.6 s (26 pings of 60 ms); faster bypasses come
from `OBST_BYPASS_SPEED_MM_S`, `OBST_BYPASS_TURN_DEG_S` and
`OBST_TURN_OUT_DDEG`, once the turns are accurate (phase D).

**G7. Reacquisition (S5-M06)** ≥ 9/10. **Blocked path (S5-M08):** walls on
both sides → two reverse-and-rescans, BLOCKED, FAULT after 10 s.

---

## Phase H — Full runs and the demo

**H1.** Set `APP_BENCH 0`, `APP_TIMING_PINS 0`, `MISSION_SELFTEST_MASK
0x0Fu`, rebuild, flash the **normal** image. Check the laptop IP still
matches `MQTT_BROKER_IP` (A8) and the broker is running (A7).

**H2. Demo procedure** (instructor, forum 24 Oct):
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

**H3. After any FAULT:** remove the cause, GP21 held 2 s or
`{"cmd":"reset"}` → `BOOT` → `IDLE`; calibrate again before the next run,
and **send the map again** if the board rebooted.

Integration tests T1–T7 are run at this phase; log them in
`testing/integration/TEST_LOG.md`.

---

## Reference

### Values to measure

| File | Value | Phase / test |
|---|---|---|
| `subsystem2/motion_cfg.h` | `MOTION_COUNTS_PER_REV`, `MOTION_ENC_SIGN_*`, `MOTION_MOT_SIGN_*` | D / S2-M02 |
| | `MOTION_WHEEL_CIRC_UM` | D / S2-M03 |
| | `MOTION_TRACK_MM` | D / S2-M04 |
| | `MOTION_KFF_MILLI`, `_KP_`, `_KI_`, `MOTION_MIN_DUTY` | D / S2-M05 |
| `subsystem3/line_cfg.h` | `LINE_BLACK_IS_HIGH`, `LINE_STEER_SIGN`, `LINE_KP`, `LINE_KD`, `LINE_BC_OFFSET_MM` | F / S3-M01–M03 |
| MQTT `set` | `junction_mm` (default 12), `lost_mm` (80), `speed_mm_s` (250) | F / S3-M04 |
| `subsystem4/imu_cfg.h` | `IMU_AXIS_*`, `IMU_SIGN_*`, `IMU_WHEELBASE_MM`, `IMU_HUMP_ON_CDEG` | E–F / S4-M02–M04 |
| `subsystem5/mission_cfg.h` | `OBST_SERVO_MIN_US`, `_MAX_US`, `OBST_SERVO_LEFT_IS_HIGH` | G / S5-M01 |
| | `OBST_HALF_BEAM_CDEG`, `OBST_DEPTH_TOL_MM` | G / S5-M04 |
| | `OBST_ROBOT_WIDTH_MM`, `OBST_ROBOT_LENGTH_MM`, `MISSION_NAV_CENTRE_MM` | measure the robot |
| MQTT `set` | `stop_cm` (20), `slow_cm` (40) | G / S5-M03 |

### Telemetry `LOG` codes (`event` topic, `"type":"LOG"`)

| code | from | meaning (`data`) |
|---|---|---|
| 31 | S3 | barcode rejected (reason, see step F6) |
| 32–34 | S3 | calibration: raw ADC max << 16 \| min for left, right, barcode sensor |
| 41 | S4 | heartbeat missed (bit mask: 1 Motion, 2 Line, 4 Mission, 8 Obstacle, 16 Imu) |
| 42 | S4 | the last reset was the watchdog |
| 51 | S4 | motion class (0 stationary, 1 accelerating, 2 turning, 3 climbing, 4 descending, 5 impact, 6 cruising, 7 braking) |
| 52 | S4 | a climb > 600 mm was a ramp, not a hump |
| 53 | S4 | IMU start result: stage << 24 \| error << 16 \| IRA_REG_M << 8 \| WHO_AM_I_M. Stage 0 ok (low byte 0x48xx = LSM303DLHC, xx40 = LSM303AGR), 1 magnetometer 0x1E not answering, 2 unknown chip, 3 accelerometer 0x19 not answering, 4 configuration write failed; `0xFF000000` = I2C1 did not configure |
| 54 | S4 | after a failed IMU start: the first 4 I2C1 addresses that answer, one per byte (GY-511: `0x1E19` = 0x19 and 0x1E). 0 = nothing answers (wiring/power), `-1` = bus stuck (SDA or SCL held low) |
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
