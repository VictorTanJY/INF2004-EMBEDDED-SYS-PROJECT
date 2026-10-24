# Team and subsystems

PicoCar (team IS-23). The example rows have been replaced with the Week 6
architecture freeze; update a row whenever the design changes.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi/MQTT command & telemetry (CommsTask = template WiFi service, MQTT client extension) and buttons (HmiTask) – Victor | 2503553 | VictorTanJY | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: PWM motor driver, PIO quadrature encoders, PID, move/turn API (MotionTask) – Jedrek | 2500917 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following and Code 39 barcode decoding (LineBarcodeTask) – Ho Boon Hean | 2500519 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU terrain monitoring (ImuTask) and fault supervision / watchdog (SafetyTask) – Purvez | 2503229 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Obstacle scanning and avoidance (ObstacleTask) and system state machine (MissionTask) – Ryan | 2500443 | | `app_program/subsystem5/` | `testing/subsystem5/` |

Source of every row below: Week 6 design review (architecture freeze), with the
changes justified in Tribunal 1 (marked **T1-gN**).

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S5 | US_TRIG, HC-SR04P trigger, Grove 1 (unplug for `CRASH_REPORT=1` / `CONSOLE=uart` builds) |
| GP1 | S5 | US_ECHO, edge IRQ (echo_isr, IO_IRQ_BANK0 on core 1), Grove 1 (same caveat) |
| GP2 | S4 | IMU_SDA, I2C1, Grove 2 (Maker port left empty) |
| GP3 | S4 | IMU_SCL, I2C1, Grove 2 |
| GP4 | S2 | ENC_L_A, PIO0 SM0, Grove 3 |
| GP5 | S2 | ENC_L_B, PIO0 SM0 (= A + 1), Grove 3 |
| GP6 | spare (S2 timing) | Grove 5 second wire, not connected. With `APP_TIMING_PINS=1` (app.h) MotionTask toggles it every 10 ms cycle, for NFR1 jitter captures |
| GP7 | spare (S3 timing) | Grove 7 second wire, not connected. With `APP_TIMING_PINS=1` LineBarcodeTask toggles it every ADC block (NFR2) |
| GP8 | S2 | MOT_L_A, PWM4 A, 20 kHz (SafetyTask may brake it through `motor_estop()`) |
| GP9 | S2 | MOT_L_B, PWM4 B (same) |
| GP10 | S2 | MOT_R_A, PWM5 A (same) |
| GP11 | S2 | MOT_R_B, PWM5 B (same) |
| GP12 | spare | servo header S1, empty |
| GP13 | spare | servo header S2, empty |
| GP14 | spare | servo header S3, empty; shares PWM slice 7 with GP15, keep unused |
| GP15 | S5 | SERVO_PWM, PWM7 B, 50 Hz, servo header S4 |
| GP16 | S2 | ENC_R_A, PIO0 SM1, Grove 4 |
| GP17 | S2 | ENC_R_B, PIO0 SM1 (= A + 1), Grove 4 |
| GP18 | spare | **wired on the Robo Pico to the NeoPixel LED**: leave as input |
| GP19 | S4 | IMU_INT1, edge IRQ (imu_int1_isr, same bank-0 handler as GP1, core 1) |
| GP20 | S1 | BTN_START, on-board button, pull-up, pressed = LOW (check S1-M01) |
| GP21 | S1 | BTN_MODE, on-board button; long press 2 s = fault reset |
| GP22 | spare | **wired on the Robo Pico to the piezo buzzer**: leave as input |
| GP26 | S3 | IR_LINE_L, ADC0, Grove 5 (GP26 wire only) |
| GP27 | S3 | IR_LINE_R, ADC1, Grove 6 (GP27 wire only; its GP26 wire insulated) |
| GP28 | S3 | IR_BARCODE, ADC2, Grove 7; VBAT-sense jumper must stay open |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | – | – | – |
| I2C0 | – | – | – (kernel I2C driver stays off, `DEVCNF_USE_IIC = 0`) |
| I2C1 | S4 | ImuTask | GP2, GP3; `drv_i2c1.c`, polled, 400 kHz, reset pulse at init (also used to recover the bus) |
| SPI0 / SPI1 | – | – | – |
| ADC + DMA channels 10, 11 + DMA_IRQ_1 (core 1) | S3 | LineBarcodeTask | GP26–GP28; `drv_adc.c`: round robin 3 kHz, two chained 30-sample buffers (ping-pong, 10 ms each). Kernel ADC driver off (`DEVCNF_USE_ADC = 0`). Channels 10/11 are fixed: the radio's SDK driver claims its two from the bottom (0, 1) |
| PWM slices 4, 5 | S2 | MotionTask | GP8–GP11; `drv_motor.c`, 20 kHz (TOP 6249). The PWM block is held in reset at boot: `app_reset_release()` releases it and never re-asserts it (S5 shares the block) |
| PWM slice 7 | S5 | ObstacleTask | GP15; `drv_servo.c`, 50 Hz, 1 us resolution |
| PIO0 SM0, SM1 | S2 | MotionTask | GP4/5, GP16/17; `drv_encoder.c`: pico-examples quadrature program (24 instr.) at offset 0; refuses to start if SM0/SM1 already run. The radio takes PIO1 first (SDK 2.2.0) |
| IO_IRQ_BANK0 (core 1) | shared base | `app_gpio_irq.c`: one handler, per-pin callbacks (echo_isr S5, imu_int1_isr S4), registered in usermain, enabled by each owner on core 1 | GP1, GP19 |
| RP2040 watchdog + timer tick | S4 | SafetyTask | – `drv_watchdog.c`: tick = 12 cycles first thing in `subsystem4_init()` (else the 1 us timer runs 42.7 × slow after a cold power-on); 200 ms timeout, load 2 × 200 000 (erratum RP2040-E1); boot self-test S4-U04 compares the timer with clk_sys |
| TIMER (read only) | – | any | – the 1 us counter is read by every subsystem (`app_time_us()`); nobody writes it |
| WiFi radio, TCP/IP, MQTT | *kernel* + S1 | `cyw43_utk_task` (built in) = CommsTask | GP23–25, GP29; PIO1 + 2 DMA channels |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 11 (+ kernel WiFi 4) | HmiTask – 11 – core 0; CommsTask = `cyw43_utk_task` – 4 – core 0 (kernel) |
| 2 | 6 | MotionTask – 6 – core 1 |
| 3 | 7 | LineBarcodeTask – 7 – core 1 |
| 4 | 5, 10 | SafetyTask – 5 – core 1; ImuTask – 10 – core 0 |
| 5 | 8, 9 | MissionTask – 8 – core 1; ObstacleTask – 9 – core 1 |

**Team agreement:** application tasks use priorities 5–11 instead of the default
bands 12–21, ordered by criticality and deadline (Week 6 Section 3). The kernel
keeps 1–4 (USB console 3, WiFi 4, both on core 0). Every control loop runs on
core 1. Each task's priority and core is a macro in its subsystem header
(`S2_PRI_MOTION`, ...).

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S3, S5 → S2 | message buffer (owned by S2), `TMO_POL` | `g_mbf_motion` (mbf_motion) | `motion_cmd_t` (8 B) in `motion_types.h`: DRIVE v mm/s + ω mrad/s, MOVE mm, TURN 0.1 deg, STOP, SET_AUTH, CLEAR_ESTOP; authority rules there | implemented; owners S2, S3, S5 to review |
| IF-02 | S2 → all | snapshot + per-snapshot spinlock (T1-g2) | `motion_get_odom()` | `motion_odom_t`: counts, distance, pose (x, y, heading), speeds, state, `last_done_seq` / `last_fail_seq` (ObstacleTask reads its step completions here) | implemented; S2 to review |
| IF-03 | S4 → S2 | function call, core 1 only | `motor_estop()`, `motion_estop_latched()` | brake GP8–11 via SIO LOW + latch; CLEAR_ESTOP inside DI/EI only while the latched-fault word is 0 (T1-g3) | implemented; S2, S4 to review |
| IF-04 | S3 → all | snapshot + spinlock | `line_get_snap()` | `line_snap_t` (`line_types.h`): error, normalised levels, flags (ON_LINE, LOST, JUNCTION, ...), barcode counters, block counter for NFR2 | implemented; S3 to review |
| IF-05 | S1, S2, S4, S5 → S4 | event flag (owned by S4) | `g_flg_safety` (flg_safety) | `SAFETY_FLT_*` bits (`imu_types.h`); SafetyTask waits `TWF_ORW \| TWF_BITCLR` on brake bits + `SAFETY_REQ_RESET` (T1-g1); setter clears report-only bits | implemented; S4 to review |
| IF-06 | all → S5 | message buffer (owned by S5), `TMO_POL` | `g_mbf_mission` (mbf_mission) | `mission_evt_t` (8 B), 29 `mission_ev_t` codes (`mission_types.h`; new: REMOTE_MAP, SAFETY_OK). A dropped post is counted; while running it is a fault | implemented; all to review |
| IF-07 | monitored tasks → S4 | one 32-bit word per task | `safety_heartbeat()` | timeouts 50/50/300/300/100 ms (`safety_core.c`) | implemented |
| IF-08 | S4 → all | snapshot + spinlock | `imu_get_snap()` | `imu_snap_t`: pitch/roll, hump state and maxima, motion class, `ok`, `pitch_valid` (obstacle hump mask) | implemented; S4 to review |
| IF-09 | S5 (Mission → Obstacle) | message buffer + event flag (owned by S5) | `g_mbf_scan`, `g_flg_obst` | `scan_req_t` via `scan_request()` (sets `OBST_FLG_REQ`) | implemented |
| IF-10 | S5 → all | snapshots + spinlocks | `mission_get_snap()`, `obst_get_snap()` | `mission_snap_t` (state, calibration phase, run parameters, speed cap, barcode map, latched command), `obst_snap_t` (range, width, edges, clearances, plan) | implemented; S5 to review |
| IF-11 | S1 ↔ laptop | MQTT (template client extended, A14) | topics `picocar/is23/{telemetry,event,status,cmd,ack}` | JSON, see `comms_proto.h`. Commands: start, stop, estop, reset, set, **map** (barcode character → left / right / straight / U-turn, sent before the demo: instructor, 24 Oct). Status shows the map | **instructor agreement needed** (A14); ranges: S2, S3, S5 |
| IF-12 | all → S1 | message buffer (owned by S1), `TMO_POL` | `g_mbf_telem` (mbf_telem, 1 KB) | `comms_rec_t` (12 B) via `comms_telem_post()` | pending: all |

"pending" = written into the headers by the shared base; the named owners
must confirm the format before relying on it.

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are PicoCar's planned usage.

| Object | Limit | Kernel / template | S1 Comms+HMI | S2 Motion | S3 Line | S4 IMU+Safety | S5 Obstacle+Mission | **Total** |
|---|---|---|---|---|---|---|---|---|
| Task | 32 | 3: initial task, USB console, WiFi (`cyw43_utk_task` = CommsTask) | 1: HmiTask | 1: MotionTask | 1: LineBarcodeTask | 2: SafetyTask, ImuTask | 2: MissionTask, ObstacleTask | **10** |
| Message buffer | 12 | – | 1: mbf_telem | 1: mbf_motion | – | – | 2: mbf_mission, mbf_scan | **4** |
| Event flag | 16 | 3: serial driver ×2, device-manager lock | – | – | – | 1: flg_safety | 1: flg_obst | **5** |
| Semaphore | 16 | 1: serial driver lock | – | – | – | – | – | **1** |
| Cyclic handler | 12 | – | – | 1: motion_cyc | – | 1: imu_cyc | 1: range_cyc | **3** |
| Mutex | 12 | – | – | – | – | – | – | **0** (Week 6 had 1: mtx_snap, replaced by spinlocks, T1-g2) |
| Mailbox, memory pools, alarm handler | 12 each | – | – | – | – | – | – | **0** |

Every total is well under its limit, so `config/config.h` is unchanged.

- **All created:** 7 application tasks, 4 message buffers, 2 application
  event flags and 3 cyclic handlers (feature/subsystems-2-5).
- **Kernel / template column:** counted from the source for this build
  configuration (`DEVCNF_USE_SER = 1`, `DEVCNF_USE_ADC = 0`,
  `DEVCNF_USE_IIC = 0`): `sysinit.c`, `usb_console.c`, `cyw43_utk_kernel.c`,
  `ser_rp2040.c`, `msdrvif.c`, `device.c`. Confirm on the board with
  `tk_ref_*` if a limit is ever approached.
- **Not kernel objects:** the per-snapshot spinlocks (`app_lock_t`) and the
  heartbeat words cost no IDs.
