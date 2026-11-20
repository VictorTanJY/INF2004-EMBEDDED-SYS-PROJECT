# Change Requirement 1: minimum-bit data representation

How the firmware meets Change Requirement 1, where each part is in the
code, how it is tested, and what it changed (measured, before and after).

**Before:** commit `e22e527` (JSON MQTT, full-width fields).
**After:** this branch from `619b823` (MQTT) onwards.
Raw measurements: `testing/system/logs/2026-11-15_cr1_before_after.txt`.

## 1. What changed, by the eight areas of the requirement

| # | Area | What is stored in minimum bits | Where |
|---|---|---|---|
| 1 | Motion control and encoder data | `mbf_motion` command: one 32-bit word. Odometry snapshot: 329 bits (11 words) | `motion_core.c` (`motion_core_cmd_pack`, `g_odom_layout`) |
| 2 | Line and barcode sensing | Line snapshot: 209 bits (7 words); barcode character as a 6-bit Code 39 symbol | `line_track.c` (`g_line_layout`) |
| 3 | Obstacle detection | Obstacle snapshot: 166 bits (6 words); `mbf_scan` request: 16 bits | `obstacle_core.c`, `mission_core.c` |
| 4 | Hump estimation | IMU snapshot (pitch, humps, motion class): 164 bits (6 words) | `imu_core.c` (`g_imu_layout`) |
| 5 | Vehicle and controller states | Every state stored outside its task in 2-4 bits (mission 4, obstacle 4, motion 3, phase 3, barcode 2, hump 2, motion class 3) | the layout tables above |
| 6 | Data shared between tasks / ISR and task | Snapshots stored packed; heartbeat counters one byte each; faults one bit each in `flg_safety`; echo ISR result one 17-bit word | `subsystem2.c` ... `subsystem5.c`, `drv_ultrasonic.c` |
| 7 | Inter-task queue messages | `mbf_motion` 8 -> 4 B, `mbf_mission` 8 -> 4 B, `mbf_scan` 4 -> 2 B, `mbf_telem` 12 -> 8 B | `motion_core.c`, `mission_core.c`, `comms_proto.c` |
| 8 | MQTT telemetry and command payloads | Binary bit-field frames instead of JSON text | `comms_proto.c`, decoded by `dashboard/picocar_codec.py` |

Faults were already one bit each in the `flg_safety` kernel event flag:
`LIFT 0x0001`, `TOO_CLOSE 0x0002`, `STALL 0x0004`, `TILT 0x0008`,
`HEARTBEAT 0x0020`, `ENC_FAULT 0x0040`, `IMU_FAULT 0x0080`, `NO_ECHO 0x0100`,
`WIFI_LOSS 0x0200`, `SELFTEST 0x0400` (`subsystem4/imu_types.h`).

## 2. How fields are packed (`app_program/app_pack.h`, `app_pack.c`)

- A field holds only the bits its valid range needs.
- **Unsigned** values saturate at the field's largest value, so an out-of-range
  value is never stored as a wrong, wrapped value.
- **Signed** values are stored offset-binary in an unsigned field
  (stored = value + 2^(width-1)), saturating at both ends. No signed bit-field
  and no bit operation on signed data is used (Barr-C 2018).
- **Counters and times** that readers only difference (encoder counts,
  `pings`, `blocks`, sequence numbers) keep their low bits and wrap.
- **"None" values** (`0xFFFF` = no range) map to the field's all-ones value.
- **Code 39 characters** are stored as a 6-bit symbol index (1..43, 0 = none).
- Words in RAM use explicit masks and shifts (`app_pack_set` / `app_pack_get`); MQTT
  frames are written least significant bit first (`app_pack_put` / `app_pack_take`), so
  the layout does not depend on the compiler or byte order.
- Each snapshot has a **layout table** (`app_pack_field_t`): one line per field with
  its width and kind. The tables are the data representation tables below,
  in the code itself, and the host tests check that every table's widths
  add up to the declared number of bits.

## 3. Data representation tables

### 3.1 MQTT frames (byte 0: type 5 bits + version 3 bits)

| Frame | Fields (bits) | Bytes | JSON before |
|---|---|---|---|
| telemetry (5 Hz) | seq 8, time 24 (10 ms), state 4, v_l 10, v_r 10, enc_l 16, enc_r 16, dist 16, line_err 11, lost 1, line_bc 10, barcode 6, nav 6, pitch 11, roll 12 (0.1 deg), hump_max 8, d 9 (cm), w 10 (mm), clr_l 7, clr_r 7 | **27** | 297 typical, 340 worst |
| event | seq 8, time 24, kind 2, src 3, code 7, data length 2, data 0/8/16/32 | **7-11** | 60-72 |
| status (1 Hz) | uptime 17 (s), rssi 7, drops 8, has_map 1, map 4 x 6 | **6 / 9** | 110 |
| Last Will | header only | **1** | 16 |
| ack | has_id 1, id 8, error 4 | **3** | 27-41 |
| command (in) | id 8, cmd 3; set: param 4, value 16; map: 4 x 6 | **3 / 5 / 6** | 31 / 63 / 79 |

Field order and none codes: `app_program/subsystem1/comms_proto.h`.

### 3.2 Inter-task messages

| Message | Layout | Size (buffer slot) before -> after |
|---|---|---|
| `mbf_motion` | src 2, cmd 3, seq 5, payload 22: DRIVE v 10 + omega 12; MOVE distance 13 + speed 9; TURN angle 13 + rate 9; SET_AUTH 2 | 8 B (12) -> 4 B (8) |
| `mbf_mission` | type 5, src 3, a 7, data 17 (offset-binary); REMOTE_MAP: 4 x 6-bit symbols | 8 B (12) -> 4 B (8) |
| `mbf_scan` | type 3, seq 5, param 8 | 4 B (8) -> 2 B (8: the kernel rounds to 4) |
| `mbf_telem` | kind 2, src 3, code 7, time 20 (10 ms) + data 32 | 12 B (16) -> 8 B (12) |

### 3.3 Shared snapshots (stored packed; tasks work on an unpacked copy)

| Snapshot | Bits | Before -> after | Notable widths |
|---|---|---|---|
| odometry (IF-02) | 329 | 52 -> 44 B | wheel speeds 10, duty 11, state 3, phase 3, authority 2, seq 5; encoder counts, distance and heading stay 32 (running totals that readers difference) |
| line (IF-04) | 209 | 36 -> 28 B | IR readings 10, line error 11, omega 12, flags 7, barcode state 2, last barcode 6 |
| IMU (IF-08) | 164 | 32 -> 24 B | pitch 15, roll 16, hump peaks 10, hump count 8, hump state 2, motion class 3 |
| mission (IF-10) | 165 | 36 -> 24 B | state 4, cruise 9, ranges 6-10, map 4 x 6, pending command 3, cause 3 |
| obstacle (IF-10) | 166 | 28 -> 24 B | range 12 (with "none"), width 11, lateral edges 12, state 4, plan 3 |
| heartbeat counters | 5 x 8 | 20 -> 5 B | one byte per task |

## 4. Safe access to shared packed data

No task ever does a read-modify-write on a shared packed word:

1. **Snapshots:** one writer task. It packs into a local array, then copies
   the whole record into the shared array under the snapshot's spin-lock
   (`app_lock`, interrupts off on this core plus the hardware spin-lock
   across cores). Readers copy the whole record out under the same lock and
   unpack in their own task. Packing and unpacking run outside the lock, so
   the lock is held for a copy of 6-11 words only.
2. **Messages:** a packed word is built by the sender, copied by the kernel
   (`tk_snd_mbf` / `tk_rcv_mbf`, under the kernel lock) and unpacked by the
   receiver. Nobody shares the word.
3. **Fault bits:** set and cleared only with `tk_set_flg` / `tk_clr_flg` /
   `TWF_BITCLR`, which the kernel makes atomic across both cores and from
   interrupts. No task writes the flag word directly.
4. **Heartbeat counters:** one byte per task; each task stores only its own
   byte (a single `STRB`, indivisible on the M0+), and SafetyTask only reads.
5. **ISR to task (echo time):** one word, the pulse width in 16 bits
   (saturating, so a 65 ms no-echo pulse reads as "no echo", never as a
   short range) plus a done bit. `echo_isr` writes the whole word with one
   store; ObstacleTask reads and clears it with interrupts masked. Both run
   on core 1, so the interrupt cannot fall between the read and the clear
   (`subsystem5/drv_ultrasonic.c`).

## 5. Measured effect (release build, `make NET=lwip MQTT=1`)

| | Before (`e22e527`) | After | Change |
|---|---|---|---|
| MQTT telemetry payload | 297 B typical, 340 B worst | 27 B | -91 % |
| MQTT traffic at 5 Hz telemetry + 1 Hz status | about 1 595 B/s | about 141 B/s | -91 % |
| Static RAM (`.bss`) | 101 800 B | 101 252 B | -548 B |
| Of which: MQTT TX buffer / offline ring / dedup | 384 / 260 / 132 B | 32 / 148 / 100 B | -496 B |
| Of which: the five snapshots + heartbeats | 204 B | 149 B | -55 B |
| Message buffers (kernel heap) | 1 344 B | 992 B | -352 B |
| Flash (`.text`) | 389 290 B | 384 798 B | -4 492 B (the JSON writer and parser are gone) |
| MotionTask run time per cycle (emulator, S2-U07) | 331 us max | 363 us max | +32 us (packing), period limit 10 000 us |

Payload traffic counts the payloads only, not the topic, MQTT, TCP or IP
headers. The timing figure is from the emulator; repeat S2-U07 and S4-U08 on
the robot with the new image for the real values.

## 6. Tests

| ID | What it checks |
|---|---|
| SH-H05 | `app_pack.h`: saturation and offset-binary at every width used, word fields, stream round trip, overflow |
| S1-H01-H21 | every MQTT frame byte for byte against golden vectors; the Python decoder (S1-H21) decodes the same bytes |
| S1-U02, U03, U05 | on the Pico: telemetry frame size, binary command parser, ack frame |
| S2-H16 | motion command word: every command round-trips, saturation, 5-bit seq; odometry layout round-trip |
| S3-H22 | line snapshot layout round-trip and saturation |
| S4-H11 | IMU snapshot layout round-trip and saturation |
| S5-H26 | mission event word (with the map), scan request, mission and obstacle snapshots, "none" values |
| S4-U02 | on the Pico: the 8-bit heartbeat counter advances (modular) |

The robot itself is the final check: one run with the dashboard (telemetry
decoded, commands acknowledged) and the self-test image (S2-U07, S4-U08).

## 7. Decisions and limits

- **Task-local working data stays in natural C types.** Each task unpacks
  into a normal struct and computes with it; packing happens where data is
  stored for, or sent to, another task or the laptop. The M0+ has no
  bit-field instructions, so computing on packed fields would cost time in
  the 10 ms loops for no saving in shared memory. Section 8 gives the range
  analysis of these values, as the Tribunal 1 Version B promised.
- **Running totals stay 32 bits** (encoder counts, distance, heading,
  timestamps, `pings`, `blocks`): their valid range is the whole run, and
  readers take differences, which a shorter wrapping field would break.
- **Sequence numbers are 5 bits.** Senders wrap them with `motion_types_seq_next()`;
  a completion is matched to the latest command, never to one 32 commands old.
- **Command ids are 8 bits.** The laptop counts them up; an id comes back only
  after 256 commands, far outside the 60 s duplicate window.
- **The motion API limits follow the fields:** one MOVE up to 4 095 mm, one
  TURN up to 409.5 deg, speeds and rates up to 511. Every current caller is
  inside these (the longest are the 360 deg calibration spin and 2 m bench
  moves).

## 8. Range analysis of each task's own working values

The Tribunal 1 Version B said the minimum-bit analysis also covers gains,
errors, integrators, filters and intermediates, that keeping them on the
stack or in a task's private memory is no exemption, and that ranges,
precision and overflow must be derived and justified. This section does that
for every value a task keeps for itself. "Bits needed" is the smallest field
that holds the valid range; "kept as" is the C type in the code.

Sizes of each task's private state, release build (`arm-none-eabi-nm -S`):
MotionTask `g_core` 192 B; LineBarcodeTask `g_trk` 56 B, `g_cal` 28 B,
`g_bc` 3 272 B, `g_slip` 4 812 B; ImuTask `g_ic` 224 B; MissionTask `g_mc`
60 B; ObstacleTask `g_o` 292 B, `g_prof` 24 B, `g_steps` 40 B. About 9.0 KB
in all, 3.4 % of the RP2040's 264 KB.

### 8.1 Gains

| Value | Value / range | Bits needed | Kept as | Why |
|---|---|---|---|---|
| Wheel PID `MOTION_KP_MILLI`, `MOTION_KI_MILLI`, `MOTION_KFF_MILLI` | 1040, 6000, 1200 (per mille) | 11, 13, 11 | `#define` | Compile-time constants: they become immediate operands in the code and take no RAM at all. |
| `MOTION_I_MAX`, `MOTION_MIN_DUTY` | 400, 23 | 9, 5 | `#define` | as above |
| Line steering `LINE_KP`, `LINE_KD` | 850, 0 | 10, 1 | `#define` | as above |
| Hold / bypass gains (`OBST_HOLD_K`, ...) | small integers | <= 12 | `#define` | as above |

### 8.2 Integrators and filters

| Value (task) | Valid range | Bits needed | Kept as | Why it stays wider |
|---|---|---|---|---|
| `integ_l_um`, `integ_r_um` wheel PID integrators (Motion) | clamped to +-MOTION_I_MAX x 10^6 / KI = +-66 667 | 18 | `int32_t` | The update `integ + e x dt` is formed before the clamp: e up to +-1 022 mm/s times dt (10 ms, more if a cycle is late) must not overflow before it is clamped. |
| `v_l`, `v_r` filtered wheel speeds (Motion, EMA 1/2) | +-511 mm/s in use | 10 | `int32_t` | The raw speed `dl / dt` before filtering is not bounded (an encoder glitch is reported, not clipped); the filter step `(vraw - v) / 2` needs the headroom. |
| `yaw_rate_cdeg_s` filtered (Motion) | +-32 767 cdeg/s | 16 | `int32_t` | Same: the raw rate is formed from a 32-bit heading difference times 1 000. |
| `acc_f[3]` filtered acceleration, mg x 16 (IMU, EMA 1/4) | +-16 000 mg x 16 = +-256 000 | 19 | `int32_t` | x16 keeps the fraction of the filter; 19 bits do not fit 16. |
| `accel_mm_s2` filtered (IMU) | +-3 000 used (`IMU_ACC_COMP_MAX_MM_S2`) | 13 | `int32_t` | Formed from a speed difference divided by dt before clamping. |
| `h_um`, `h_peak_um` integrated hump height (IMU) | a hump is < 50 mm = 50 000 um | 17 | `int64_t` | Integrated from `distance x sin(pitch)` products in Q15, which exceed 32 bits before the shift. |
| `lvl_sum[3]` level calibration sums (IMU) | up to 1 s of samples x +-16 000 mg | about 21 | `int64_t` | Sum of many samples; 64 bits rules out overflow whatever the sample count. |

### 8.3 Errors and set-points

| Value (task) | Valid range | Bits needed | Kept as | Why it stays wider |
|---|---|---|---|---|
| `err`, `prev_err`, `line_err`, `exit_err`, `peak_err` (Line) | -1 000..1 000 | 11 | `int32_t` | Multiplied by `LINE_KP` (850): the product needs 21 bits. |
| `omega`, `v` steering output (Line) | +-2 000 mrad/s, 0..450 mm/s | 12, 9 | `int32_t` | Ramped and clamped after the multiply above. Sent to MotionTask packed (12 + 10 bits). |
| `sp_v`, `sp_w`, `drive_v`, `drive_w` set-points (Motion) | +-450 mm/s, +-6 000 mrad/s | 10, 14 | `int32_t` | The ramp and braking curve (`v = sqrt(2 x a x remaining)`, remaining in um, a 64-bit product) are computed from and compared with them; 32-bit avoids a widening on every step. |
| `m_target_um`, `m_start_*_um` manoeuvre (Motion) | MOVE <= 4 095 mm = 4.1 x 10^6 um | 23 | `int64_t` | Compared with the 64-bit wheel distances below. |
| `dist_l_um`, `dist_r_um`, `x_um`, `y_um` (Motion) | running totals: about 2 km in a long session | 32 | `int64_t` | Heading = `(dist_r - dist_l) x 18 x 10^6 / (pi x track)`: the product needs about 55 bits. |
| `duty_l`, `duty_r` (Motion) | +-1 000 per mille | 11 | `int16_t` | Smallest C type that holds it. |

### 8.4 Intermediates and buffers

| Value (task) | Valid range | Bits needed | Kept as | Why |
|---|---|---|---|---|
| `widths[60]` bar / space widths (Line, barcode) | an element is at most the 420 mm capture guard = 420 000 um | 19 | `uint32_t` | Width ratios (wide / narrow x 10, sums of 9 elements) are computed on them. |
| `prof[1 411]`, `pre[75]` darkness profile (Line, barcode) | 0..1 000 (normalised) | 10 | `uint16_t` | Indexed and rescanned by up to 7 decoding passes; packed 10-bit entries would need a shift and mask on every access in the decoder. |
| `raw_l[800]`, `raw_r[800]` calibration-spin record (Line) | 0..4 095 (12-bit ADC) | 12 | `uint16_t` | Hardware width: DMA writes the ADC in 16-bit units (Version B: hardware widths are fixed at acquisition). |
| `h_ddeg[800]` spin heading record (Line) | +-3 600 x 0.1 deg | 13 | `int16_t` | Smallest C type that holds it. |
| `min`, `max`, `white`, `black` per channel (Line calibration) | 0..4 095 | 12 | `uint16_t` | ADC hardware width, as above. |
| distances `*_um` in the line tracker | 0 .. a few metres between events | 22 | `uint32_t` | Only added to and compared; 32-bit add and compare are single instructions. |
| `pitch_cdeg`, `roll_cdeg`, `heading_cdeg` (IMU) | +-9 000, +-18 000, 0..35 999 | 15, 16, 16 | `int32_t` | Results of 32-bit atan2 / filter arithmetic; stored packed in the snapshot. |
| `last_count[5]` (Safety) | 8-bit heartbeat counters since CR1 | 8 | `uint32_t` | Compared with the counters for a change; could be 8 bits (saves 15 B). |
| times `*_ms` (all tasks) | 32-bit ms clock, wraps after 49.7 days | 32 | `uint32_t` | Wrap-safe subtraction needs the full clock width. |
| `held_n`, `held_type[4]`, `held_mm[4]` hump events held while all IR are black (IMU, 20 Nov) | 0..4; 2 event types (22, 23); peak 0..1 023 mm (the snapshot's 10-bit hump field) | 3, 5, 10 | `uint8_t`, `uint8_t`, `uint16_t` | Smallest C types that hold them; `held_mm` is the same type as `imu_core_t.last_hump_mm`, which it copies. 13 B on ImuTask's stack. |
| states, phases, flags (all tasks) | 2..13 values; true / false | 1..4 | `uint8_t` / `bool` | One byte is the smallest a C object can be addressed as; a bit-field would add a mask and shift to every test in the 10 ms loops. Stored packed (2..4 bits) wherever another task reads them. |

### 8.5 What full narrowing would save, and what it would cost

If every value above were stored in exactly its "bits needed", the private
state would shrink by about 2.5 KB: 1.1 KB in the darkness profile
(6 spare bits x 1 486 entries), 1.1 KB in the calibration-spin record
(3-4 spare bits x 2 400 entries), 0.1 KB in the bar widths and about 0.2 KB
spread over the rest. That is 0.9 % of the 264 KB SRAM; the current release
uses 101 KB of static RAM, so memory is not short.

The cost would be time and risk in the control loops: the Cortex-M0+ has no
bit-field insert or extract instruction, so each packed field costs a shift,
a mask and (for signed values) a sign extension on every read and write.
Packing only at the task boundary already measured +32 us per MotionTask
cycle on the emulator (S2-U07, section 5). Packing every working value was
not measured; it would apply that cost to every access in the integrators,
filters and decoder loops instead of once per cycle. The team therefore
keeps a task's own working values in the smallest convenient C type,
justified above, and packs everything that is stored for, queued for or
sent to another task or the laptop. Values that are already exactly
representable in a smaller C type (`duty_l` / `duty_r`, `h_ddeg`, the ADC
records) use it.
