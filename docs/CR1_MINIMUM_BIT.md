# Change Requirement 1: minimum-bit data representation

How the firmware meets Change Requirement 1, where each part is in the
code, how it is tested, and what it changed (measured, before and after).

**Before:** commit `e22e527` (JSON MQTT, full-width fields).
**After:** this branch from `619b823` (MQTT) onwards.
Raw measurements: `testing/system/logs/2026-11-15_cr1_before_after.txt`.

## 1. What changed, by the eight areas of the requirement

| # | Area | What is stored in minimum bits | Where |
|---|---|---|---|
| 1 | Motion control and encoder data | `mbf_motion` command: one 32-bit word. Odometry snapshot: 329 bits (11 words) | `motion_core.c` (`motion_cmd_pack`, `k_odom_layout`) |
| 2 | Line and barcode sensing | Line snapshot: 208 bits (7 words); barcode character as a 6-bit Code 39 symbol | `line_track.c` (`k_line_layout`) |
| 3 | Obstacle detection | Obstacle snapshot: 166 bits (6 words); `mbf_scan` request: 16 bits | `obstacle_core.c`, `mission_core.c` |
| 4 | Hump estimation | IMU snapshot (pitch, humps, motion class): 164 bits (6 words) | `imu_core.c` (`k_imu_layout`) |
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
- Words in RAM use explicit masks and shifts (`pk_set` / `pk_get`); MQTT
  frames are written least significant bit first (`pk_put` / `pk_take`), so
  the layout does not depend on the compiler or byte order.
- Each snapshot has a **layout table** (`pk_field_t`): one line per field with
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
| line (IF-04) | 208 | 36 -> 28 B | IR readings 10, line error 11, omega 12, flags 7, barcode state 2, last barcode 6 |
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
  the 10 ms loops for no saving in shared memory.
- **Running totals stay 32 bits** (encoder counts, distance, heading,
  timestamps, `pings`, `blocks`): their valid range is the whole run, and
  readers take differences, which a shorter wrapping field would break.
- **Sequence numbers are 5 bits.** Senders wrap them with `motion_seq_next()`;
  a completion is matched to the latest command, never to one 32 commands old.
- **Command ids are 8 bits.** The laptop counts them up; an id comes back only
  after 256 commands, far outside the 60 s duplicate window.
- **The motion API limits follow the fields:** one MOVE up to 4 095 mm, one
  TURN up to 409.5 deg, speeds and rates up to 511. Every current caller is
  inside these (the longest are the 360 deg calibration spin and 2 m bench
  moves).
