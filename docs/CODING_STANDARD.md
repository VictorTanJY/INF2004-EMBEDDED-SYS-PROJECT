# Coding standard: Barr-C 2018

The team's C code follows the *Embedded C Coding Standard* (Barr Group,
2018). This page says which files that covers, how to keep new code
conforming, and where the code knowingly departs from a rule.

## 1. Scope

| Covered (113 files, about 29,400 lines) | Not covered |
|---|---|
| `app_program/` (firmware application) | `kernel/`, `include/`, `device/`, `config/` (project template) |
| `testing/*/host/` (host unit tests) | `lib/` other than the two files on the left (template, lwIP, CYW43 driver) |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `.h` (the template's MQTT client, mostly rewritten by the team) | `dashboard/` (Python, JavaScript) |

Template files keep the template's own style so they can still be compared
with the upstream template. That includes the 22 lines the team added to
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`.

## 2. Keeping code conforming

Run the formatter before every commit:

```
tools/barr_format.sh
```

It needs clang-format 23 (`pip install clang-format==23.1.5`) and Python 3.
It runs clang-format with the repository's `.clang-format`, then
`tools/barr_post_format.py` for the three layout rules clang-format has no
option for. Always use the script, not clang-format alone.

| Done by the script | Rule |
|---|---|
| Lines of at most 80 characters, 4-space indents, no tabs | 1.2, 3.4, 3.5 |
| Braces around every block, each brace on its own line | 1.3 |
| Spacing around keywords, operators, commas; a space before `(` in a function definition only; `*` with a space on both sides in declarations | 3.1 |
| Aligned declarations, struct members and adjacent assignments | 3.2 |
| `const` to the right of the type it qualifies | house style |
| A final `else` on every `else if` chain | 8.2d |
| Each `break` aligned with its `case` | 8.3a |
| `/*** end of file ***/` and a newline at the end of every file | 3.3c |

The script cannot do the rest, so write new code this way:

| Write it like this | Rule |
|---|---|
| A Doxygen comment (`/** ... */`) for every file (`@file`, `@brief`) and every function, in the header for public functions and above the definition for static ones | 2.2h |
| No `//`, `/*` or `\` inside a comment (write a web address without `https://`) | 2.1b |
| A blank line before and after every `if` / `else`, loop and `switch` block, and after a run of declarations. A comment that describes the block goes directly above it, after the blank line | 3.3b |
| One `return` per function, as its last statement: use a result variable, a loop flag or an `else` instead of an early `return` | 6.2c |
| Functions of at most 100 lines, header to closing brace: move a self-contained step into a `static` helper | 6.2a |
| A comment above every statement that casts, naming the cast and saying why the value fits the new type (`/* Cast (uint16_t)(n_elems - 2u): n_elems >= 11 (tested above). */`). Leave out a cast that converts nothing | 1.6a |
| Names of at least 3 characters, lower case with underscores | 7.1 |
| Prefixes, in this order before the first underscore: `g` file-scope or global, `p` pointer (`pp` pointer to pointer), `b` boolean, `h` handle. Examples: `g_ring`, `p_cmd`, `b_valid`, `gb_online`, `gp_client`, `pb_reversed` | 7.1 |
| Public functions and types start with the module name (`comms_proto_parse_cmd`, `motion_types_cmd_t`) | 6.1i, 5.1c |
| No global variables in headers: a kernel object ID or a shared value is read through a function (`subsystem2_mbf_motion()`) | 4.2c |
| Types end in `_t` and are always made with `typedef`, never a bare tag | 5.1 |
| Macros and constants in upper case; no number other than 0 or 1 in a loop test | 6.1, 7.1, 8.4a |
| Fixed-width types (`uint8_t`, `int32_t`, ...) and `bool`; `float32_t` / `float64_t` from `app_types.h` if floating point is ever needed | 5.2, 5.4, 5.6 |
| One variable per declaration | 8.1a |
| The constant on the left of `==` (`if (0u == count)`) | 8.6a |
| Parentheses around every operand of `&&` and `\|\|` that is not a single name or constant | 1.4 |
| At most two levels of `if` inside one another | 8.2b |
| A `default` in every `switch`; `for (;;)` for an endless loop | 8.3b, 8.4c |
| A `static inline` function instead of a macro with parameters, unless a function cannot do the job | 6.3a |
| Interrupt handlers end in `_isr`, task functions in `_task` | 6.4, 6.5 |
| No `goto`, no `continue`, no assignment inside an `if` or loop condition | 1.7, 8.2c, 8.4b |

## 3. How conformance was checked

- Checkers: a script over the source text for the layout rules; a Clang
  AST scan of every translation unit (nine firmware configurations and all
  host tests, with the real compiler flags) for names, types,
  declarations, casts, `&&` / `||` operands, `if` nesting, `switch`
  defaults, loop forms and file section order; scripts for function
  length, returns per function, blank lines around blocks, and a cast
  comment above every cast; Doxygen (`docs/Doxyfile`) for undocumented
  functions. None of them reports an open item outside section 4.
- Changes that only rename, reformat or comment (most of the work) leave
  every firmware object byte-for-byte identical, compared function by
  function in the nine configurations (release, bench, timing, self-test,
  UART, UART self-test, single-core USB, flipped motors, flipped self-test).
- Changes that restructure code (one return per function, splitting long
  functions) change the machine code of about 60 functions. Their
  behaviour was checked by: the host tests (1,258 checks) passing with the
  same output and the same check results in the same order; differential
  fuzzing of the host-testable modules against the code before the change
  (about 260 million random inputs, 0 differences); the on-target self-tests
  in the RP2040 emulator printing the same `[TEST]` lines; valgrind on every
  host test (0 reports). The emulator does not model the motors, sensors or
  radio, so the drivers changed for 6.2c (I2C, LSM303, ADC, ultrasonic,
  encoders) still need the usual bench test on the car.

## 4. Known departures

"Preferred" marks rules the standard itself words as preferred practice.

| Rule | Where | Why it stays |
|---|---|---|
| 4.1b module names unique in their first 8 characters | `subsystem1` ... `subsystem5`, `mission_*`, `obstacle_*`, `motion_core` / `motion_cfg` | Renaming the files would change every build file, include and saved test log that names them. The names are unique in full. |
| 6.1i public functions start with the module name | `usermain` (`app_main.c`), `mqtt_utk_app_config`, `_connected`, `_disconnected`, `_poll`, `_rx` (`comms_service.c`), the host test stub `subsystem2_post` (`test_s2_main.c`) | `usermain` is called by micro T-Kernel; the `mqtt_utk_app_*` hooks are called by name from `mqtt_utk.c`; the stub stands in for the firmware function of that name. |
| 5.1c public types start with the module name | `float32_t`, `float64_t` (`app_types.h`) | The standard's own names for floating-point types (5.4), shared by every module. |
| 5.2a no plain `int` | `int main` in the host tests; `motion_api_err_t` (`typedef int`) | `main` is fixed by the C standard. The motion API returns the kernel's `ER`, which is an `int`. |
| 6.3a no macros with parameters | `PK_WORDS` (`app_pack.h`), `TEST_CHECK` (`test_report.h`), `CHECK` in each host test | `PK_WORDS` sizes arrays, so it must be a constant expression; the check macros pass `__FILE__` and `__LINE__` of the caller. |
| 2.1c(ii) debug output inside `#ifndef NDEBUG` | `imu_debug_logs()` (`imu_task.c`), under `#if IMU_DEBUG_LOGS` | The template defines `NDEBUG` in the USB and WiFi builds, and these bring-up logs go out over MQTT, so `#ifndef NDEBUG` would make them impossible to turn on. They are off by default (`IMU_DEBUG_LOGS 0` in `imu_cfg.h`). |
| 5.4b(v) `isfinite()` after floating-point calculations | the physics models in the host tests (`float64_t`); the firmware has no floating point | The models only produce test inputs and expected values. Most checks have the form `fabs(got - ref) <= tol`, which a NaN fails (every comparison with NaN is false). |
| 1.6a each cast has a comment | the 254 casts to `(void)` | They discard a return value or mark an unused parameter; there is no value whose range a comment could explain. Every other cast has its comment. |
| Kernel-imposed names and types | `EXPORT` / `IMPORT` / `LOCAL`, `ER`, `ID`, `INT`, `UINT`, `UB`, `FP`, `T_CTSK` and other `T_*` packets | Fixed by micro T-Kernel. |
