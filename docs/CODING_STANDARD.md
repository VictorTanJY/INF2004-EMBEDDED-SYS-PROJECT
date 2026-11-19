# Coding standard: Barr-C 2018

The team's C code follows the *Embedded C Coding Standard* (Barr Group,
2018). This page says which files that covers, how to keep new code
conforming, and where the code knowingly departs from a rule.

## 1. Scope

| Covered (91 files, about 23,500 lines) | Not covered |
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
| `const` to the right of the type it qualifies | house style |
| A final `else` on every `else if` chain | 8.2d |
| Each `break` aligned with its `case` | 8.3a |
| `/*** end of file ***/` and a newline at the end of every file | 3.3c |

The script cannot do the rest, so write new code this way:

| Write it like this | Rule |
|---|---|
| Names of at least 3 characters, lower case with underscores | 7.1 |
| Prefixes, in this order before the first underscore: `g` file-scope or global, `p` pointer (`pp` pointer to pointer), `b` boolean, `h` handle. Examples: `g_ring`, `p_cmd`, `b_valid`, `gb_online`, `gp_client`, `pb_reversed` | 7.1 |
| Types end in `_t` and are always made with `typedef`, never a bare tag | 5.1 |
| Macros and constants in upper case | 6.1, 7.1 |
| Fixed-width types (`uint8_t`, `int32_t`, ...) and `bool`; `float32_t` / `float64_t` from `app_types.h` if floating point is ever needed | 5.2, 5.4, 5.6 |
| One variable per declaration | 8.1a |
| The constant on the left of `==` (`if (0u == count)`) | 8.6a |
| Parentheses around every operand of `&&` and `\|\|` that is not a single name or constant | 1.4 |
| A `default` in every `switch`; `for (;;)` for an endless loop | 8.3b, 8.4c |
| A `static inline` function instead of a macro with parameters, unless a function cannot do the job | 6.3a |
| Interrupt handlers end in `_isr`, task functions in `_task` | 6.4, 6.5 |
| No `goto`, no assignment inside an `if` or loop condition | 1.7, 8.2c, 8.4b |

## 3. How conformance was checked

- A script over the source text for the layout rules, and a Clang AST scan
  of every translation unit (release, bench, self-test and timing firmware
  builds, and all host tests, with the real compiler flags) for names,
  types, declarations, `&&` / `||` operands, `switch` defaults, loop
  forms and tagged types. Both report no open item outside section 4.
- The conversion changed names, layout and spelling only. The firmware
  images built before and after it are byte-for-byte identical (release,
  bench, timing, and the self-test image with `__LINE__` held constant),
  and the host tests print the same output (21, 193 + 16, 59, 161, 70,
  274 checks, 0 failed).

## 4. Known departures

"Preferred" marks rules the standard itself words as preferred practice.

| Rule | Where | Why it stays |
|---|---|---|
| 1.6a each cast has a comment | about 1,480 casts | The value ranges are explained once, at the field tables and limits they come from, not at every cast. |
| 1.7d no `continue` (preferred) | 14 uses: 9 firmware, 5 host tests | Guard clauses at the top of long loop bodies. Removing them would nest the task loops one level deeper. |
| 6.2c one exit per function (preferred) | 58 functions | Early returns on error or "nothing to do". |
| 6.2a functions of at most 100 lines | 9 firmware functions: the task loops `line_task` (294), `imu_task` (383), the obstacle state machine `slot` (627), `plan` (117), `probe_slot` (103) and four `subsystemN_test` self-tests; 13 host test functions | Splitting the task loops and the obstacle state machine means moving their shared local state, which is a functional change and needs a full re-test on the car. |
| 6.3a no macros with parameters | 13 macros: `CHECK`, `TEST_INIT`, `TEST_CHECK`, `TEST_SKIP`, `TEST_SUMMARY`, the host tests' `CHECK`, `PK_FIELD`, `PK_WORDS` | A function cannot do the same: they use `__FILE__`, `__LINE__` or `#cond`, declare variables in the caller, take a type and member name (`offsetof`), or must be a constant expression for an array size. |
| 5.2 fixed-width types only | `int` return type of the `motion_*` API; `int main` in host tests; `char` for barcode characters and text | The motion API must match the kernel's error type `ER` (an `int`). `main` is fixed by the C standard. Barcode characters are text. |
| 4.1d a file with `main()` has "main" in its name | `testing/*/host/test_*.c` | The saved test logs name these files in their build commands. |
| 4.2a one header per source file, same name | No header: the task files, `comms_service.c`, `app_main.c`, the `subsystemN_test.c` self-tests (their entry points are declared in the subsystem header). No source: `*_cfg.h`, `*_types.h`, `app.h`, `app_config.h`, `app_types.h`, `test_report.h` | Each subsystem publishes one interface header (`subsystemN.h`); tuning values and shared types have their own headers. |
| 4.3b order of sections in a source file | most `.c` files | Private helpers are defined above the public functions that use them, so no separate prototype list is needed. |
| 2.2 Doxygen-style function comments | everywhere | Plain block comments are used. |
| Kernel-imposed names and types | `usermain`, `EXPORT` / `IMPORT` / `LOCAL`, `ER`, `ID`, `INT`, `UB` | Fixed by micro T-Kernel. |
