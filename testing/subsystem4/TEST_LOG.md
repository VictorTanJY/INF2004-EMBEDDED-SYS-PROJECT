# Subsystem 4 — test log

**Subsystem:** IMU terrain monitoring (ImuTask) and safety (SafetyTask)  ·  **Student ID:** 2503229  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | flg_safety exists | IF-05 | auto | `subsystem4_test.c` | PASS |
| S4-U02 | heartbeat counter increments | IF-07 | auto | `subsystem4_test.c` | PASS |
| S4-U03 | IMU snapshot round trip under its lock | IF-08 | auto | `subsystem4_test.c` | PASS |

_Shared-base checks (interface objects and snapshots). The owner adds the subsystem's own tests below them._

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
