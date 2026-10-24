# Subsystem 2 — test log

**Subsystem:** Motion control (MotionTask)  ·  **Student ID:** 2500917  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | mbf_motion exists | IF-01 | auto | `subsystem2_test.c` | PASS |
| S2-U02 | odometry snapshot round trip under its lock | IF-02 | auto | `subsystem2_test.c` | PASS |

_Shared-base checks (interface objects and snapshots). The owner adds the subsystem's own tests below them._

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
