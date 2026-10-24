# Subsystem 5 — test log

**Subsystem:** Obstacle scanning (ObstacleTask) and mission state machine (MissionTask)  ·  **Student ID:** 2500443  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | mbf_mission exists | IF-06 | auto | `subsystem5_test.c` | PASS |
| S5-U02 | mbf_scan exists | IF-09 | auto | `subsystem5_test.c` | PASS |
| S5-U03 | flg_obst exists | IF-09 | auto | `subsystem5_test.c` | PASS |
| S5-U04 | mission snapshot starts in BOOT | IF-10 | auto | `subsystem5_test.c` | PASS |

_Shared-base checks (interface objects and snapshots). The owner adds the subsystem's own tests below them._

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
