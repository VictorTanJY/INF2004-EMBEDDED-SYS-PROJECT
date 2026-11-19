/*
 *  test_report.h
 *  Minimal on-target test helpers.
 *
 *  Every check prints ONE line in a fixed format, so a saved console log
 *  can be searched for results ("grep -F '[TEST]' log.txt"):
 *
 *      [TEST] S1-U01 PASS  ADC reads in range
 *      [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *      [TEST] S2-U08 SKIP  wheels did not turn   (no encoder counts)
 *      [TEST] subsystem1 SUMMARY 1 passed, 1 failed, 0 skipped
 *
 *  Usage, inside a test function:
 *      test_report_t report = {0, 0, 0};
 *      TEST_CHECK(&report, "S1-U01", (value >= 0) && (value <= 4095),
 *                 "ADC reads in range");
 *      test_report_summary(&report, "subsystem1");
 *
 *  Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md.
 *
 *  TEST_SKIP is for a check whose set-up is missing (no wall in front,
 *  motor battery off, or the emulator, which has no sensors or motors).
 *  A SKIP is NOT a pass: on the robot, fix the set-up and run again.
 *  Measured values are printed on "[INFO]" lines for the test log.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <stdbool.h>
#include <tk/tkernel.h>

/* Results of one subsystem's test run. */
typedef struct
{
    INT pass;
    INT fail;
    INT skip;
} test_report_t;

void test_report_check(test_report_t * p_report, char const * p_id, bool b_ok,
                       char const * p_desc, char const * p_file, INT line);
void test_report_skip(test_report_t * p_report, char const * p_id,
                      char const * p_desc, char const * p_why);
void test_report_summary(test_report_t const * p_report, char const * p_name);

/* NOTE: a macro rather than a function only so that a failure reports the
   caller's file and line (__FILE__, __LINE__). */
#define TEST_CHECK(p_report, id, cond, desc) \
    (test_report_check((p_report), (id), (cond), (desc), __FILE__, __LINE__))

#endif /* TEST_REPORT_H */

/*** end of file ***/
