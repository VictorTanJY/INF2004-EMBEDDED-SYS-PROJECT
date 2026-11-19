/** @file test_report.c
 *
 *  @brief Result lines of the on-target tests (format in test_report.h).
 *  Built only with make TEST=1.
 */

#include "test_report.h"
#include <tm/tmonitor.h>
#include "app_config.h"

#if APP_TEST

/*!
 * @brief Count one check and print its PASS or FAIL line.
 * @param[in,out] p_report The run's counters.
 * @param[in] p_id Test id, as in testing/subsystemN/TEST_LOG.md.
 * @param[in] b_ok Whether the check passed.
 * @param[in] p_desc What was checked.
 * @param[in] p_file Source file of the check (FAIL lines only).
 * @param[in] line Source line of the check (FAIL lines only).
 */
void test_report_check (test_report_t * p_report, char const * p_id, bool b_ok,
                        char const * p_desc, char const * p_file, INT line)
{
    if (b_ok)
    {
        p_report->pass++;
        tm_printf((UB *)"[TEST] %s PASS  %s\n", p_id, p_desc);
    }
    else
    {
        p_report->fail++;
        tm_printf((UB *)"[TEST] %s FAIL  %s   (%s:%d)\n", p_id, p_desc, p_file,
                  line);
    }
}

/*!
 * @brief Count one skipped check and print its SKIP line.
 * @param[in,out] p_report The run's counters.
 * @param[in] p_id Test id.
 * @param[in] p_desc What would have been checked.
 * @param[in] p_why The missing set-up.
 */
void test_report_skip (test_report_t * p_report, char const * p_id,
                       char const * p_desc, char const * p_why)
{
    p_report->skip++;
    tm_printf((UB *)"[TEST] %s SKIP  %s   (%s)\n", p_id, p_desc, p_why);
}

/*!
 * @brief Print the run's SUMMARY line.
 * @param[in] p_report The run's counters.
 * @param[in] p_name Subsystem name.
 */
void test_report_summary (test_report_t const * p_report, char const * p_name)
{
    tm_printf((UB *)"[TEST] %s SUMMARY %d passed, %d failed, %d skipped\n",
              p_name, p_report->pass, p_report->fail, p_report->skip);
}

#endif /* APP_TEST */

/*** end of file ***/
