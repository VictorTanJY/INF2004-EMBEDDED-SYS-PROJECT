/*
 *  subsystem2_test.c
 *  On-target tests for subsystem 2.  Built only with: make TEST=1
 *  Record results in testing/subsystem2/TEST_LOG.md.
 *
 *  Shared-base checks only; the owner adds the motion tests.
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
    T_RMBF          rmbf;
    motion_odom_t   odom;

    TEST_INIT();

    TEST_CHECK("S2-U01", tk_ref_mbf(g_mbf_motion, &rmbf) == E_OK,
               "mbf_motion exists");

    odom.dist_mm = 1234;
    motion_put_odom(&odom);
    odom.dist_mm = 0;
    motion_get_odom(&odom);
    TEST_CHECK("S2-U02", odom.dist_mm == 1234, "odometry snapshot round trip");

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
