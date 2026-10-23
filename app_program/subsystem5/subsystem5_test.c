/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5.  Built only with: make TEST=1
 *  Record results in testing/subsystem5/TEST_LOG.md.
 *
 *  Shared-base checks only; the owner adds the mission and obstacle tests.
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
    T_RMBF          rmbf;
    T_RFLG          rflg;
    mission_snap_t  snap;

    TEST_INIT();

    TEST_CHECK("S5-U01", tk_ref_mbf(g_mbf_mission, &rmbf) == E_OK,
               "mbf_mission exists");
    TEST_CHECK("S5-U02", tk_ref_mbf(g_mbf_scan, &rmbf) == E_OK,
               "mbf_scan exists");
    TEST_CHECK("S5-U03", tk_ref_flg(g_flg_obst, &rflg) == E_OK,
               "flg_obst exists");

    mission_get_snap(&snap);
    TEST_CHECK("S5-U04", snap.state == (uint8_t)MISSION_ST_BOOT,
               "mission snapshot starts in BOOT");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
