/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5.  Built only with: make TEST=1
 *  Record results in testing/subsystem5/TEST_LOG.md.
 *  The statechart and geometry are tested on a PC (testing/subsystem5/host).
 */

#include "subsystem5.h"
#include "obstacle_core.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem5_test(void)
{
    T_RMBF          rmbf;
    T_RFLG          rflg;
    mission_snap_t  ms;
    obst_snap_t     a;
    obst_snap_t     b;

    TEST_INIT();

    TEST_CHECK("S5-U01", tk_ref_mbf(g_mbf_mission, &rmbf) == E_OK, "mbf_mission exists");
    TEST_CHECK("S5-U02", tk_ref_mbf(g_mbf_scan, &rmbf) == E_OK, "mbf_scan exists");
    TEST_CHECK("S5-U03", tk_ref_flg(g_flg_obst, &rflg) == E_OK, "flg_obst exists");

    mission_get_snap(&ms);
    TEST_CHECK("S5-U05", (ms.state < (uint8_t)MISSION_ST_COUNT) && (ms.cruise_mm_s > 0u) &&
                         (ms.nav_map[0] == '\0'),
               "mission snapshot valid; no barcode map until MQTT sends one");

    obst_get_snap(&a);
    (void)tk_dly_tsk(150);
    obst_get_snap(&b);
    TEST_CHECK("S5-U06", b.t_ms != a.t_ms, "ObstacleTask runs (range_cyc / 100 ms wait)");
    TEST_CHECK("S5-U07", (obst_echo_to_mm(1160u) == 200u) && (obst_median3(300u, 120u, 200u) == 200u),
               "echo conversion and median on target");

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
