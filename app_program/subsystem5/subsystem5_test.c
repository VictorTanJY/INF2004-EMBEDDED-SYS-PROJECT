/*
 *  subsystem5_test.c
 *  On-target tests for subsystem 5.  Built only with: make TEST=1
 *  Record results in testing/subsystem5/TEST_LOG.md.
 *  The statechart and geometry are tested on a PC (testing/subsystem5/host).
 *
 *  S5-U08/U09 put ObstacleTask in MONITOR (as a run start does: servo at
 *  90 deg, one ping every 60 ms) and stop it again afterwards.
 *  S5-U09 wants a flat object (a wall or a box) 30-100 cm in front of the
 *  ultrasonic sensor; write the tape-measured distance next to the
 *  printed one in the test log.
 */

#include "subsystem5.h"
#include "obstacle_core.h"
#include "../test_report.h"

#if APP_TEST

/* S5-U08: range_cyc pings every S5_RANGE_PERIOD_MS; count them over this
   window and allow one ping either way (start and end of the window). */
#define T_PING_WIN_MS   1200u
#define T_PINGS         (T_PING_WIN_MS / S5_RANGE_PERIOD_MS)
#define T_SETTLE_MS     600u
#define T_SCAN_SEQ      0xE0u       /* not used by MissionTask */
#define T_RANGE_MIN_MM  20u         /* HC-SR04 datasheet: 2 cm .. 4 m */
#define T_RANGE_MAX_MM  4000u

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

    /* ---- S5-U08: ping rate ------------------------------------------ */
    (void)scan_request((uint8_t)SCAN_REQ_MONITOR, T_SCAN_SEQ, 0);
    (void)tk_dly_tsk(T_SETTLE_MS);          /* servo to 90 deg, first 3 pings */
    obst_get_snap(&a);
    (void)tk_dly_tsk(T_PING_WIN_MS);
    obst_get_snap(&b);
    tm_printf((UB *)"[INFO] S5-U08 %d pings in %d ms (expected %d)\n",
              (INT)(b.pings - a.pings), (INT)T_PING_WIN_MS, (INT)T_PINGS);
    TEST_CHECK("S5-U08", ((b.pings - a.pings) + 1u >= T_PINGS) && ((b.pings - a.pings) <= T_PINGS + 1u),
               "ultrasonic pings every 60 ms");

    /* ---- S5-U09: one range against the object in front -------------- */
    if (b.d_mm == OBST_RANGE_NONE)
    {
        TEST_SKIP("S5-U09", "ultrasonic range to the object in front",
                  "no echo: put a wall 30-100 cm ahead, or emulator");
    }
    else
    {
        tm_printf((UB *)"[INFO] S5-U09 range %d mm (median of 3); compare with a tape measure\n",
                  (INT)b.d_mm);
        TEST_CHECK("S5-U09", (b.d_mm >= T_RANGE_MIN_MM) && (b.d_mm <= T_RANGE_MAX_MM) && (b.no_echo == 0u),
                   "ultrasonic echo received, range within 2 cm .. 4 m");
    }

    (void)scan_request((uint8_t)SCAN_REQ_STOP, (uint8_t)(T_SCAN_SEQ + 1u), 0);

    TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
