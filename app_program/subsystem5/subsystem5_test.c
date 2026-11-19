/** @file subsystem5_test.c
 *
 *  @brief On-target tests for subsystem 5.  Built only with: make TEST=1
 *  Record results in testing/subsystem5/TEST_LOG.md.
 *  The statechart and geometry are tested on a PC (testing/subsystem5/host).
 *
 *  S5-U08/U09 put ObstacleTask in MONITOR (as a run start does: servo at
 *  90 deg, one ping every 60 ms) and stop it again afterwards.
 *  S5-U09 wants a flat object (a wall or a box) 30-100 cm in front of the
 *  ultrasonic sensor; write the tape-measured distance next to the
 *  printed one in the test log.
 */

#include "subsystem5_test.h"
#include "subsystem5.h"
#include "obstacle_core.h"
#include "../test_report.h"

#if APP_TEST

/* S5-U08: range_cyc pings every S5_RANGE_PERIOD_MS; count them over this
   window and allow one ping either way (start and end of the window). */
#define T_PING_WIN_MS  1200u
#define T_PINGS        (T_PING_WIN_MS / S5_RANGE_PERIOD_MS)
#define T_SETTLE_MS    600u
#define T_SCAN_SEQ     0xE0u /* not used by MissionTask */
#define T_RANGE_MIN_MM 20u   /* HC-SR04 datasheet: 2 cm .. 4 m */
#define T_RANGE_MAX_MM 4000u

EXPORT void subsystem5_test_run (void)
{
    T_RMBF                    rmbf;
    T_RFLG                    rflg;
    mission_types_snap_t      mission;
    mission_types_obst_snap_t snap_a;
    mission_types_obst_snap_t snap_b;

    test_report_t report = {0, 0, 0};

    TEST_CHECK(&report, "S5-U01",
               E_OK == tk_ref_mbf(subsystem5_mbf_mission(), &rmbf),
               "mbf_mission exists");
    TEST_CHECK(&report, "S5-U02",
               E_OK == tk_ref_mbf(subsystem5_mbf_scan(), &rmbf),
               "mbf_scan exists");
    TEST_CHECK(&report, "S5-U03",
               E_OK == tk_ref_flg(subsystem5_flg_obst(), &rflg),
               "flg_obst exists");

    subsystem5_get_mission_snap(&mission);
    TEST_CHECK(&report, "S5-U05",
               (mission.state < MISSION_ST_COUNT) && (mission.cruise_mm_s > 0u)
                   && ('\0' == mission.nav_map[0]),
               "mission snapshot valid; no barcode map until MQTT sends one");

    subsystem5_get_obst_snap(&snap_a);
    (void)tk_dly_tsk(150);
    subsystem5_get_obst_snap(&snap_b);
    TEST_CHECK(&report, "S5-U06", snap_b.t_ms != snap_a.t_ms,
               "ObstacleTask runs (range_cyc / 100 ms wait)");
    TEST_CHECK(&report, "S5-U07",
               (200u == obstacle_core_echo_to_mm(1160u))
                   && (200u == obstacle_core_median3(300u, 120u, 200u)),
               "echo conversion and median on target");

    /* ---- S5-U08: ping rate ------------------------------------------ */
    (void)subsystem5_scan_request(SCAN_REQ_MONITOR, T_SCAN_SEQ, 0);
    (void)tk_dly_tsk(T_SETTLE_MS); /* servo to 90 deg, first 3 pings */
    subsystem5_get_obst_snap(&snap_a);
    (void)tk_dly_tsk(T_PING_WIN_MS);
    subsystem5_get_obst_snap(&snap_b);
    tm_printf((UB *)"[INFO] S5-U08 %d pings in %d ms (expected %d)\n",
              (INT)(snap_b.pings - snap_a.pings), (INT)T_PING_WIN_MS,
              (INT)T_PINGS);
    TEST_CHECK(&report, "S5-U08",
               (((snap_b.pings - snap_a.pings) + 1u) >= T_PINGS)
                   && ((snap_b.pings - snap_a.pings) <= (T_PINGS + 1u)),
               "ultrasonic pings every 60 ms");

    /* ---- S5-U09: one range against the object in front -------------- */
    if (OBST_RANGE_NONE == snap_b.d_mm)
    {
        test_report_skip(&report, "S5-U09",
                         "ultrasonic range to the object in front",
                         "no echo: put a wall 30-100 cm ahead, or emulator");
    }
    else
    {
        tm_printf((UB *)"[INFO] S5-U09 range %d mm (median of 3); compare with "
                        "a tape measure\n",
                  (INT)snap_b.d_mm);
        TEST_CHECK(&report, "S5-U09",
                   (snap_b.d_mm >= T_RANGE_MIN_MM)
                       && (snap_b.d_mm <= T_RANGE_MAX_MM)
                       && (0u == snap_b.no_echo),
                   "ultrasonic echo received, range within 2 cm .. 4 m");
    }

    (void)subsystem5_scan_request(SCAN_REQ_STOP, (uint8_t)(T_SCAN_SEQ + 1u), 0);

    test_report_summary(&report, "subsystem5");
}

#endif /* APP_TEST */

/*** end of file ***/
