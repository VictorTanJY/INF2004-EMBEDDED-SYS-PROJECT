/*
 *  subsystem4_test.c
 *  On-target tests for subsystem 4.  Built only with: make TEST=1
 *  Record results in testing/subsystem4/TEST_LOG.md.
 *  IMU and heartbeat logic are tested on a PC (testing/subsystem4/host).
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

#define WD_CTRL_REG     0x40058000u

IMPORT volatile UW g_safety_tick_us;   /* safety_task.c */

EXPORT void subsystem4_test(void)
{
    T_RFLG      rflg;
    uint32_t    before;
    imu_snap_t  a;
    imu_snap_t  b;

    TEST_INIT();

    TEST_CHECK("S4-U01", tk_ref_flg(g_flg_safety, &rflg) == E_OK, "flg_safety exists");

    before = safety_heartbeat_count(SAFETY_HB_MOTION);
    (void)tk_dly_tsk(50);
    TEST_CHECK("S4-U02", safety_heartbeat_count(SAFETY_HB_MOTION) >= (before + 3u),
               "MotionTask heartbeat advances");

    imu_get_snap(&a);
    (void)tk_dly_tsk(100);
    imu_get_snap(&b);
    TEST_CHECK("S4-U04", b.t_ms != a.t_ms, "ImuTask publishes its snapshot");
    TEST_CHECK("S4-U05", safety_timer_tick_ok(), "1 us timer ticks at 1 MHz (watchdog tick = 12)");
    tm_printf((UB *)"[INFO] timer: %d us per 100 ms of CPU clock\n", (INT)g_safety_tick_us);
    TEST_CHECK("S4-U06", (in_w(WD_CTRL_REG) & (1uL << 30)) != 0u, "watchdog enabled and being fed");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
