/*
 *  subsystem2_test.c
 *  On-target tests for subsystem 2.  Built only with: make TEST=1
 *  Record results in testing/subsystem2/TEST_LOG.md.
 *  The controller logic itself is tested on a PC (testing/subsystem2/host).
 */

#include "subsystem2.h"
#include "motion_cfg.h"
#include "../test_report.h"

#if APP_TEST

#define PWM_TOP_SLICE4      (PWM_BASE + (0x14u * 4u) + 0x10u)
#define PIO0_CTRL_REG       0x50200000u

EXPORT void subsystem2_test(void)
{
    T_RMBF          rmbf;
    motion_odom_t   a;
    motion_odom_t   b;

    TEST_INIT();

    TEST_CHECK("S2-U01", tk_ref_mbf(g_mbf_motion, &rmbf) == E_OK, "mbf_motion exists");

    motion_get_odom(&a);
    (void)tk_dly_tsk(50);
    motion_get_odom(&b);
    TEST_CHECK("S2-U03", b.t_ms != a.t_ms, "MotionTask publishes odometry every cycle");
    TEST_CHECK("S2-U04", (b.hw_ok == 1u) && (in_w(PWM_TOP_SLICE4) == MOTION_PWM_TOP) &&
                         ((in_w(PIO0_CTRL_REG) & 0x3u) == 0x3u),
               "PWM 20 kHz configured, PIO0 SM0/SM1 counting");
    TEST_CHECK("S2-U05", (b.state == (uint8_t)MOTION_ST_IDLE_BRAKED) && (b.duty_l == 0) && (b.duty_r == 0),
               "boots braked (duty 0/0)");

    /* An unauthorised DRIVE is rejected and counted (authority, Section 6.1). */
    (void)motion_post((uint8_t)MOTION_SRC_LINE, 1u, (uint8_t)MOTION_CMD_DRIVE, 100, 0);
    (void)tk_dly_tsk(30);
    motion_get_odom(&a);
    TEST_CHECK("S2-U06", (a.rejected > b.rejected) && (a.state == (uint8_t)MOTION_ST_IDLE_BRAKED),
               "DRIVE without LINE authority rejected");

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
