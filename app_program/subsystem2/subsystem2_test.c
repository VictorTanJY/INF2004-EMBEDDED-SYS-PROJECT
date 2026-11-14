/*
 *  subsystem2_test.c
 *  On-target tests for subsystem 2.  Built only with: make TEST=1
 *  Record results in testing/subsystem2/TEST_LOG.md.
 *  The controller maths is tested on a PC (testing/subsystem2/host).
 *
 *  S2-U08..U11 turn the wheels: put the robot on a stand with the wheels
 *  off the floor (on the floor it drives 30 cm forward, 30 cm back and
 *  turns 90 deg left).  The motor battery switch must be on.
 */

#include "subsystem2.h"
#include "motion_cfg.h"
#include "../subsystem5/subsystem5.h"       /* mission_get_snap: wait for IDLE */
#include "../test_report.h"

#if APP_TEST

#define PWM_TOP_SLICE4      (PWM_BASE + (0x14u * 4u) + 0x10u)   /* PWM CH4_TOP */
#define PIO0_CTRL_REG       0x50200000u

/* S2-U07: the cycle is motion_cyc (S2_PERIOD_MS).  One kernel tick of
   jitter is allowed either way between the tick interrupt on core 0 and
   the wake-up on core 1 (assumption: the 10 ms tick is the only source). */
#define T_PER_LO_US         ((S2_PERIOD_MS * 1000u) - 1000u)
#define T_PER_HI_US         ((S2_PERIOD_MS * 1000u) + 1000u)
#define T_EXEC_MAX_US       ((S2_PERIOD_MS * 1000u) / 2u)   /* half the period */

/* S2-U08..U11 */
#define T_DIST_MM           300u
#define T_SPEED_MM_S        200
#define T_BACK_SPEED_MM_S   300u
#define T_TURN_DDEG         900u            /* 90.0 deg                       */
#define T_TURN_RATE_DEG_S   90u
#define T_DIST_TOL_MM       ((T_DIST_MM * 3u) / 100u)   /* NFR4: +-3 %        */
#define T_BAND_PCT          10              /* "settled": within +-10 %        */
#define T_SETTLE_MAX_MS     500             /* NFR5                            */
#define T_WIN_LO_MS         400             /* cruise window, before the decel */
#define T_WIN_HI_MS         1000
#define T_NO_TURN_MS        400             /* no counts by then: wheels idle  */
#define T_SEQ               0xE0u           /* not used by MissionTask         */

static int32_t iabs(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* MissionTask idle and no brake latched, so MotionTask accepts MISSION
   moves and nothing else commands the wheels. */
static bool wait_idle(uint32_t max_ms)
{
    mission_snap_t ms;
    uint32_t       t;

    for (t = 0u; t < max_ms; t += 50u)
    {
        mission_get_snap(&ms);
        if ((ms.state == (uint8_t)MISSION_ST_IDLE) && !motion_estop_latched())
        {
            return true;
        }
        (void)tk_dly_tsk(50);
    }
    return false;
}

/* Wait until the MOVE/TURN posted after p_start completes; false on timeout. */
static bool wait_done(motion_odom_t const * p_start, uint32_t max_ms, motion_odom_t * p_end)
{
    uint32_t t;

    for (t = 0u; t < max_ms; t += 20u)
    {
        (void)tk_dly_tsk(20);
        motion_get_odom(p_end);
        if ((p_end->done_count != p_start->done_count) &&
            (p_end->state == (uint8_t)MOTION_ST_IDLE_BRAKED))
        {
            return true;
        }
    }
    return false;
}

EXPORT void subsystem2_test(void)
{
    T_RMBF               rmbf;
    motion_odom_t        a;
    motion_odom_t        b;
    motion_test_timing_t tt;

    TEST_INIT();

    TEST_CHECK("S2-U01", tk_ref_mbf(g_mbf_motion, &rmbf) == E_OK, "mbf_motion exists");

    motion_get_odom(&a);
    (void)tk_dly_tsk(50);
    motion_get_odom(&b);
    TEST_CHECK("S2-U03", b.t_ms != a.t_ms, "MotionTask publishes odometry every cycle");
    TEST_CHECK("S2-U04", (b.hw_ok == 1u) && (in_w(PWM_TOP_SLICE4) == MOTION_PWM_TOP) &&
                         ((in_w(PIO0_CTRL_REG) & 0x3u) == 0x3u),
               "PWM 20 kHz configured, PIO0 SM0/SM1 counting");
    TEST_CHECK("S2-U05", (b.state == (uint8_t)MOTION_ST_IDLE_BRAKED) && (b.duty_l == 0) &&
                         (b.duty_r == 0),
               "boots braked (duty 0/0)");

    (void)motion_drive((uint8_t)MOTION_SRC_LINE, 1u, 100, 0);
    (void)tk_dly_tsk(30);
    motion_get_odom(&a);
    TEST_CHECK("S2-U06", (a.rejected > b.rejected) && (a.state == (uint8_t)MOTION_ST_IDLE_BRAKED),
               "DRIVE without LINE authority rejected");

    /* ---- S2-U07: loop period and run time over 1 s (NFR1) ---------- */
    motion_test_timing_restart();
    (void)tk_dly_tsk(1000);
    motion_test_timing(&tt);
    tm_printf((UB *)"[INFO] S2-U07 period min %d us, max %d us, run time max %d us, %d cycles\n",
              (INT)tt.per_min_us, (INT)tt.per_max_us, (INT)tt.exec_max_us, (INT)tt.cycles);
    TEST_CHECK("S2-U07", (tt.cycles >= 90u) && (tt.per_min_us >= T_PER_LO_US) &&
                         (tt.per_max_us <= T_PER_HI_US) && (tt.exec_max_us <= T_EXEC_MAX_US),
               "MotionTask runs every 10 ms +-1 ms, work under 5 ms");

    /* ---- S2-U08..U11: the wheels turn -------------------------------- */
    if (!wait_idle(8000u))
    {
        mission_snap_t ms;

        mission_get_snap(&ms);
        tm_printf((UB *)"[INFO] S2-U08 mission state %d (1 = IDLE; 10 = FAULT, e.g. no IMU)\n", (INT)ms.state);
        TEST_SKIP("S2-U08", "speed step 0 -> 200 mm/s", "MissionTask not IDLE");
        TEST_SKIP("S2-U09", "left and right wheels match", "MissionTask not IDLE");
        TEST_SKIP("S2-U10", "MOVE backward: both encoders count down", "MissionTask not IDLE");
        TEST_SKIP("S2-U11", "TURN left: wheels turn opposite ways", "MissionTask not IDLE");
    }
    else
    {
        motion_odom_t o0;
        motion_odom_t o;
        motion_odom_t o_lo = {0};
        motion_odom_t o_hi = {0};
        int32_t       t_lo = 0;
        int32_t       t_hi = 0;
        uint32_t      t0;
        int32_t       el     = 0;
        int32_t       peak   = 0;
        int32_t       settle = 0;
        int32_t       diff_sum = 0;
        int32_t       n_win  = 0;
        bool          b_lo   = false;
        bool          b_turn = true;
        bool          b_done = false;
        int32_t const band   = (T_SPEED_MM_S * T_BAND_PCT) / 100;

        motion_get_odom(&o0);
        t0 = (uint32_t)app_time_ms();
        (void)motion_move_forward((uint8_t)MOTION_SRC_MISSION, T_SEQ, (uint16_t)T_DIST_MM,
                                  (uint16_t)T_SPEED_MM_S);
        for (;;)
        {
            int32_t v;

            (void)tk_dly_tsk(10);
            motion_get_odom(&o);
            el = (int32_t)((uint32_t)app_time_ms() - t0);
            v  = ((int32_t)o.v_l_mm_s + (int32_t)o.v_r_mm_s) / 2;
            if (el <= T_WIN_HI_MS)
            {
                if (v > peak)
                {
                    peak = v;
                }
                if (iabs(v - T_SPEED_MM_S) > band)
                {
                    settle = el;            /* last time outside the band */
                }
            }
            if ((el >= T_WIN_LO_MS) && (el <= T_WIN_HI_MS))
            {
                if (!b_lo)
                {
                    o_lo = o;
                    t_lo = el;
                    b_lo = true;
                }
                o_hi = o;
                t_hi = el;
                diff_sum += iabs((int32_t)o.v_l_mm_s - (int32_t)o.v_r_mm_s);
                n_win++;
            }
            if ((el >= T_NO_TURN_MS) && (o.enc_l == o0.enc_l) && (o.enc_r == o0.enc_r))
            {
                b_turn = false;
                break;
            }
            if ((o.done_count != o0.done_count) && (o.state == (uint8_t)MOTION_ST_IDLE_BRAKED))
            {
                b_done = true;
                break;
            }
            if (el > 5000)
            {
                break;
            }
        }

        if (!b_turn)
        {
            (void)motion_stop((uint8_t)MOTION_SRC_MISSION, T_SEQ);
            TEST_SKIP("S2-U08", "speed step 0 -> 200 mm/s", "no encoder counts: motor battery off, or emulator");
            TEST_SKIP("S2-U09", "left and right wheels match", "wheels did not turn");
            TEST_SKIP("S2-U10", "MOVE backward: both encoders count down", "wheels did not turn");
            TEST_SKIP("S2-U11", "TURN left: wheels turn opposite ways", "wheels did not turn");
            /* A stall fault may have latched meanwhile: clear it for S3..S5. */
            (void)mission_post((uint8_t)MISSION_EV_REMOTE_RESET, (uint8_t)APP_SRC_MOTION, 0u, 0);
            (void)wait_idle(8000u);
        }
        else
        {
            int32_t const dist = o.dist_mm - o0.dist_mm;
            int32_t const over = ((peak - T_SPEED_MM_S) * 100) / T_SPEED_MM_S;

            tm_printf((UB *)"[INFO] S2-U08 settle %d ms (within +-%d mm/s), peak %d mm/s, overshoot %d %%\n",
                      (INT)settle, (INT)band, (INT)peak, (INT)((over > 0) ? over : 0));
            tm_printf((UB *)"[INFO] S2-U08 distance %d mm (asked %d), encoders L %d R %d counts\n",
                      (INT)dist, (INT)T_DIST_MM, (INT)(o.enc_l - o0.enc_l), (INT)(o.enc_r - o0.enc_r));
            TEST_CHECK("S2-U08", b_done && (settle <= T_SETTLE_MAX_MS) &&
                                 (iabs(dist - (int32_t)T_DIST_MM) <= (int32_t)T_DIST_TOL_MM) &&
                                 (o.enc_l > o0.enc_l) && (o.enc_r > o0.enc_r),
                       "speed step 0 -> 200 mm/s settles in 500 ms; MOVE 300 mm ends within 3 %");

            if (b_lo && (n_win > 0) && (t_hi > t_lo))
            {
                int32_t const diff = diff_sum / n_win;
                int32_t const dt   = t_hi - t_lo;

                tm_printf((UB *)"[INFO] S2-U09 mean |vL - vR| %d mm/s over %d samples; "
                          "encoders L %d R %d counts/s at 200 mm/s\n",
                          (INT)diff, (INT)n_win,
                          (INT)(((o_hi.enc_l - o_lo.enc_l) * 1000) / dt),
                          (INT)(((o_hi.enc_r - o_lo.enc_r) * 1000) / dt));
                TEST_CHECK("S2-U09", diff <= band, "left and right wheels match within 10 % at 200 mm/s");
            }
            else
            {
                TEST_CHECK("S2-U09", false, "left and right wheels match (no cruise samples)");
            }

            /* Reverse */
            motion_get_odom(&o0);
            (void)motion_move_backward((uint8_t)MOTION_SRC_MISSION, (uint8_t)(T_SEQ + 1u),
                                       (uint16_t)T_DIST_MM, (uint16_t)T_BACK_SPEED_MM_S);
            b_done = wait_done(&o0, 5000u, &o);
            tm_printf((UB *)"[INFO] S2-U10 distance %d mm, encoders L %d R %d counts\n",
                      (INT)(o.dist_mm - o0.dist_mm), (INT)(o.enc_l - o0.enc_l), (INT)(o.enc_r - o0.enc_r));
            TEST_CHECK("S2-U10", b_done && (o.enc_l < o0.enc_l) && (o.enc_r < o0.enc_r) &&
                                 (iabs((o.dist_mm - o0.dist_mm) + (int32_t)T_DIST_MM) <= (int32_t)T_DIST_TOL_MM),
                       "MOVE backward 300 mm: both encoders count down, ends within 3 %");

            /* Spin left */
            motion_get_odom(&o0);
            (void)motion_turn_left((uint8_t)MOTION_SRC_MISSION, (uint8_t)(T_SEQ + 2u),
                                   (uint16_t)T_TURN_DDEG, (uint16_t)T_TURN_RATE_DEG_S);
            b_done = wait_done(&o0, 6000u, &o);
            tm_printf((UB *)"[INFO] S2-U11 encoders L %d R %d counts\n",
                      (INT)(o.enc_l - o0.enc_l), (INT)(o.enc_r - o0.enc_r));
            TEST_CHECK("S2-U11", b_done && (o.enc_l < o0.enc_l) && (o.enc_r > o0.enc_r),
                       "TURN left 90 deg: left wheel back, right wheel forward");
        }
    }

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
