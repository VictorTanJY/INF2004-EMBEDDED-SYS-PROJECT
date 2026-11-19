/** @file subsystem2_test.c
 *
 *  @brief On-target tests for subsystem 2.  Built only with: make TEST=1
 *  Record results in testing/subsystem2/TEST_LOG.md.
 *  The controller maths is tested on a PC (testing/subsystem2/host).
 *
 *  S2-U08..U11 turn the wheels: put the robot on a stand with the wheels
 *  off the floor (on the floor it drives 30 cm forward, 30 cm back and
 *  turns 90 deg left).  The motor battery switch must be on.
 */

#include "subsystem2_test.h"
#include "motion_task.h"
#include "subsystem2.h"
#include "motion_cfg.h"

/* subsystem5_get_mission_snap: wait for IDLE */
#include "../subsystem5/subsystem5.h"
#include "../test_report.h"

#if APP_TEST

#define PWM_TOP_SLICE4 \
    (APP_REGS_PWM_BASE + (0x14u * 4u) + 0x10u) /* PWM CH4_TOP */
#define PIO0_CTRL_REG 0x50200000u

/* S2-U07: the cycle is motion_cyc (S2_PERIOD_MS).  One kernel tick of
   jitter is allowed either way between the tick interrupt on core 0 and
   the wake-up on core 1 (assumption: the 10 ms tick is the only source). */
#define T_PER_LO_US   ((S2_PERIOD_MS * 1000u) - 1000u)
#define T_PER_HI_US   ((S2_PERIOD_MS * 1000u) + 1000u)
#define T_EXEC_MAX_US ((S2_PERIOD_MS * 1000u) / 2u) /* half the period */

/* S2-U08..U11 */
#define T_DIST_MM         300u
#define T_SPEED_MM_S      200
#define T_BACK_SPEED_MM_S 300u
#define T_TURN_DDEG       900u /* 90.0 deg                       */
#define T_TURN_RATE_DEG_S 90u
#define T_DIST_TOL_MM     ((T_DIST_MM * 3u) / 100u) /* NFR4: +-3 %        */
#define T_BAND_PCT        10 /* "settled": within +-10 %        */
#define T_BAND_MM_S       ((T_SPEED_MM_S * T_BAND_PCT) / 100)
#define T_SETTLE_MAX_MS   500 /* NFR5                            */
#define T_WIN_LO_MS       400 /* cruise window, before the decel */
#define T_WIN_HI_MS       1000
#define T_NO_TURN_MS      400 /* no counts by then: wheels idle  */
#define T_SEQ 20u /* 5-bit seq, not one MissionTask is waiting for */

/** S2-U08 / U09: what the speed step measured. */
typedef struct
{
    motion_types_odom_t odom_start;
    motion_types_odom_t odom; /**< last sample */
    motion_types_odom_t o_lo; /**< first and last sample of the cruise window */
    motion_types_odom_t o_hi;
    int32_t             t_lo;
    int32_t             t_hi;
    int32_t             peak;
    int32_t             settle;
    int32_t             diff_sum;
    int32_t             n_win;
    bool                b_lo;
    bool                b_turn;
    bool                b_done;
} step_rec_t;

#endif

#if APP_TEST
static int32_t iabs(int32_t value);
static bool    wait_idle(uint32_t max_ms);
static bool    wait_done(motion_types_odom_t const * p_start, uint32_t max_ms,
                         motion_types_odom_t * p_end);
static void    test_wheels(test_report_t * p_report);
static void    step_measure(step_rec_t * p_rec);
static void    step_sample(step_rec_t * p_rec, int32_t elapsed);
static void    test_step_result(test_report_t *    p_report,
                                step_rec_t const * p_rec);
static void    test_reverse(test_report_t * p_report);
static void    test_spin(test_report_t * p_report);
#endif

#if APP_TEST

EXPORT void subsystem2_test_run (void)
{
    T_RMBF               rmbf;
    motion_types_odom_t  snap_a;
    motion_types_odom_t  snap_b;
    motion_task_timing_t timing;

    test_report_t report = {0, 0, 0};

    TEST_CHECK(&report, "S2-U01",
               E_OK == tk_ref_mbf(subsystem2_mbf_motion(), &rmbf),
               "mbf_motion exists");

    subsystem2_get_odom(&snap_a);
    (void)tk_dly_tsk(50);
    subsystem2_get_odom(&snap_b);
    TEST_CHECK(&report, "S2-U03", snap_b.t_ms != snap_a.t_ms,
               "MotionTask publishes odometry every cycle");
    TEST_CHECK(&report, "S2-U04",
               (1u == snap_b.hw_ok) && (MOTION_PWM_TOP == in_w(PWM_TOP_SLICE4))
                   && (0x3u == (in_w(PIO0_CTRL_REG) & 0x3u)),
               "PWM 20 kHz configured, PIO0 SM0/SM1 counting");
    TEST_CHECK(&report, "S2-U05",
               (MOTION_ST_IDLE_BRAKED == snap_b.state) && (0 == snap_b.duty_l)
                   && (0 == snap_b.duty_r),
               "boots braked (duty 0/0)");

    (void)motion_api_drive(MOTION_SRC_LINE, 1u, 100, 0);
    (void)tk_dly_tsk(30);
    subsystem2_get_odom(&snap_a);
    TEST_CHECK(&report, "S2-U06",
               (snap_a.rejected > snap_b.rejected)
                   && (MOTION_ST_IDLE_BRAKED == snap_a.state),
               "DRIVE without LINE authority rejected");

    /* ---- S2-U07: loop period and run time over 1 s (NFR1) ---------- */
    motion_task_timing_restart();
    (void)tk_dly_tsk(1000);
    motion_task_test_timing(&timing);
    tm_printf((UB *)"[INFO] S2-U07 period min %d us, max %d us, run time max "
                    "%d us, %d cycles\n",
              (INT)timing.per_min_us, (INT)timing.per_max_us,
              (INT)timing.exec_max_us, (INT)timing.cycles);
    TEST_CHECK(&report, "S2-U07",
               (timing.cycles >= 90u) && (timing.per_min_us >= T_PER_LO_US)
                   && (timing.per_max_us <= T_PER_HI_US)
                   && (timing.exec_max_us <= T_EXEC_MAX_US),
               "MotionTask runs every 10 ms +-1 ms, work under 5 ms");

    /* ---- S2-U08..U11: the wheels turn -------------------------------- */
    if (!wait_idle(8000u))
    {
        mission_types_snap_t mission;

        subsystem5_get_mission_snap(&mission);
        tm_printf((UB *)"[INFO] S2-U08 mission state %d (1 = IDLE; 10 = FAULT, "
                        "e.g. no IMU)\n",
                  (INT)mission.state);
        test_report_skip(&report, "S2-U08", "speed step 0 -> 200 mm/s",
                         "MissionTask not IDLE");
        test_report_skip(&report, "S2-U09", "left and right wheels match",
                         "MissionTask not IDLE");
        test_report_skip(&report, "S2-U10",
                         "MOVE backward: both encoders count down",
                         "MissionTask not IDLE");
        test_report_skip(&report, "S2-U11",
                         "TURN left: wheels turn opposite ways",
                         "MissionTask not IDLE");
    }
    else
    {
        test_wheels(&report);
    }

    test_report_summary(&report, "subsystem2");
}

#endif

#if APP_TEST

/** S2-U08..U11 with MissionTask idle: speed step, wheel match, reverse,
   spin. */
static void test_wheels (test_report_t * p_report)
{
    step_rec_t rec = {0};

    step_measure(&rec);
    if (!rec.b_turn)
    {
        (void)motion_api_stop(MOTION_SRC_MISSION, T_SEQ);
        test_report_skip(p_report, "S2-U08", "speed step 0 -> 200 mm/s",
                         "no encoder counts: motor battery off, or emulator");
        test_report_skip(p_report, "S2-U09", "left and right wheels match",
                         "wheels did not turn");
        test_report_skip(p_report, "S2-U10",
                         "MOVE backward: both encoders count down",
                         "wheels did not turn");
        test_report_skip(p_report, "S2-U11",
                         "TURN left: wheels turn opposite ways",
                         "wheels did not turn");
        /* A stall fault may have latched meanwhile: clear it for S3..S5. */
        (void)subsystem5_mission_post(MISSION_EV_REMOTE_RESET,
                                      (uint8_t)APP_SRC_MOTION, 0u, 0);
        (void)wait_idle(8000u);
    }
    else
    {
        test_step_result(p_report, &rec);
        test_reverse(p_report);
        test_spin(p_report);
    }
}

/** S2-U08: MOVE 300 mm at 200 mm/s, sampled every 10 ms until it ends, the
   wheels stay still, or 5 s pass. */
static void step_measure (step_rec_t * p_rec)
{
    uint32_t t_start;

    subsystem2_get_odom(&p_rec->odom_start);
    t_start       = (uint32_t)app_time_ms();
    p_rec->b_turn = true;
    (void)motion_api_move_forward(MOTION_SRC_MISSION, T_SEQ,
                                  (uint16_t)T_DIST_MM, (uint16_t)T_SPEED_MM_S);
    for (;;)
    {
        int32_t elapsed;

        (void)tk_dly_tsk(10);
        subsystem2_get_odom(&p_rec->odom);
        elapsed = (int32_t)((uint32_t)app_time_ms() - t_start);
        step_sample(p_rec, elapsed);
        if ((elapsed >= T_NO_TURN_MS)
            && (p_rec->odom.enc_l == p_rec->odom_start.enc_l)
            && (p_rec->odom.enc_r == p_rec->odom_start.enc_r))
        {
            p_rec->b_turn = false;
            break;
        }
        if ((p_rec->odom.done_count != p_rec->odom_start.done_count)
            && (MOTION_ST_IDLE_BRAKED == p_rec->odom.state))
        {
            p_rec->b_done = true;
            break;
        }
        if (elapsed > 5000)
        {
            break;
        }
    }
}

/** S2-U08 / U09: peak, settling time and the cruise window of one sample. */
static void step_sample (step_rec_t * p_rec, int32_t elapsed)
{
    int32_t const value =
        ((int32_t)p_rec->odom.v_l_mm_s + (int32_t)p_rec->odom.v_r_mm_s) / 2;

    if (elapsed <= T_WIN_HI_MS)
    {
        if (value > p_rec->peak)
        {
            p_rec->peak = value;
        }
        if (iabs(value - T_SPEED_MM_S) > T_BAND_MM_S)
        {
            p_rec->settle = elapsed; /* last time outside the band */
        }
    }
    if ((elapsed >= T_WIN_LO_MS) && (elapsed <= T_WIN_HI_MS))
    {
        if (!p_rec->b_lo)
        {
            p_rec->o_lo = p_rec->odom;
            p_rec->t_lo = elapsed;
            p_rec->b_lo = true;
        }
        p_rec->o_hi = p_rec->odom;
        p_rec->t_hi = elapsed;
        p_rec->diff_sum +=
            iabs((int32_t)p_rec->odom.v_l_mm_s - (int32_t)p_rec->odom.v_r_mm_s);
        p_rec->n_win++;
    }
}

/** S2-U08 / U09 results. */
static void test_step_result (test_report_t *    p_report,
                              step_rec_t const * p_rec)
{
    motion_types_odom_t const * const p_odom  = &p_rec->odom;
    motion_types_odom_t const * const p_start = &p_rec->odom_start;
    int32_t const                     dist = p_odom->dist_mm - p_start->dist_mm;
    int32_t const over = ((p_rec->peak - T_SPEED_MM_S) * 100) / T_SPEED_MM_S;

    tm_printf((UB *)"[INFO] S2-U08 settle %d ms (within +-%d mm/s), "
                    "peak %d mm/s, overshoot %d %%\n",
              (INT)p_rec->settle, (INT)T_BAND_MM_S, (INT)p_rec->peak,
              (INT)((over > 0) ? over : 0));
    tm_printf((UB *)"[INFO] S2-U08 distance %d mm (asked %d), encoders "
                    "L %d R %d counts\n",
              (INT)dist, (INT)T_DIST_MM, (INT)(p_odom->enc_l - p_start->enc_l),
              (INT)(p_odom->enc_r - p_start->enc_r));
    TEST_CHECK(
        p_report, "S2-U08",
        p_rec->b_done && (p_rec->settle <= T_SETTLE_MAX_MS)
            && (iabs(dist - (int32_t)T_DIST_MM) <= (int32_t)T_DIST_TOL_MM)
            && (p_odom->enc_l > p_start->enc_l)
            && (p_odom->enc_r > p_start->enc_r),
        "speed step 0 -> 200 mm/s settles in 500 ms; MOVE 300 "
        "mm ends within 3 %");

    if (p_rec->b_lo && (p_rec->n_win > 0) && (p_rec->t_hi > p_rec->t_lo))
    {
        int32_t const diff     = p_rec->diff_sum / p_rec->n_win;
        int32_t const delta_ms = p_rec->t_hi - p_rec->t_lo;

        tm_printf(
            (UB *)"[INFO] S2-U09 mean |vL - vR| %d mm/s over %d "
                  "samples; "
                  "encoders L %d R %d counts/s at 200 mm/s\n",
            (INT)diff, (INT)p_rec->n_win,
            (INT)(((p_rec->o_hi.enc_l - p_rec->o_lo.enc_l) * 1000) / delta_ms),
            (INT)(((p_rec->o_hi.enc_r - p_rec->o_lo.enc_r) * 1000) / delta_ms));
        TEST_CHECK(p_report, "S2-U09", diff <= T_BAND_MM_S,
                   "left and right wheels match within 10 % at 200 mm/s");
    }
    else
    {
        TEST_CHECK(p_report, "S2-U09", false,
                   "left and right wheels match (no cruise samples)");
    }
}

/** S2-U10: MOVE backward 300 mm. */
static void test_reverse (test_report_t * p_report)
{
    motion_types_odom_t odom_start;
    motion_types_odom_t odom;
    bool                b_done;

    subsystem2_get_odom(&odom_start);
    (void)motion_api_move_backward(MOTION_SRC_MISSION, (uint8_t)(T_SEQ + 1u),
                                   (uint16_t)T_DIST_MM,
                                   (uint16_t)T_BACK_SPEED_MM_S);
    b_done = wait_done(&odom_start, 5000u, &odom);
    tm_printf((UB *)"[INFO] S2-U10 distance %d mm, encoders L %d R %d "
                    "counts\n",
              (INT)(odom.dist_mm - odom_start.dist_mm),
              (INT)(odom.enc_l - odom_start.enc_l),
              (INT)(odom.enc_r - odom_start.enc_r));
    TEST_CHECK(
        p_report, "S2-U10",
        b_done && (odom.enc_l < odom_start.enc_l)
            && (odom.enc_r < odom_start.enc_r)
            && (iabs((odom.dist_mm - odom_start.dist_mm) + (int32_t)T_DIST_MM)
                <= (int32_t)T_DIST_TOL_MM),
        "MOVE backward 300 mm: both encoders count down, ends "
        "within 3 %");
}

/** S2-U11: TURN left 90 deg. */
static void test_spin (test_report_t * p_report)
{
    motion_types_odom_t odom_start;
    motion_types_odom_t odom;
    bool                b_done;

    subsystem2_get_odom(&odom_start);
    (void)motion_api_turn_left(MOTION_SRC_MISSION, (uint8_t)(T_SEQ + 2u),
                               (uint16_t)T_TURN_DDEG,
                               (uint16_t)T_TURN_RATE_DEG_S);
    b_done = wait_done(&odom_start, 6000u, &odom);
    tm_printf((UB *)"[INFO] S2-U11 encoders L %d R %d counts\n",
              (INT)(odom.enc_l - odom_start.enc_l),
              (INT)(odom.enc_r - odom_start.enc_r));
    TEST_CHECK(p_report, "S2-U11",
               b_done && (odom.enc_l < odom_start.enc_l)
                   && (odom.enc_r > odom_start.enc_r),
               "TURN left 90 deg: left wheel back, right wheel forward");
}

/** Absolute value (value must not be INT32_MIN). */
static int32_t iabs (int32_t value)
{
    return (value < 0) ? -value : value;
}

/** MissionTask idle and no brake latched, so MotionTask accepts MISSION
   moves and nothing else commands the wheels. */
static bool wait_idle (uint32_t max_ms)
{
    mission_types_snap_t mission;
    uint32_t             elapsed;

    for (elapsed = 0u; elapsed < max_ms; elapsed += 50u)
    {
        subsystem5_get_mission_snap(&mission);
        if ((MISSION_ST_IDLE == mission.state) && !subsystem2_estop_latched())
        {
            return true;
        }
        (void)tk_dly_tsk(50);
    }
    return false;
}

/** Wait until the MOVE/TURN posted after p_start completes; false on timeout.
 */
static bool wait_done (motion_types_odom_t const * p_start, uint32_t max_ms,
                       motion_types_odom_t * p_end)
{
    uint32_t elapsed;

    for (elapsed = 0u; elapsed < max_ms; elapsed += 20u)
    {
        (void)tk_dly_tsk(20);
        subsystem2_get_odom(p_end);
        if ((p_end->done_count != p_start->done_count)
            && (MOTION_ST_IDLE_BRAKED == p_end->state))
        {
            return true;
        }
    }
    return false;
}

#endif

/*** end of file ***/
