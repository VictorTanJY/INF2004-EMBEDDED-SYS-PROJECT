/*
 *  motion_task.c
 *  Subsystem 2 -- MotionTask (priority 6, core 1), woken by motion_cyc
 *  every 10 ms (Week 6 Sections 3 and 6.1).  Owner: Buddy 2 (Jedrek).
 *
 *  Every cycle:
 *    1. heartbeat; read both PIO encoder counts;
 *    2. drain mbf_motion (TMO_POL) into the controller (authority rules);
 *    3. one controller step (PID, profiles, STOPPING, stall);
 *    4. write the PWM duty -- the latch is checked immediately before the
 *       write, inside DI/EI, so subsystem2_motor_estop() cannot be undone;
 *    5. publish the odometry snapshot; report events:
 *       MOTION_DONE / MOTION_FAILED -> mbf_mission, STALL -> flg_safety.
 */

#include "motion_task.h"
#include "subsystem2.h"
#include "motion_core.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

/* still runs (and heartbeats) if motion_cyc stops */
#define MOTION_WAKE_TIMEOUT_MS 30

static motion_core_t g_core;

#if APP_TEST

/* make TEST=1 only: loop timing for S2-U07.  Written by MotionTask
   (core 1) only; the test task reads each 32-bit word, which is one
   indivisible load on the M0+.  The test asks for a restart through
   g_tt_restart (a byte it sets and MotionTask clears), so the counters
   have a single writer and no read-modify-write is shared. */
static uint32_t volatile g_tt_per_min_us;
static uint32_t volatile g_tt_per_max_us;
static uint32_t volatile g_tt_exec_max_us;
static uint32_t volatile g_tt_cycles;
static uint8_t volatile g_tt_restart = 1u;
static uint32_t g_tt_prev_us;

#endif

#if APP_TEST
static void tt_wake(uint32_t now_us);
static void tt_done(uint32_t wake_us);
#endif
static void timing_pin_init(void);
static void timing_pin_toggle(void);
static void write_pwm(int16_t duty_l, int16_t duty_r);
static void report_events(void);

#if APP_TEST

EXPORT void motion_task_timing_restart (void)
{
    g_tt_restart = 1u;
}

EXPORT void motion_task_test_timing (motion_task_timing_t * p_out)
{
    p_out->per_min_us  = g_tt_per_min_us;
    p_out->per_max_us  = g_tt_per_max_us;
    p_out->exec_max_us = g_tt_exec_max_us;
    p_out->cycles      = g_tt_cycles;
}

#endif

EXPORT void motion_task_thread (INT stacd, void * p_exinf)
{
    motion_types_odom_t odom = {0};
    motion_types_cmd_t  cmd;
    uint32_t            cmd_word; /* mbf_motion: one packed word (CR1) */
    bool                b_hw_ok;
    UINT                intsts;

    (void)stacd;
    (void)p_exinf;

    timing_pin_init();
    b_hw_ok = (E_OK == drv_motor_init());
    b_hw_ok = (E_OK == drv_encoder_init()) && b_hw_ok;

    /* Connect PWM (duty 0) unless SafetyTask already latched the brake. */
    DI(intsts);
    if (!subsystem2_estop_latched())
    {
        drv_motor_connect();
    }
    EI(intsts);

    motion_core_init(&g_core, (uint32_t)app_time_ms_hw());

    for (;;)
    {
        uint32_t now_ms;
        int32_t  enc_l;
        int32_t  enc_r;
#if APP_TEST
        uint32_t wake_us;
#endif

        (void)tk_slp_tsk(MOTION_WAKE_TIMEOUT_MS);
#if APP_TEST
        wake_us = (uint32_t)app_time_us();
        tt_wake(wake_us);
#endif
        timing_pin_toggle();
        subsystem4_heartbeat(SAFETY_HB_MOTION);

        now_ms = (uint32_t)app_time_ms_hw();
        enc_l  = drv_encoder_read(DRV_ENC_LEFT);
        enc_r  = drv_encoder_read(DRV_ENC_RIGHT);

        while ((INT)sizeof(cmd_word)
               == tk_rcv_mbf(g_mbf_motion, &cmd_word, TMO_POL))
        {
            motion_core_cmd_unpack(cmd_word, &cmd);
            motion_core_result_t const res =
                motion_core_command(&g_core, &cmd, now_ms);

            if (MOTION_RES_CLEAR_ESTOP == res)
            {
                if (subsystem2_estop_reconnect())
                {
                    motion_core_estop_cleared(&g_core);
                }
            }
        }

        motion_core_step(&g_core, enc_l, enc_r, subsystem2_estop_latched(),
                         now_ms);
        write_pwm(g_core.duty_l, g_core.duty_r);

        motion_core_snapshot(&g_core, &odom);
        odom.t_ms  = (uint32_t)app_time_ms();
        odom.hw_ok = b_hw_ok ? 1u : 0u;
        subsystem2_put_odom(&odom);

        report_events();
#if APP_TEST
        tt_done(wake_us);
#endif
    }
}

#if APP_TEST


static void tt_wake (uint32_t now_us)
{
    if (g_tt_restart != 0u)
    {
        g_tt_per_min_us  = 0xFFFFFFFFu;
        g_tt_per_max_us  = 0u;
        g_tt_exec_max_us = 0u;
        g_tt_cycles      = 0u;
        g_tt_restart     = 0u;
    }
    else
    {
        uint32_t const per = now_us - g_tt_prev_us;

        if (per < g_tt_per_min_us)
        {
            g_tt_per_min_us = per;
        }
        if (per > g_tt_per_max_us)
        {
            g_tt_per_max_us = per;
        }
        g_tt_cycles = g_tt_cycles + 1u;
    }
    g_tt_prev_us = now_us;
}

static void tt_done (uint32_t wake_us)
{
    uint32_t const exec = (uint32_t)app_time_us() - wake_us;

    if (exec > g_tt_exec_max_us)
    {
        g_tt_exec_max_us = exec;
    }
}

#endif

static void timing_pin_init (void)
{
#if APP_TIMING_PINS
    out_w(APP_REGS_GPIO_OUT_CLR, 1uL << S2_PIN_TIMING);
    out_w(APP_REGS_GPIO_OE_SET, 1uL << S2_PIN_TIMING);
    out_w(app_regs_gpio_ctrl(S2_PIN_TIMING), APP_REGS_FUNCSEL_SIO);
#endif
}

static void timing_pin_toggle (void)
{
#if APP_TIMING_PINS
    out_w(APP_REGS_GPIO_OUT_XOR, 1uL << S2_PIN_TIMING);
#endif
}

/* Step 4: check the latch and write, as one indivisible step on core 1. */
static void write_pwm (int16_t duty_l, int16_t duty_r)
{
    UINT intsts;

    DI(intsts);
    if (!subsystem2_estop_latched())
    {
        drv_motor_write(duty_l, duty_r);
    }
    EI(intsts);
}

static void report_events (void)
{
    motion_core_event_t event;

    while (motion_core_pop_event(&g_core, &event))
    {
        switch (event.code)
        {
            case MOTION_EV_DONE:
                (void)subsystem5_mission_post(
                    MISSION_EV_MOTION_DONE, (uint8_t)APP_SRC_MOTION,
                    (uint16_t)event.seq, (int32_t)event.src);
            break;

            case MOTION_EV_FAILED:
                (void)subsystem5_mission_post(
                    MISSION_EV_MOTION_FAILED, (uint8_t)APP_SRC_MOTION,
                    (uint16_t)event.seq, (int32_t)event.src);
            break;

            case MOTION_EV_STALL:
                (void)tk_set_flg(g_flg_safety, SAFETY_FLT_STALL);
            break;

            case MOTION_EV_ENC_FAULT:
                (void)tk_set_flg(g_flg_safety, SAFETY_FLT_ENC_FAULT);
                (void)subsystem1_telem_post(COMMS_KIND_FAULT,
                                            (uint8_t)APP_SRC_MOTION, 0u,
                                            (int32_t)SAFETY_FLT_ENC_FAULT);
            break;

            case MOTION_EV_ENC_OK:
                (void)tk_clr_flg(g_flg_safety, ~(UINT)SAFETY_FLT_ENC_FAULT);
            break;

            default:
            break;
        }
    }
}

/*** end of file ***/
