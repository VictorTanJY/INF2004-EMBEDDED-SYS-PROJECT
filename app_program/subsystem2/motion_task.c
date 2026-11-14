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
 *       write, inside DI/EI, so motor_estop() cannot be undone;
 *    5. publish the odometry snapshot; report events:
 *       MOTION_DONE / MOTION_FAILED -> mbf_mission, STALL -> flg_safety.
 */

#include "subsystem2.h"
#include "motion_core.h"
#include "motion_rec.h"
#include "../app_types.h"
#include "drv_motor.h"
#include "drv_encoder.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#define MOTION_WAKE_TIMEOUT_MS  30      /* still runs (and heartbeats) if motion_cyc stops */

static motion_core_t g_core;

#if APP_BENCH
/* 15 Nov: speed-step recorder for the PID tuning report (motion_rec.h).
   Each MOVE records its first 1.5 s at 10 ms; the samples are then posted
   as log records, at most REC_POSTS_PER_STEP per 10 ms step so mbf_telem
   (drained 8 per 20 ms by CommsTask) does not fill. */
#define REC_POSTS_PER_STEP      2u
static motion_rec_t g_rec;

static void rec_step(void)
{
    motion_rec_sample_t s;
    int32_t             sp_l;
    int32_t             sp_r;
    uint16_t            code;
    int32_t             data;
    uint16_t            k;

    if (g_rec.state == (uint8_t)MOTION_REC_RECORDING)
    {
        motion_core_wheel_sp(&g_core, &sp_l, &sp_r);
        s.sp_l   = (int16_t)sp_l;
        s.sp_r   = (int16_t)sp_r;
        s.v_l    = (int16_t)app_clamp_i32(g_core.v_l, -32768, 32767);
        s.v_r    = (int16_t)app_clamp_i32(g_core.v_r, -32768, 32767);
        s.duty_l = g_core.duty_l;
        s.duty_r = g_core.duty_r;
        motion_rec_add(&g_rec, &s);
    }
    for (k = 0u; (k < REC_POSTS_PER_STEP) && motion_rec_peek(&g_rec, &code, &data); k++)
    {
        if (!comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_MOTION, code, data))
        {
            break;                      /* full: the same record next step */
        }
        motion_rec_sent(&g_rec);
    }
}
#endif

static void timing_pin_init(void)
{
#if APP_TIMING_PINS
    out_w(GPIO_OUT_CLR, 1uL << S2_PIN_TIMING);
    out_w(GPIO_OE_SET, 1uL << S2_PIN_TIMING);
    out_w(GPIO_CTRL(S2_PIN_TIMING), GPIO_CTRL_FUNCSEL_SIO);
#endif
}

static void timing_pin_toggle(void)
{
#if APP_TIMING_PINS
    out_w(GPIO_OUT_XOR, 1uL << S2_PIN_TIMING);
#endif
}

/* Step 4: check the latch and write, as one indivisible step on core 1. */
static void write_pwm(int16_t duty_l, int16_t duty_r)
{
    UINT intsts;

    DI(intsts);
    if (!motion_estop_latched())
    {
        drv_motor_write(duty_l, duty_r);
    }
    EI(intsts);
}

static void report_events(void)
{
    motion_event_t ev;

    while (motion_core_pop_event(&g_core, &ev))
    {
        switch (ev.code)
        {
            case MOTION_EV_DONE:
                (void)mission_post((uint8_t)MISSION_EV_MOTION_DONE, (uint8_t)APP_SRC_MOTION,
                                   (uint16_t)ev.seq, (int32_t)ev.src);
                break;

            case MOTION_EV_FAILED:
                (void)mission_post((uint8_t)MISSION_EV_MOTION_FAILED, (uint8_t)APP_SRC_MOTION,
                                   (uint16_t)ev.seq, (int32_t)ev.src);
                break;

            case MOTION_EV_STALL:
                (void)tk_set_flg(g_flg_safety, SAFETY_FLT_STALL);
                break;

            case MOTION_EV_ENC_FAULT:
                (void)tk_set_flg(g_flg_safety, SAFETY_FLT_ENC_FAULT);
                (void)comms_telem_post((uint8_t)COMMS_KIND_FAULT, (uint8_t)APP_SRC_MOTION,
                                       0u, (int32_t)SAFETY_FLT_ENC_FAULT);
                break;

            case MOTION_EV_ENC_OK:
                (void)tk_clr_flg(g_flg_safety, ~(UINT)SAFETY_FLT_ENC_FAULT);
                break;

            default:
                break;
        }
    }
}

EXPORT void motion_task(INT stacd, void * p_exinf)
{
    motion_odom_t odom = {0};
    motion_cmd_t  cmd;
    bool          b_hw_ok;
    UINT          intsts;

    (void)stacd;
    (void)p_exinf;

    timing_pin_init();
    b_hw_ok = (drv_motor_init() == E_OK);
    b_hw_ok = (drv_encoder_init() == E_OK) && b_hw_ok;

    /* Connect PWM (duty 0) unless SafetyTask already latched the brake. */
    DI(intsts);
    if (!motion_estop_latched())
    {
        drv_motor_connect();
    }
    EI(intsts);

    motion_core_init(&g_core, (uint32_t)app_time_ms_hw());
#if APP_BENCH
    motion_rec_init(&g_rec);
#endif

    for (;;)
    {
        uint32_t now_ms;
        int32_t  enc_l;
        int32_t  enc_r;

        (void)tk_slp_tsk(MOTION_WAKE_TIMEOUT_MS);
        timing_pin_toggle();
        safety_heartbeat(SAFETY_HB_MOTION);

        now_ms = (uint32_t)app_time_ms_hw();
        enc_l  = drv_encoder_read(DRV_ENC_LEFT);
        enc_r  = drv_encoder_read(DRV_ENC_RIGHT);

        while (tk_rcv_mbf(g_mbf_motion, &cmd, TMO_POL) == (INT)sizeof(cmd))
        {
            motion_result_t const res = motion_core_command(&g_core, &cmd, now_ms);

            if (res == MOTION_RES_CLEAR_ESTOP)
            {
                if (motion_estop_clear_and_reconnect())
                {
                    motion_core_estop_cleared(&g_core);
                }
            }
#if APP_BENCH
            else if ((res == MOTION_RES_ACCEPTED) && (cmd.cmd == (uint8_t)MOTION_CMD_MOVE))
            {
                motion_rec_start(&g_rec);
            }
#endif
        }

        motion_core_step(&g_core, enc_l, enc_r, motion_estop_latched(), now_ms);
        write_pwm(g_core.duty_l, g_core.duty_r);
#if APP_BENCH
        rec_step();
#endif

        motion_core_snapshot(&g_core, &odom);
        odom.t_ms  = (uint32_t)app_time_ms();
        odom.hw_ok = b_hw_ok ? 1u : 0u;
        motion_put_odom(&odom);

        report_events();
    }
}
