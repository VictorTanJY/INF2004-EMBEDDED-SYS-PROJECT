/*
 *  line_task.c
 *  Subsystem 3 -- LineBarcodeTask (priority 7, core 1), woken by
 *  adc_dma_isr every 10 ms (Week 6 Sections 3 and 6.2).  Owner: Buddy 3.
 *
 *  Every block (10 samples per channel):
 *    1. heartbeat; take the block; count missed blocks (NFR2);
 *    2. CALIBRATING: record min/max during the spin, finish on request;
 *    3. region B: feed the 10 barcode samples, with the distance the
 *       barcode sensor moved per sample, to the capture state machine;
 *       a valid read posts BARCODE(character) -- MissionTask decides from
 *       the MQTT map whether it is a command;
 *    4. region A: steering error, junction / gap / lost; post events;
 *    5. while holding LINE authority in LINE_FOLLOW: DRIVE(v, omega);
 *    6. publish the line snapshot.
 */

#include "subsystem3.h"
#include "line_cfg.h"
#include "line_track.h"
#include "line_barcode.h"
#include "drv_adc.h"
#include "../app_types.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

#define LINE_WAKE_TIMEOUT_MS    30      /* still heartbeats if the DMA stops */
#define LINE_LOG_BC_REJECT      31u     /* comms_rec_t code for a rejected read */
#define LINE_LOG_CAL_LEVELS     32u     /* 32/33/34 = L/R/barcode: max << 16 | min raw */
/* Diagnosis of a rejected barcode (2 Nov), right after LOG 31:
   35 = elements << 24 | length mm << 8 | darkest reading / 4 (0..250),
   36 = narrowest element << 16 | widest element, both in 0.1 mm.        */
#define LINE_LOG_BC_SHAPE       35u
#define LINE_LOG_BC_WIDTHS      36u
/* 37 = a dark pattern of 2..8 elements, dropped as "not a barcode" (a
   crossing line is 1): data = elements; LOG 35 / 36 follow.            */
#define LINE_LOG_BC_SHORT       37u
/* 38 = turn slip from the calibration spin, permille (0 = not found) */
#define LINE_LOG_TURN_GAIN      38u
/* 45 = its diagnosis: line crossings seen by L << 24 | by R << 16 | deg spun */
#define LINE_LOG_SLIP_DIAG      45u

static line_cal_t   g_cal;
static line_track_t g_trk;
static bc_capture_t g_bc;
static line_slip_t  g_slip;

/* LOG 35 / 36 for the capture that was just rejected (still in g_bc). */
static void bc_reject_detail(bc_capture_t const * p_bc)
{
    uint32_t const len_mm = app_min_i32((int32_t)(p_bc->total_um / 1000u), 0xFFFF);
    uint32_t       w_min  = 0xFFFFu;
    uint32_t       w_max  = 0u;
    uint16_t       k;

    for (k = 0u; (k < p_bc->n) && (k <= BC_MAX_ELEMENTS); k++)
    {
        uint32_t const w = app_min_i32((int32_t)(p_bc->widths[k] / 100u), 0xFFFF);

        w_min = (w < w_min) ? w : w_min;
        w_max = (w > w_max) ? w : w_max;
    }
    if (p_bc->n == 0u)
    {
        w_min = 0u;
    }
    (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_LINE, LINE_LOG_BC_SHAPE,
                           (int32_t)(((uint32_t)app_min_i32(p_bc->n, 255) << 24) |
                                     (len_mm << 8) |
                                     ((uint32_t)p_bc->peak / 4u)));
    (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_LINE, LINE_LOG_BC_WIDTHS,
                           (int32_t)((w_min << 16) | w_max));
}

static void timing_pin_init(void)
{
#if APP_TIMING_PINS
    out_w(GPIO_OUT_CLR, 1uL << S3_PIN_TIMING);
    out_w(GPIO_OE_SET, 1uL << S3_PIN_TIMING);
    out_w(GPIO_CTRL(S3_PIN_TIMING), GPIO_CTRL_FUNCSEL_SIO);
#endif
}

static void timing_pin_toggle(void)
{
#if APP_TIMING_PINS
    out_w(GPIO_OUT_XOR, 1uL << S3_PIN_TIMING);
#endif
}

/* Barcode sensor ground speed in um per 1 ms sample (= mm/s). */
static uint32_t bc_step_um(motion_odom_t const * p_odom)
{
    /* yaw [cdeg/s] x pi / 18000 x offset [mm] = mm/s; offset + right, so a left turn
       (yaw > 0) speeds up a right-hand sensor and slows a left-hand one */
    int32_t const side = (int32_t)(((int64_t)p_odom->yaw_rate_cdeg_s * 3141593 * LINE_BC_OFFSET_MM) /
                                   ((int64_t)18000 * 1000000));
    return (uint32_t)app_abs_i32((int32_t)p_odom->v_mm_s + side);
}

EXPORT void line_task(INT stacd, void * p_exinf)
{
    uint16_t       block[DRV_ADC_BLOCK_LEN];
    line_snap_t    snap = {0};
    motion_odom_t  odom;
    mission_snap_t ms;
    uint32_t       seq;
    uint32_t       last_seq = 0u;
    bool           b_hw_ok;
    bool           b_cal_active = false;
    uint8_t        drive_seq = 0u;

    (void)stacd;
    (void)p_exinf;

    timing_pin_init();
    line_track_init(&g_trk);
    bc_init(&g_bc);
    b_hw_ok = (drv_adc_init() == E_OK);
    /* tk_get_tid(): on SMP this task may run before g_tid_line is stored */
    b_hw_ok = b_hw_ok && (drv_adc_start(tk_get_tid()) == E_OK);

    for (;;)
    {
        uint32_t step_um;
        uint32_t sum[LINE_CH_COUNT] = { 0u, 0u, 0u };
        uint16_t mean[LINE_CH_COUNT];
        uint8_t  ev = 0u;
        uint32_t i;

        (void)tk_slp_tsk(LINE_WAKE_TIMEOUT_MS);
        safety_heartbeat(SAFETY_HB_LINE);

        if (!b_hw_ok || !drv_adc_get_block(block, &seq) || (seq == last_seq))
        {
            snap.t_ms  = (uint32_t)app_time_ms();
            snap.flags = (uint8_t)((snap.flags & (uint8_t)~LINE_FLAG_HW_OK) |
                                   (b_hw_ok ? LINE_FLAG_HW_OK : 0u));
            line_put_snap(&snap);
            continue;
        }
        timing_pin_toggle();
        if ((last_seq != 0u) && ((seq - last_seq) > 1u))
        {
            snap.missed_blocks += (seq - last_seq) - 1u;
        }
        last_seq = seq;
        snap.blocks++;

        motion_get_odom(&odom);
        mission_get_snap(&ms);

        /* ---- 2. calibration (360 deg spin) -------------------------- */
        if (ms.cal_phase == (uint8_t)MISSION_CAL_PH_SPIN)
        {
            uint32_t cl = 0u;
            uint32_t cr = 0u;

            if (!b_cal_active)
            {
                line_cal_start(&g_cal);
                line_slip_start(&g_slip, odom.heading_cdeg);
                b_cal_active = true;
            }
            for (i = 0u; i < LINE_SAMPLES_PER_BLOCK; i++)
            {
                line_cal_sample(&g_cal, &block[i * LINE_CH_COUNT]);
                cl += block[(i * LINE_CH_COUNT) + LINE_CH_L];
                cr += block[(i * LINE_CH_COUNT) + LINE_CH_R];
            }
            line_slip_sample(&g_slip, odom.heading_cdeg, (uint16_t)(cl / LINE_SAMPLES_PER_BLOCK),
                             (uint16_t)(cr / LINE_SAMPLES_PER_BLOCK));
        }
        else if (b_cal_active && (ms.cal_phase == (uint8_t)MISSION_CAL_PH_FINISH))
        {
            bool const b_ok = line_cal_finish(&g_cal);
            uint8_t    ch;

            /* raw levels seen during the spin, for S3-M01 / A7 on the laptop */
            for (ch = 0u; ch < LINE_CH_COUNT; ch++)
            {
                (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_LINE,
                                       (uint16_t)(LINE_LOG_CAL_LEVELS + ch),
                                       (int32_t)(((uint32_t)g_cal.max[ch] << 16) | g_cal.min[ch]));
            }
            /* turn slip for the bypass turns (3 Nov) */
            {
                uint32_t diag = 0u;

                snap.turn_gain_pm = line_slip_gain(&g_slip, &g_cal, &diag);
                (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_LINE, LINE_LOG_TURN_GAIN,
                                       (int32_t)snap.turn_gain_pm);
                (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_LINE, LINE_LOG_SLIP_DIAG,
                                       (int32_t)diag);
            }
            b_cal_active      = false;
            snap.contrast_x10 = g_cal.contrast_x10;
            line_track_init(&g_trk);
            bc_init(&g_bc);
            (void)mission_post((uint8_t)MISSION_EV_CAL_DONE, (uint8_t)APP_SRC_LINE,
                               (uint16_t)APP_SRC_LINE, b_ok ? 1 : 0);
        }
        else if (ms.cal_phase == (uint8_t)MISSION_CAL_PH_NONE)
        {
            b_cal_active = false;       /* calibration aborted */
        }
        else
        {
            /* STILL phase: nothing for the IR sensors */
        }

        /* ---- 3. barcode region (per sample) ------------------------- */
        step_um = bc_step_um(&odom);
        for (i = 0u; i < LINE_SAMPLES_PER_BLOCK; i++)
        {
            uint16_t const * p_s = &block[i * LINE_CH_COUNT];
            bc_out_t         out;
            bc_result_t      r;
            bool             bc_was_capturing;

            sum[LINE_CH_L]  += p_s[LINE_CH_L];
            sum[LINE_CH_R]  += p_s[LINE_CH_R];
            sum[LINE_CH_BC] += p_s[LINE_CH_BC];
            if (!g_cal.b_valid || b_cal_active)
            {
                continue;
            }
            bc_was_capturing = (g_bc.state == (uint8_t)LINE_BC_CAPTURING);
            r = bc_sample(&g_bc, line_cal_norm(&g_cal, LINE_CH_BC, p_s[LINE_CH_BC]), step_um, &out);
            if (r == BC_VALID)
            {
                snap.barcode_ok++;
                snap.last_barcode = out.ch;
                (void)mission_post((uint8_t)MISSION_EV_BARCODE, (uint8_t)APP_SRC_LINE,
                                   out.b_reversed ? 1u : 0u, (int32_t)out.ch);
            }
            else if (r == BC_REJECT)
            {
                /* BARCODE_ERR: telemetry only, never a command (FR3) */
                snap.barcode_err++;
                snap.last_reject = out.reason;
                (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_LINE,
                                       LINE_LOG_BC_REJECT, (int32_t)out.reason);
                bc_reject_detail(&g_bc);
            }
            else if (bc_was_capturing && (g_bc.state == (uint8_t)LINE_BC_ARMED) && (g_bc.n >= 2u))
            {
                /* dropped silently by the decoder (< 9 elements): say so */
                (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_LINE,
                                       LINE_LOG_BC_SHORT, (int32_t)g_bc.n);
                bc_reject_detail(&g_bc);
            }
            else
            {
                /* nothing yet */
            }
        }

        /* ---- 4. line region (per block) ----------------------------- */
        for (i = 0u; i < LINE_CH_COUNT; i++)
        {
            mean[i] = (uint16_t)(sum[i] / LINE_SAMPLES_PER_BLOCK);
        }
        snap.raw_l  = line_cal_norm(&g_cal, LINE_CH_L, mean[LINE_CH_L]);
        snap.raw_r  = line_cal_norm(&g_cal, LINE_CH_R, mean[LINE_CH_R]);
        snap.raw_bc = line_cal_norm(&g_cal, LINE_CH_BC, mean[LINE_CH_BC]);
        if (g_cal.b_valid && !b_cal_active)
        {
            line_trk_cfg_t cfg;

            cfg.junction_um = (uint32_t)ms.junction_mm * 1000u;
            cfg.lost_um     = (uint32_t)ms.lost_mm * 1000u;
            cfg.cruise_mm_s = app_min_i32((int32_t)ms.cruise_mm_s, (int32_t)ms.speed_cap_mm_s);
            ev = line_track_step(&g_trk, &cfg, snap.raw_l, snap.raw_r,
                                 (uint32_t)app_abs_i32(odom.v_mm_s) * 10u);
            if ((ev & LINE_TRK_EV_JUNCTION) != 0u)
            {
                (void)mission_post((uint8_t)MISSION_EV_JUNCTION, (uint8_t)APP_SRC_LINE, 0u, 0);
            }
            if ((ev & LINE_TRK_EV_LOST) != 0u)
            {
                (void)mission_post((uint8_t)MISSION_EV_LINE_LOST, (uint8_t)APP_SRC_LINE, 0u, 0);
            }
            if ((ev & LINE_TRK_EV_FOUND) != 0u)
            {
                (void)mission_post((uint8_t)MISSION_EV_LINE_FOUND, (uint8_t)APP_SRC_LINE, 0u, 0);
            }
        }

        /* ---- 5. steering command ------------------------------------ */
        if ((odom.auth == (uint8_t)MOTION_SRC_LINE) &&
            (ms.state == (uint8_t)MISSION_ST_LINE_FOLLOW) && g_cal.b_valid)
        {
            int32_t omega = g_trk.omega;

            /* 7 Nov reading mode: steady steering while a code is read, so
               the side barcode IR crosses the bars on a straight path; not
               on curves (the line well off centre) */
            if ((g_bc.state == (uint8_t)LINE_BC_CAPTURING) && (app_abs_i32(g_trk.err) <= LINE_BC_HOLD_ERR))
            {
                omega = app_clamp_i32(omega, -LINE_BC_OMEGA_MAX, LINE_BC_OMEGA_MAX);
            }
            drive_seq++;
            (void)motion_drive((uint8_t)MOTION_SRC_LINE, drive_seq,
                               (int16_t)g_trk.v, (int16_t)omega);
        }

        /* ---- 6. snapshot -------------------------------------------- */
        snap.t_ms         = (uint32_t)app_time_ms();
        snap.err          = (int16_t)g_trk.err;
        snap.omega_mrad_s = (int16_t)g_trk.omega;
        snap.bc_state     = g_bc.state;
        snap.sens         = (uint8_t)((g_trk.b_white_l ? 0u : LINE_SENS_L) | (g_trk.b_white_r ? 0u : LINE_SENS_R));
        snap.flags        = (uint8_t)((g_cal.b_valid ? LINE_FLAG_CALIBRATED : 0u) |
                                      (g_trk.b_lost ? LINE_FLAG_LOST : 0u) |
                                      (g_trk.b_junction ? LINE_FLAG_JUNCTION : 0u) |
                                      ((odom.auth == (uint8_t)MOTION_SRC_LINE) ? LINE_FLAG_HAS_AUTH : 0u) |
                                      ((!g_trk.b_white_l || !g_trk.b_white_r) ? LINE_FLAG_ON_LINE : 0u) |
                                      LINE_FLAG_HW_OK |
                                      ((g_trk.both_white_um > 0u) ? LINE_FLAG_GAP : 0u));
        line_put_snap(&snap);
    }
}
