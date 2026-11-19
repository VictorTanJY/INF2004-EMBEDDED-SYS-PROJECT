/** @file line_task.c
 *
 *  @brief Subsystem 3 -- LineBarcodeTask (priority 7, core 1), woken by
 *  adc_dma_isr every 10 ms (Week 6 Sections 3 and 6.2).  Owner: Buddy 3.
 *
 *  Every block (10 samples per channel):
 *  1. heartbeat; take the block; count missed blocks (NFR2);
 *  2. CALIBRATING: record min/max during the spin, finish on request;
 *  3. region B: feed the 10 barcode samples, with the distance the
 *  barcode sensor moved per sample, to the capture state machine;
 *  a valid read posts BARCODE(character) -- MissionTask decides from
 *  the MQTT map whether it is a command;
 *  4. region A: steering error, junction / gap / lost; post events;
 *  5. while holding LINE authority in LINE_FOLLOW: DRIVE(v, omega);
 *  6. publish the line snapshot.
 */

#include "line_task.h"
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

#define LOG_READ_SETS      4u /* entries of k_count_rd */
#define LOG_ELEMS_PER_WORD 3u /* 8-bit widths in one LOG word */

#define LINE_WAKE_TIMEOUT_MS 30 /* still heartbeats if the DMA stops */

/* comms_proto_rec_t code for a rejected read */
#define LINE_LOG_BC_REJECT 31u

/* 32/33/34 = L/R/barcode: max << 16 | min raw */
#define LINE_LOG_CAL_LEVELS 32u

/* Diagnosis of a rejected barcode (2 Nov), right after LOG 31:
   35 = elements << 24 | length mm << 8 | darkest reading / 4 (0..250),
   36 = narrowest element << 16 | widest element, both in 0.1 mm.        */
#define LINE_LOG_BC_SHAPE  35u
#define LINE_LOG_BC_WIDTHS 36u

/* 37 = a dark pattern of 2..8 elements, dropped as "not a barcode" (a
   crossing line is 1): data = elements; LOG 35 / 36 follow.            */
#define LINE_LOG_BC_SHORT 37u

/* 73 (7 Nov) = barcode sensor levels after every capture, 0..1000 each:
   decoded by pass << 29 (0 swing, 1 mid, 2 low, 3 adaptive swing, 4 best
   match, 5 exact without an end speck, 6 best match without one) | white noise
   (darkest - lightest white, max 511) << 20 | lightest white << 10 | darkest
   bar.  For choosing BC_DELTA. */
#define LINE_LOG_BC_LEVELS 73u

/* 9 Nov, after a rejected read: 76 = elements found by each reading,
   swing << 24 | low threshold << 16 | adaptive 20 % << 8 | adaptive 10 %;
   then 75 = every width of the swing and the adaptive 10 % readings,
   three per record: reading (0 swing, 3 adaptive) << 30 | index of the
   first << 24 | three widths of 8 bits in 0.2 mm (255 = 51 mm or more,
   0 = none). */
#define LINE_LOG_BC_ELEM   75u
#define LINE_LOG_BC_COUNTS 76u
#define LINE_LOG_ELEM_MAX  45u /* widths logged per reading (15 records) */

/* 38 = turn slip from the calibration spin, permille (0 = not found) */
/* 14 Nov, lifted: raw_l << 20 | raw_r << 10 | raw_bc */
#define LINE_LOG_NO_FLOOR 79u

/* 14 Nov: 80 = how long the decode at the end of a capture took, us
   (the line task's heartbeat allows 50 ms; best match took longer). */
#define LINE_LOG_BC_DECODE_US 80u
#define LINE_LOG_TURN_GAIN    38u

/* 45 = its diagnosis: line crossings seen by L << 24 | by R << 16 | deg spun */
#define LINE_LOG_SLIP_DIAG 45u

/** LineBarcodeTask's state, kept from one block to the next. */
typedef struct
{
    line_types_snap_t snap;
    uint32_t          last_seq;
    bool              b_hw_ok;
    bool              b_cal_active;
    bool     b_all_black; /**< 14 Nov: all three IR sensors black now */
    bool     b_no_floor;  /**< ...for LINE_NOFLOOR_UM: lifted         */
    uint32_t nofloor_um;
    uint8_t  drive_seq;
} line_task_state_t;

static line_track_cal_t       g_cal;
static line_track_t           g_trk;
static line_barcode_capture_t g_bc;
static line_track_slip_t      g_slip;

static void     bc_reject_detail(line_barcode_capture_t const * p_bc);
static void     bc_widths_log(line_barcode_capture_t const * p_bc);
static void     bc_levels_log(line_barcode_capture_t const * p_bc);
static void     timing_pin_init(void);
static void     timing_pin_toggle(void);
static uint32_t bc_step_um(motion_types_odom_t const * p_odom);
static void line_block(line_task_state_t * p_st,
                       uint16_t const block[DRV_ADC_BLOCK_LEN], uint32_t seq);
static void line_cal_spin(line_task_state_t *         p_st,
                          uint16_t const              block[DRV_ADC_BLOCK_LEN],
                          motion_types_odom_t const * p_odom);
static void line_cal_finish(line_task_state_t * p_st);
static void line_barcode_block(line_task_state_t * p_st,
                               uint16_t const      block[DRV_ADC_BLOCK_LEN],
                               motion_types_odom_t const * p_odom,
                               uint32_t                    sum[LINE_CH_COUNT]);
static void line_barcode_one(line_task_state_t * p_st, uint16_t bc_raw,
                             uint32_t step_um);
static void line_region(line_task_state_t *          p_st,
                        uint32_t const               sum[LINE_CH_COUNT],
                        motion_types_odom_t const *  p_odom,
                        mission_types_snap_t const * p_mission);
static void line_no_floor(line_task_state_t *         p_st,
                          motion_types_odom_t const * p_odom);
static void line_snapshot(line_task_state_t *         p_st,
                          motion_types_odom_t const * p_odom);

EXPORT void line_task_thread (INT stacd, void * p_exinf)
{
    uint16_t          block[DRV_ADC_BLOCK_LEN];
    line_task_state_t state = {0};
    uint32_t          seq;

    (void)stacd;
    (void)p_exinf;

    timing_pin_init();
    line_track_init(&g_trk);
    line_barcode_init(&g_bc);
    state.b_hw_ok = (E_OK == drv_adc_init());
    /* tk_get_tid(): on SMP this task may run before subsystem3_start()
       has its ID */
    state.b_hw_ok = state.b_hw_ok && (E_OK == drv_adc_start(tk_get_tid()));

    for (;;)
    {
        (void)tk_slp_tsk(LINE_WAKE_TIMEOUT_MS);
        subsystem4_heartbeat(SAFETY_HB_LINE);

        if (!state.b_hw_ok || !drv_adc_get_block(block, &seq)
            || (seq == state.last_seq))
        {
            /* Casts (uint8_t)((state.snap.flags & ...)): flags are 8 bits; only
               8-bit masks are combined; (uint8_t)~LINE_FLAG_HW_OK: the 8-bit
               mask of every flag but HW_OK (low 8 bits kept on purpose). */
            state.snap.t_ms = app_time_ms();
            state.snap.flags =
                (uint8_t)((state.snap.flags & (uint8_t)~LINE_FLAG_HW_OK)
                          | (state.b_hw_ok ? LINE_FLAG_HW_OK : 0u));
            subsystem3_put_snap(&state.snap);
        }
        else
        {
            line_block(&state, block, seq);
        }
    }
}

/** One new block of DRV_ADC_BLOCK_LEN samples (sequence number seq). */
static void line_block (line_task_state_t * p_st,
                        uint16_t const block[DRV_ADC_BLOCK_LEN], uint32_t seq)
{
    motion_types_odom_t  odom;
    mission_types_snap_t mission;
    uint32_t             sum[LINE_CH_COUNT] = {0u, 0u, 0u};

    timing_pin_toggle();
    if ((p_st->last_seq != 0u) && ((seq - p_st->last_seq) > 1u))
    {
        p_st->snap.missed_blocks += (seq - p_st->last_seq) - 1u;
    }
    p_st->last_seq = seq;
    p_st->snap.blocks++;

    subsystem2_get_odom(&odom);
    subsystem5_get_mission_snap(&mission);

    /* ---- 2. calibration (360 deg spin) -------------------------- */
    if (MISSION_CAL_PH_SPIN == mission.cal_phase)
    {
        line_cal_spin(p_st, block, &odom);
    }
    else if (p_st->b_cal_active && (MISSION_CAL_PH_FINISH == mission.cal_phase))
    {
        line_cal_finish(p_st);
    }
    else if (MISSION_CAL_PH_NONE == mission.cal_phase)
    {
        p_st->b_cal_active = false; /* calibration aborted */
    }
    else
    {
        /* STILL phase: nothing for the IR sensors */
    }

    /* ---- 3. barcode region (per sample) ------------------------- */
    line_barcode_block(p_st, block, &odom, sum);

    /* ---- 4. line region (per block) ----------------------------- */
    line_region(p_st, sum, &odom, &mission);

    /* ---- 4b. no floor: lifted (14 Nov, see LINE_NOFLOOR_*) ------- */
    line_no_floor(p_st, &odom);

    /* ---- 5. steering command ------------------------------------ */
    if ((MOTION_SRC_LINE == odom.auth)
        && (MISSION_ST_LINE_FOLLOW == mission.state) && g_cal.b_valid)
    {
        int32_t omega = g_trk.omega;

        /* 7 Nov reading mode: steady steering while a code is read, so
           the side barcode IR crosses the bars on a straight path; not
           on curves (the line well off centre) */
        if ((LINE_BC_CAPTURING == g_bc.state)
            && (app_types_abs_i32(g_trk.err) <= LINE_BC_HOLD_ERR))
        {
            omega = app_types_clamp_i32(omega, -LINE_BC_OMEGA_MAX,
                                        LINE_BC_OMEGA_MAX);
        }
        /* Casts (int16_t)g_trk.v_mm_s: v_mm_s is at most the cruise speed (<=
           350 mm/s); (int16_t)omega: omega is clamped to +-LINE_OMEGA_MAX by
           line_track_step(). */
        p_st->drive_seq = motion_types_seq_next(p_st->drive_seq);
        (void)motion_api_drive(MOTION_SRC_LINE, p_st->drive_seq,
                               (int16_t)g_trk.v_mm_s, (int16_t)omega);
    }

    /* ---- 6. snapshot -------------------------------------------- */
    line_snapshot(p_st, &odom);
}

/** Calibration spin: record the levels and the line crossings. */
static void line_cal_spin (line_task_state_t *         p_st,
                           uint16_t const              block[DRV_ADC_BLOCK_LEN],
                           motion_types_odom_t const * p_odom)
{
    uint32_t cross_left  = 0u;
    uint32_t cross_right = 0u;
    uint32_t idx;

    if (!p_st->b_cal_active)
    {
        line_track_cal_start(&g_cal);
        line_track_slip_start(&g_slip, p_odom->heading_cdeg);
        p_st->b_cal_active = true;
    }
    for (idx = 0u; idx < LINE_SAMPLES_PER_BLOCK; idx++)
    {
        line_track_cal_sample(&g_cal, &block[idx * LINE_CH_COUNT]);
        cross_left += block[(idx * LINE_CH_COUNT) + LINE_CH_L];
        cross_right += block[(idx * LINE_CH_COUNT) + LINE_CH_R];
    }
    /* Casts (uint16_t)(cross_left / ...), (uint16_t)(cross_right / ...): a mean
       of 12-bit ADC samples. */
    line_track_slip_sample(&g_slip, p_odom->heading_cdeg,
                           (uint16_t)(cross_left / LINE_SAMPLES_PER_BLOCK),
                           (uint16_t)(cross_right / LINE_SAMPLES_PER_BLOCK));
}

/** Calibration finished: levels, turn slip, report CAL_DONE. */
static void line_cal_finish (line_task_state_t * p_st)
{
    bool const b_ok = line_track_cal_finish(&g_cal);
    uint8_t    chan;
    uint32_t   diag = 0u;

    /* raw levels seen during the spin, for S3-M01 / A7 on the laptop */
    for (chan = 0u; chan < LINE_CH_COUNT; chan++)
    {
        /* Casts (uint16_t)(LINE_LOG_CAL_LEVELS + ...): the value is 32..287;
           (int32_t)(((uint32_t)g_cal.max[chan ...)): bit pattern of a packed
           LOG word, not an arithmetic value; (uint32_t)g_cal.max[chan]: every
           uint16_t value fits. */
        (void)subsystem1_telem_post(
            COMMS_KIND_LOG, APP_SRC_LINE,
            (uint16_t)(LINE_LOG_CAL_LEVELS + chan),
            (int32_t)(((uint32_t)g_cal.max[chan] << 16) | g_cal.min[chan]));
    }
    /* turn slip for the bypass turns (3 Nov) */
    /* Cast (int32_t)diag: bit pattern of the packed diagnosis word. */
    p_st->snap.turn_gain_pm = line_track_slip_gain(&g_slip, &g_cal, &diag);
    (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_LINE,
                                LINE_LOG_TURN_GAIN, p_st->snap.turn_gain_pm);
    (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_LINE,
                                LINE_LOG_SLIP_DIAG, (int32_t)diag);
    p_st->b_cal_active      = false;
    p_st->snap.contrast_x10 = g_cal.contrast_x10;
    line_track_init(&g_trk);
    line_barcode_init(&g_bc);
    (void)subsystem5_mission_post(MISSION_EV_CAL_DONE, APP_SRC_LINE,
                                  APP_SRC_LINE, b_ok ? 1 : 0);
}

/** Barcode region: feed every sample to the capture state machine, and sum
   each channel for the line region. */
static void line_barcode_block (line_task_state_t * p_st,
                                uint16_t const      block[DRV_ADC_BLOCK_LEN],
                                motion_types_odom_t const * p_odom,
                                uint32_t                    sum[LINE_CH_COUNT])
{
    uint32_t const step_um = bc_step_um(p_odom);
    uint32_t       idx;

    for (idx = 0u; idx < LINE_SAMPLES_PER_BLOCK; idx++)
    {
        uint16_t const * p_s = &block[idx * LINE_CH_COUNT];

        sum[LINE_CH_L] += p_s[LINE_CH_L];
        sum[LINE_CH_R] += p_s[LINE_CH_R];
        sum[LINE_CH_BC] += p_s[LINE_CH_BC];
        if (g_cal.b_valid && !p_st->b_cal_active)
        {
            line_barcode_one(p_st, p_s[LINE_CH_BC], step_um);
        }
    }
}

/** One barcode sample (raw reading bc_raw; the sensor moved step_um). */
static void line_barcode_one (line_task_state_t * p_st, uint16_t bc_raw,
                              uint32_t step_um)
{
    bool const            b_was_capturing = (LINE_BC_CAPTURING == g_bc.state);
    uint32_t const        t0_us = b_was_capturing ? app_time_us() : 0u;
    line_barcode_out_t    out;
    line_barcode_result_t result;

    result = line_barcode_sample(
        &g_bc, line_track_cal_norm(&g_cal, LINE_CH_BC, bc_raw), step_um, &out);
    if (b_was_capturing && ((BC_VALID == result) || (BC_REJECT == result)))
    {
        /* Cast (int32_t)(app_time_us() - t0_us): a decode time in us, far below
           2^31. */
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_LINE,
                                    LINE_LOG_BC_DECODE_US,
                                    (int32_t)(app_time_us() - t0_us));
    }
    if (BC_VALID == result)
    {
        p_st->snap.barcode_ok++;
        p_st->snap.last_barcode = out.chr;
        (void)subsystem5_mission_post(MISSION_EV_BARCODE, APP_SRC_LINE,
                                      out.b_reversed ? 1u : 0u, out.chr);
        bc_levels_log(&g_bc);
    }
    else if (BC_REJECT == result)
    {
        /* BARCODE_ERR: telemetry only, never a command (FR3) */
        p_st->snap.barcode_err++;
        p_st->snap.last_reject = out.reason;
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_LINE,
                                    LINE_LOG_BC_REJECT, out.reason);
        bc_reject_detail(&g_bc);
        bc_levels_log(&g_bc);
        bc_widths_log(&g_bc);
    }
    else if (b_was_capturing && (LINE_BC_ARMED == g_bc.state)
             && (g_bc.n_elems >= 2u))
    {
        /* dropped silently by the decoder (< 9 elements): say so */
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_LINE,
                                    LINE_LOG_BC_SHORT, g_bc.n_elems);
        bc_reject_detail(&g_bc);
        bc_levels_log(&g_bc);
    }
    else
    {
        /* nothing yet */
    }
}

/** Line region: block means, steering error, junction / lost / found. */
static void line_region (line_task_state_t *          p_st,
                         uint32_t const               sum[LINE_CH_COUNT],
                         motion_types_odom_t const *  p_odom,
                         mission_types_snap_t const * p_mission)
{
    uint16_t mean[LINE_CH_COUNT];
    uint32_t idx;

    for (idx = 0u; idx < LINE_CH_COUNT; idx++)
    {
        /* Cast (uint16_t)(sum[idx] / ...): a mean of 12-bit ADC samples. */
        mean[idx] = (uint16_t)(sum[idx] / LINE_SAMPLES_PER_BLOCK);
    }
    p_st->snap.raw_l = line_track_cal_norm(&g_cal, LINE_CH_L, mean[LINE_CH_L]);
    p_st->snap.raw_r = line_track_cal_norm(&g_cal, LINE_CH_R, mean[LINE_CH_R]);
    p_st->snap.raw_bc =
        line_track_cal_norm(&g_cal, LINE_CH_BC, mean[LINE_CH_BC]);
    if (g_cal.b_valid && !p_st->b_cal_active)
    {
        line_track_cfg_t cfg;
        uint8_t          events;

        /* Casts (uint32_t)p_mission->junction_mm, (uint32_t)p_mission->lost_mm:
           every uint16_t value fits; (uint32_t)app_types_abs_i32(...): the
           value is 0..32768. */
        cfg.junction_um = (uint32_t)p_mission->junction_mm * 1000u;
        cfg.lost_um     = (uint32_t)p_mission->lost_mm * 1000u;
        cfg.cruise_mm_s = app_types_min_i32(p_mission->cruise_mm_s,
                                            p_mission->speed_cap_mm_s);
        events =
            line_track_step(&g_trk, &cfg, p_st->snap.raw_l, p_st->snap.raw_r,
                            (uint32_t)app_types_abs_i32(p_odom->v_mm_s) * 10u);
        if ((events & LINE_TRK_EV_JUNCTION) != 0u)
        {
            (void)subsystem5_mission_post(MISSION_EV_JUNCTION, APP_SRC_LINE, 0u,
                                          0);
        }
        if ((events & LINE_TRK_EV_LOST) != 0u)
        {
            (void)subsystem5_mission_post(MISSION_EV_LINE_LOST, APP_SRC_LINE,
                                          0u, 0);
        }
        if ((events & LINE_TRK_EV_FOUND) != 0u)
        {
            (void)subsystem5_mission_post(MISSION_EV_LINE_FOUND, APP_SRC_LINE,
                                          0u, 0);
        }
    }
}

/** Published in the snapshot (LINE_SENS_NO_FLOOR); ImuTask undoes the
   humps measured in the air and then sets SAFETY_FLT_LIFT. */
static void line_no_floor (line_task_state_t *         p_st,
                           motion_types_odom_t const * p_odom)
{
    if (g_cal.b_valid && !p_st->b_cal_active
        && (p_st->snap.raw_l >= LINE_NOFLOOR_NORM)
        && (p_st->snap.raw_r >= LINE_NOFLOOR_NORM)
        && (p_st->snap.raw_bc >= LINE_NOFLOOR_NORM))
    {
        p_st->b_all_black = true;
        /* um per 10 ms block */
        /* Cast (uint32_t)app_types_abs_i32(...): the value is 0..32768. */
        p_st->nofloor_um += (uint32_t)app_types_abs_i32(p_odom->v_mm_s) * 10u;
        if (!p_st->b_no_floor && (p_st->nofloor_um >= LINE_NOFLOOR_UM))
        {
            /* Casts (int32_t)(((uint32_t)p_st->snap.raw ...)): bit pattern of a
               packed LOG word, not an arithmetic value;
               (uint32_t)p_st->snap.raw_l, (uint32_t)p_st->snap.raw_r,
               (uint32_t)p_st->snap.raw_bc: every uint16_t value fits. */
            p_st->b_no_floor = true;
            (void)subsystem1_telem_post(
                COMMS_KIND_LOG, APP_SRC_LINE, LINE_LOG_NO_FLOOR,
                (int32_t)(((uint32_t)p_st->snap.raw_l << 20)
                          | ((uint32_t)p_st->snap.raw_r << 10)
                          | (uint32_t)p_st->snap.raw_bc));
        }
    }
    else
    {
        p_st->b_all_black = false;
        p_st->b_no_floor  = false;
        p_st->nofloor_um  = 0u;
    }
}

/** Publish the line snapshot. */
static void line_snapshot (line_task_state_t *         p_st,
                           motion_types_odom_t const * p_odom)
{
    /* Casts (int16_t)g_trk.err: the steering error is a difference of two
       0..1000 readings; (int16_t)g_trk.omega: omega is clamped to
       +-LINE_OMEGA_MAX by line_track_step();
       (uint8_t)((g_trk.b_white_l ? 0u : ...)),
       (uint8_t)((g_cal.b_valid ? ...)): an OR of 8-bit flag constants. */
    p_st->snap.t_ms         = app_time_ms();
    p_st->snap.err          = (int16_t)g_trk.err;
    p_st->snap.omega_mrad_s = (int16_t)g_trk.omega;
    p_st->snap.bc_state     = g_bc.state;
    p_st->snap.sens = (uint8_t)((g_trk.b_white_l ? 0u : LINE_SENS_L)
                                | (g_trk.b_white_r ? 0u : LINE_SENS_R)
                                | (p_st->b_all_black ? LINE_SENS_ALL_BLACK : 0u)
                                | (p_st->b_no_floor ? LINE_SENS_NO_FLOOR : 0u));
    p_st->snap.flags =
        (uint8_t)((g_cal.b_valid ? LINE_FLAG_CALIBRATED : 0u)
                  | (g_trk.b_lost ? LINE_FLAG_LOST : 0u)
                  | (g_trk.b_junction ? LINE_FLAG_JUNCTION : 0u)
                  | ((MOTION_SRC_LINE == p_odom->auth) ? LINE_FLAG_HAS_AUTH
                                                       : 0u)
                  | ((!g_trk.b_white_l || !g_trk.b_white_r) ? LINE_FLAG_ON_LINE
                                                            : 0u)
                  | LINE_FLAG_HW_OK
                  | ((g_trk.both_white_um > 0u) ? LINE_FLAG_GAP : 0u));
    subsystem3_put_snap(&p_st->snap);
}

/** LOG 35 / 36 for the capture that was just rejected (still in g_bc). */
static void bc_reject_detail (line_barcode_capture_t const * p_bc)
{
    /* Cast (int32_t)(p_bc->total_um / 1000u): the value is 0..4294967. */
    uint32_t const len_mm =
        app_types_min_i32((int32_t)(p_bc->total_um / 1000u), 0xFFFF);
    uint32_t w_min = 0xFFFFu;
    uint32_t w_max = 0u;
    uint16_t kdx;

    for (kdx = 0u; (kdx < p_bc->n_elems) && (kdx <= BC_MAX_ELEMENTS); kdx++)
    {
        /* Cast (int32_t)(p_bc->widths[kdx] / ...): the value is 0..42949672. */
        uint32_t const width =
            app_types_min_i32((int32_t)(p_bc->widths[kdx] / 100u), 0xFFFF);

        w_min = (width < w_min) ? width : w_min;
        w_max = (width > w_max) ? width : w_max;
    }
    if (0u == p_bc->n_elems)
    {
        w_min = 0u;
    }
    /* Casts (int32_t)(((uint32_t)app_types_min_ ...)),
       (int32_t)((w_min << 16) | w_max): bit pattern of a packed LOG word, not
       an arithmetic value; (uint32_t)app_types_min_i32(...): the value is
       0..255; (uint32_t)p_bc->peak: every uint16_t value fits. */
    (void)subsystem1_telem_post(
        COMMS_KIND_LOG, APP_SRC_LINE, LINE_LOG_BC_SHAPE,
        (int32_t)(((uint32_t)app_types_min_i32(p_bc->n_elems, 255) << 24)
                  | (len_mm << 8) | ((uint32_t)p_bc->peak / 4u)));
    (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_LINE,
                                LINE_LOG_BC_WIDTHS,
                                (int32_t)((w_min << 16) | w_max));
}

/** LOG 76 and 75 for the capture that was just rejected (still in g_bc). */
static void bc_widths_log (line_barcode_capture_t const * p_bc)
{
    static uint32_t widths[BC_MAX_ELEMENTS + 1u];
    uint32_t        counts = 0u;
    uint8_t         reading;

    {
        static uint8_t const k_count_rd[LOG_READ_SETS] = {0u, 2u, 3u, 5u};
        uint8_t              jdx;

        for (jdx = 0u; jdx < LOG_READ_SETS; jdx++)
        {
            uint16_t const n_elems =
                line_barcode_reading(p_bc, k_count_rd[jdx], widths);

            /* Cast (uint32_t)app_types_min_i32(n_elems ...): the value is
               0..255. */
            counts |= (uint32_t)app_types_min_i32(n_elems, 255)
                      << (8u * (3u - jdx));
        }
    }
    /* Cast (int32_t)counts: bit pattern of the packed LOG word. */
    (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_LINE,
                                LINE_LOG_BC_COUNTS, (int32_t)counts);
    for (reading = 0u; reading < BC_READINGS;
         reading += 5u) /* swing (0), adaptive 10 % (5) */
    {
        uint16_t const n_elems = line_barcode_reading(p_bc, reading, widths);
        uint16_t       kdx;

        for (kdx = 0u; (kdx < n_elems) && (kdx < LINE_LOG_ELEM_MAX);
             kdx += LOG_ELEMS_PER_WORD)
        {
            /* Cast (uint32_t)kdx: every uint16_t value fits. */
            uint32_t data =
                (((0u == reading) ? 0u : 3u) << 30) | ((uint32_t)kdx << 24);
            uint8_t jdx;

            for (jdx = 0u; jdx < LOG_ELEMS_PER_WORD; jdx++)
            {
                uint32_t const elem = ((kdx + jdx) < n_elems)
                                          ? ((widths[kdx + jdx] + 100u) / 200u)
                                          : 0u; /* 0.2 mm */

                data |=
                    ((elem > 255u)
                         ? 255u
                         : (((0u == elem) && ((kdx + jdx) < n_elems)) ? 1u
                                                                      : elem))
                    << (8u * (2u - jdx));
            }
            /* Cast (int32_t)data: bit pattern of the packed LOG word. */
            (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_LINE,
                                        LINE_LOG_BC_ELEM, (int32_t)data);
        }
    }
}

/** LOG 73 for the capture that just ended (still in g_bc). */
static void bc_levels_log (line_barcode_capture_t const * p_bc)
{
    /* Cast (uint32_t)(p_bc->q_hi - ...): q_hi >= q_lo (the lightest and darkest
       white reading). */
    uint32_t const noise =
        (p_bc->q_hi > p_bc->q_lo) ? (uint32_t)(p_bc->q_hi - p_bc->q_lo) : 0u;

    /* Casts (int32_t)(((p_bc->pass & 7u) << ...)): bit pattern of a packed LOG
       word, not an arithmetic value; (uint32_t)app_types_min_i32(...): noise is
       limited to 0..511 here; (int32_t)noise: noise is a difference of 0..1000
       readings; (uint32_t)app_types_min_i32(...): the value is 0..1023. */
    (void)subsystem1_telem_post(
        COMMS_KIND_LOG, APP_SRC_LINE, LINE_LOG_BC_LEVELS,
        (int32_t)(((p_bc->pass & 7u) << 29)
                  | ((uint32_t)app_types_min_i32((int32_t)noise, 511) << 20)
                  | ((uint32_t)app_types_min_i32(p_bc->q_lo, 1023) << 10)
                  | (uint32_t)app_types_min_i32(p_bc->peak, 1023)));
}

/** With APP_TIMING_PINS, make S3_PIN_TIMING an output for the oscilloscope. */
static void timing_pin_init (void)
{
#if APP_TIMING_PINS
    out_w(APP_REGS_GPIO_OUT_CLR, 1uL << S3_PIN_TIMING);
    out_w(APP_REGS_GPIO_OE_SET, 1uL << S3_PIN_TIMING);
    out_w(app_regs_gpio_ctrl(S3_PIN_TIMING), APP_REGS_FUNCSEL_SIO);
#endif
}

/** With APP_TIMING_PINS, toggle S3_PIN_TIMING once per block. */
static void timing_pin_toggle (void)
{
#if APP_TIMING_PINS
    out_w(APP_REGS_GPIO_OUT_XOR, 1uL << S3_PIN_TIMING);
#endif
}

/** Barcode sensor ground speed in um per 1 ms sample (= mm/s). */
static uint32_t bc_step_um (motion_types_odom_t const * p_odom)
{
    /* yaw [cdeg/s] x pi / 18000 x offset [mm] = mm/s; offset + right, so a left
       turn (yaw > 0) speeds up a right-hand sensor and slows a left-hand one */
    /* Casts (int32_t)(((int64_t)p_odom->yaw_rat ...)): the value is -257..257;
       (int64_t)p_odom->yaw_rate_cdeg_s, (int32_t)p_odom->v_mm_s: every int16_t
       value fits; (int64_t)18000: a constant that fits;
       (uint32_t)app_types_abs_i32(...): the value is 0..2147516416. */
    int32_t const side = (int32_t)(((int64_t)p_odom->yaw_rate_cdeg_s * 3141593
                                    * LINE_BC_OFFSET_MM)
                                   / ((int64_t)18000 * 1000000));
    return (uint32_t)app_types_abs_i32((int32_t)p_odom->v_mm_s + side);
}

/*** end of file ***/
