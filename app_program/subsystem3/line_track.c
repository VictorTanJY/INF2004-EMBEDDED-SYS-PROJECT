/*
 *  line_track.c
 *  Subsystem 3 -- calibration and line tracking.  See line_track.h.
 */

#include "line_track.h"
#include "../app_pack.h"
#include "../app_types.h"

/* ------------------------------------------------------------------ */
void line_cal_start (line_cal_t * p_cal)
{
    uint8_t chan;

    for (chan = 0u; chan < LINE_CH_COUNT; chan++)
    {
        p_cal->min[chan] = 0xFFFFu;
        p_cal->max[chan] = 0u;
    }
}

void line_cal_sample (line_cal_t * p_cal, uint16_t const raw[LINE_CH_COUNT])
{
    uint8_t chan;

    for (chan = 0u; chan < LINE_CH_COUNT; chan++)
    {
        if (raw[chan] < p_cal->min[chan])
        {
            p_cal->min[chan] = raw[chan];
        }
        if (raw[chan] > p_cal->max[chan])
        {
            p_cal->max[chan] = raw[chan];
        }
    }
}

bool line_cal_finish (line_cal_t * p_cal)
{
    uint16_t white[LINE_CH_COUNT];
    uint16_t black[LINE_CH_COUNT];
    uint16_t worst = 0xFFFFu;
    bool     b_ok  = true;
    uint8_t  chan;

    for (chan = 0u; chan < LINE_CH_COUNT; chan++)
    {
        uint32_t low  = p_cal->min[chan];
        uint32_t high = p_cal->max[chan];
        uint32_t contrast;

        if (LINE_CH_BC == chan)
        {
            /* Barcode sensor: fixed levels (line_cfg.h), never judged by
               the spin.  min/max are still kept for LOG 34.            */
            white[chan] = (uint16_t)LINE_BC_WHITE_RAW;
            black[chan] = (uint16_t)LINE_BC_BLACK_RAW;
        }
        else
        {
            if ((low > high) || ((high - low) < LINE_CAL_MIN_SPAN))
            {
                b_ok     = false; /* never saw both black and white */
                contrast = 0u;
            }
            else
            {
                /* AO ratio of the two levels, x 10 (3:1 = 30) */
                contrast = (0u == low) ? 999u : ((high * 10u) / low);
            }
#if LINE_BLACK_IS_HIGH
            white[chan] = (uint16_t)low;
            black[chan] = (uint16_t)high;
#else
            white[chan] = (uint16_t)high;
            black[chan] = (uint16_t)low;
#endif
            if (contrast < worst)
            {
                worst = (uint16_t)contrast;
            }
        }
    }
    if (worst < LINE_CONTRAST_MIN_X10)
    {
        b_ok = false;
    }
    p_cal->contrast_x10 = worst;
    if (b_ok)
    {
        for (chan = 0u; chan < LINE_CH_COUNT; chan++)
        {
            p_cal->white[chan] = white[chan];
            p_cal->black[chan] = black[chan];
        }
        p_cal->b_valid = true;
    }
    return b_ok;
}

uint16_t line_cal_norm (line_cal_t const * p_cal, uint8_t chan, uint16_t raw)
{
    int32_t const white = (int32_t)p_cal->white[chan];
    int32_t const black = (int32_t)p_cal->black[chan];
    int32_t       norm;

    if (!p_cal->b_valid || (white == black))
    {
        return 0u;
    }
    norm = (((int32_t)raw - white) * 1000) / (black - white);
    return (uint16_t)app_clamp_i32(norm, 0, 1000);
}

/* ------------------------------------------------------------------ */
void line_track_init (line_track_t * p_t)
{
    p_t->b_black_l         = false;
    p_t->b_black_r         = false;
    p_t->b_white_l         = false;
    p_t->b_white_r         = false;
    p_t->b_lost            = false;
    p_t->b_junction        = false;
    p_t->b_have_err        = false;
    p_t->err               = 0;
    p_t->prev_err          = 0;
    p_t->line_err          = 0;
    p_t->exit_err          = 0;
    p_t->peak_err          = 0;
    p_t->peak_um           = 0u;
    p_t->omega             = 0;
    p_t->v_mm_s            = 0;
    p_t->both_black_um     = 0u;
    p_t->both_white_um     = 0u;
    p_t->straight_um       = 0u;
    p_t->since_junction_um = LINE_JUNCTION_REFRACT_UM;
}

/* Steering ramp: move the turn command at most LINE_OMEGA_STEP per block
   (10 ms), so a flip of the line error does not jump the wheel speeds. */
static int32_t steer_toward (int32_t now, int32_t target)
{
    return app_clamp_i32(target, now - LINE_OMEGA_STEP, now + LINE_OMEGA_STEP);
}

static bool is_black (bool b_black, uint16_t norm)
{
    return b_black ? (norm >= LINE_BLACK_OFF) : (norm > LINE_BLACK_ON);
}

static bool is_white (bool b_white, uint16_t norm)
{
    return b_white ? (norm <= LINE_WHITE_OFF) : (norm < LINE_WHITE_ON);
}

uint8_t line_track_step (line_track_t * p_t, line_trk_cfg_t const * p_cfg,
                         uint16_t norm_l, uint16_t norm_r, uint32_t step_um)
{
    uint8_t events = (uint8_t)LINE_TRK_EV_NONE;
    int32_t sum    = (int32_t)norm_l + (int32_t)norm_r;

    p_t->b_black_l = is_black(p_t->b_black_l, norm_l);
    p_t->b_black_r = is_black(p_t->b_black_r, norm_r);
    p_t->b_white_l = is_white(p_t->b_white_l, norm_l);
    p_t->b_white_r = is_white(p_t->b_white_r, norm_r);
    if (p_t->since_junction_um < LINE_JUNCTION_REFRACT_UM)
    {
        p_t->since_junction_um += step_um;
    }

    /* Steering error: the sensors straddle the line edges, so on the line
       both read ~50 %; e > 0 means the line is under the left sensor.
       Plain difference (-1000..+1000), see LINE_KP in line_cfg.h. */
    p_t->prev_err = p_t->err;
    if (sum >= LINE_ERR_VALID_SUM)
    {
        p_t->err        = (int32_t)norm_l - (int32_t)norm_r;
        p_t->b_have_err = true;
    }

    if (p_t->b_black_l && p_t->b_black_r)
    {
        p_t->both_white_um = 0u;
        p_t->both_black_um += step_um;
        if (!p_t->b_junction && (p_t->both_black_um >= p_cfg->junction_um)
            && (p_t->straight_um >= LINE_STRAIGHT_UM)
            && (p_t->since_junction_um >= LINE_JUNCTION_REFRACT_UM))
        {
            p_t->b_junction        = true;
            p_t->since_junction_um = 0u;
            events |= (uint8_t)LINE_TRK_EV_JUNCTION;
        }
    }
    else if (p_t->b_white_l && p_t->b_white_r)
    {
        p_t->both_black_um = 0u;
        p_t->b_junction    = false;
        if (0u == p_t->both_white_um)
        {
            /* how the line was left: the strongest error of the last
               LINE_EXIT_PEAK_UM, not the last one, which fades as the line
               slides out past the outer sensor on a curve (2 Nov) */
            p_t->exit_err = p_t->peak_err;
        }
        p_t->both_white_um += step_um;
        if (!p_t->b_lost && (p_t->both_white_um >= p_cfg->lost_um))
        {
            p_t->b_lost = true;
            events |= (uint8_t)LINE_TRK_EV_LOST;
        }
    }
    else
    {
        p_t->both_black_um = 0u;
        p_t->both_white_um = 0u;
        p_t->b_junction    = false;
    }
    if (0u == p_t->both_white_um)
    {
        p_t->line_err = p_t->err;
        p_t->peak_um += step_um;
        if ((app_abs_i32(p_t->err) >= app_abs_i32(p_t->peak_err))
            || (p_t->peak_um > LINE_EXIT_PEAK_UM))
        {
            p_t->peak_err = p_t->err;
            p_t->peak_um  = 0u;
        }
    }

    if (p_t->b_lost && (!p_t->b_white_l || !p_t->b_white_r))
    {
        p_t->b_lost = false;
        events |= (uint8_t)LINE_TRK_EV_FOUND;
    }

    /* Output.  Hold the heading (omega = 0) across a junction and across a
       short white gap; stop when lost (MissionTask starts LINE_SEARCH). */
    if (p_t->b_lost)
    {
        p_t->v_mm_s = 0;
        p_t->omega  = 0;
    }
    else if ((p_t->both_white_um > 0u)
             && (app_abs_i32(p_t->exit_err) > LINE_EXIT_ERR))
    {
        /* drifted off one side: turn back toward it, slowly */
        p_t->v_mm_s = (p_cfg->cruise_mm_s * LINE_V_MIN_PCT) / 100;
        p_t->omega  = steer_toward(
            p_t->omega, (p_t->exit_err > 0)
                            ? (LINE_STEER_SIGN * LINE_RECOVER_OMEGA)
                            : (-LINE_STEER_SIGN * LINE_RECOVER_OMEGA));
    }
    else if (p_t->b_junction || (p_t->both_white_um > 0u) || !p_t->b_have_err)
    {
        /* junction or short gap: hold the heading */
        p_t->v_mm_s = p_cfg->cruise_mm_s;
        p_t->omega  = 0;
    }
    else
    {
        int32_t const derr = p_t->err - p_t->prev_err;
        int32_t const omega =
            LINE_STEER_SIGN
            * (((LINE_KP * p_t->err) / 1000) + (LINE_KD * derr));
        int32_t const slow  = (LINE_SLOW_K * app_abs_i32(p_t->err)) / 1000;
        int32_t       speed = (p_cfg->cruise_mm_s * (1000 - slow)) / 1000;

        speed = app_max_i32(speed, (p_cfg->cruise_mm_s * LINE_V_MIN_PCT) / 100);
        p_t->omega = steer_toward(
            p_t->omega, app_clamp_i32(omega, -LINE_OMEGA_MAX, LINE_OMEGA_MAX));
        p_t->v_mm_s = speed;
    }

    if (app_abs_i32(p_t->omega) >= LINE_STRAIGHT_OMEGA)
    {
        p_t->straight_um = 0u;
    }
    else if (p_t->straight_um < LINE_STRAIGHT_UM)
    {
        p_t->straight_um += step_um;
    }
    else
    {
        /* straight long enough */
    }
    return events;
}

/* ---- Turn slip (3 Nov) ---------------------------------------------- */
void line_slip_start (line_slip_t * p_s, int32_t heading_cdeg)
{
    p_s->n_samples = 0u;
    p_s->prev_cdeg = heading_cdeg;
    p_s->acc_cdeg  = 0;
}

void line_slip_sample (line_slip_t * p_s, int32_t heading_cdeg, uint16_t raw_l,
                       uint16_t raw_r)
{
    int32_t delta = heading_cdeg - p_s->prev_cdeg;

    while (delta > 18000)
    {
        delta -= 36000;
    }
    while (delta < -18000)
    {
        delta += 36000;
    }
    p_s->prev_cdeg = heading_cdeg;
    p_s->acc_cdeg += delta;
    if (p_s->n_samples < LINE_SLIP_MAX_BLOCKS)
    {
        p_s->h_ddeg[p_s->n_samples] =
            (int16_t)app_clamp_i32(app_abs_i32(p_s->acc_cdeg) / 10, 0, 32767);
        p_s->raw_l[p_s->n_samples] = raw_l;
        p_s->raw_r[p_s->n_samples] = raw_r;
        p_s->n_samples++;
    }
}

/* One sensor: the norm-weighted centre (0.1 deg) of its first dark run
   after LINE_SLIP_SKIP_DDEG that lasts LINE_SLIP_MIN_BLOCKS; -1 if none.
   *p_runs counts the dark runs seen. */
static int32_t slip_centre (line_slip_t const * p_s, line_cal_t const * p_cal,
                            uint8_t chan, uint8_t * p_runs)
{
    int64_t  sum_weight = 0;
    int64_t  swh        = 0;
    uint16_t len        = 0u;
    int32_t  found      = -1;
    uint16_t idx;

    *p_runs = 0u;
    for (idx = 0u; idx <= p_s->n_samples; idx++)
    {
        uint16_t const raw =
            (idx < p_s->n_samples)
                ? ((LINE_CH_L == chan) ? p_s->raw_l[idx] : p_s->raw_r[idx])
                : 0u;
        uint16_t const norm =
            (idx < p_s->n_samples) ? line_cal_norm(p_cal, chan, raw) : 0u;
        bool const b_dark = (idx < p_s->n_samples)
                            && (p_s->h_ddeg[idx] >= LINE_SLIP_SKIP_DDEG)
                            && (norm >= LINE_SLIP_DARK);

        if (b_dark)
        {
            sum_weight += norm;
            swh += (int64_t)norm * p_s->h_ddeg[idx];
            len++;
        }
        else if (len != 0u)
        {
            if (len >= LINE_SLIP_MIN_BLOCKS)
            {
                (*p_runs)++;
                if ((found < 0) && (sum_weight > 0))
                {
                    found = (int32_t)(swh / sum_weight);
                }
            }
            sum_weight = 0;
            swh        = 0;
            len        = 0u;
        }
        else
        {
            /* white */
        }
    }
    return found;
}

/* The two sensors sit either side of the car's centreline, so the mean of
   their next crossings is where the centreline is on the line again: 180
   real deg from the start (the car starts centred on it).  The sensors
   need not see the line at the same time. */
uint16_t line_slip_gain (line_slip_t const * p_s, line_cal_t const * p_cal,
                         uint32_t * p_diag)
{
    uint8_t       runs_l;
    uint8_t       runs_r;
    int32_t const c_l =
        p_cal->b_valid ? slip_centre(p_s, p_cal, LINE_CH_L, &runs_l) : -1;
    int32_t const c_r =
        p_cal->b_valid ? slip_centre(p_s, p_cal, LINE_CH_R, &runs_r) : -1;
    int32_t const tot =
        (p_s->n_samples > 0u) ? (int32_t)p_s->h_ddeg[p_s->n_samples - 1u] : 0;
    int32_t gain;

    if (!p_cal->b_valid)
    {
        runs_l = 0u;
        runs_r = 0u;
    }
    /* diagnosis: dark runs L << 24 | R << 16 | encoder deg spun */
    *p_diag = ((uint32_t)runs_l << 24) | ((uint32_t)runs_r << 16)
              | (uint32_t)app_clamp_i32(tot / 10, 0, 0xFFFF);
    if ((c_l < 0) || (c_r < 0))
    {
        return 0u;
    }
    gain = ((((c_l + c_r) / 2) * 1000) + 900) / 1800;
    return ((gain >= LINE_SLIP_GAIN_MIN_PM) && (gain <= LINE_SLIP_GAIN_MAX_PM))
               ? (uint16_t)gain
               : 0u;
}

/* ---------------------------------------------------------------------
 *  Change Requirement 1: data representation of IF-04 (line snapshot).
 * --------------------------------------------------------------------- */
static pk_field_t const g_line_layout[] = {
    PK_FIELD(line_snap_t, t_ms, 32u, PK_W),
    PK_FIELD(line_snap_t, blocks, 32u, PK_W), /* counter since boot */
    PK_FIELD(line_snap_t, missed_blocks, 16u,
             PK_W), /* counter, readers take differences */
    PK_FIELD(line_snap_t, err, 11u, PK_S), /* -1000..1000                     */
    PK_FIELD(line_snap_t, omega_mrad_s, 12u,
             PK_S), /* +-2047 (LINE_OMEGA_MAX 2000)    */
    PK_FIELD(line_snap_t, raw_l, 10u, PK_U), /* 0..1000 */
    PK_FIELD(line_snap_t, raw_r, 10u, PK_U),
    PK_FIELD(line_snap_t, raw_bc, 10u, PK_U),
    PK_FIELD(line_snap_t, barcode_ok, 16u, PK_W), /* counters */
    PK_FIELD(line_snap_t, barcode_err, 16u, PK_W),
    PK_FIELD(line_snap_t, contrast_x10, 10u, PK_U), /* saturates at 102.3 : 1 */
    PK_FIELD(line_snap_t, flags, 7u, PK_U),         /* LINE_FLAG_* (7 bits) */
    PK_FIELD(line_snap_t, bc_state, 2u, PK_U),      /* 4 states */
    PK_FIELD(line_snap_t, last_barcode, 6u,
             PK_SYM), /* Code 39 symbol or none          */
    PK_FIELD(line_snap_t, last_reject, 4u, PK_U),   /* code39_result_t 0..8 */
    PK_FIELD(line_snap_t, turn_gain_pm, 11u, PK_U), /* 0..2047 permille */
    PK_FIELD(line_snap_t, sens, 3u, PK_U) /* LINE_SENS_* (3 bits)            */
};

#define LINE_FIELDS \
    ((uint16_t)(sizeof(g_line_layout) / sizeof(g_line_layout[0])))

void line_snap_pack (line_snap_t const * p_snap, uint32_t * p_words)
{
    pk_pack_rec(g_line_layout, LINE_FIELDS, p_snap, p_words, LINE_SNAP_WORDS);
}

void line_snap_unpack (uint32_t const * p_words, line_snap_t * p_snap)
{
    pk_unpack_rec(g_line_layout, LINE_FIELDS, p_words, p_snap);
}

uint32_t line_snap_layout_bits (void)
{
    return pk_layout_bits(g_line_layout, LINE_FIELDS);
}

/*** end of file ***/
