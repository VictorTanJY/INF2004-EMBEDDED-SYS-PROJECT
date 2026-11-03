/*
 *  line_track.c
 *  Subsystem 3 -- calibration and line tracking.  See line_track.h.
 */

#include "line_track.h"
#include "../app_types.h"

/* ------------------------------------------------------------------ */
void line_cal_start(line_cal_t * p_cal)
{
    uint8_t ch;

    for (ch = 0u; ch < LINE_CH_COUNT; ch++)
    {
        p_cal->min[ch] = 0xFFFFu;
        p_cal->max[ch] = 0u;
    }
}

void line_cal_sample(line_cal_t * p_cal, uint16_t const raw[LINE_CH_COUNT])
{
    uint8_t ch;

    for (ch = 0u; ch < LINE_CH_COUNT; ch++)
    {
        if (raw[ch] < p_cal->min[ch])
        {
            p_cal->min[ch] = raw[ch];
        }
        if (raw[ch] > p_cal->max[ch])
        {
            p_cal->max[ch] = raw[ch];
        }
    }
}

bool line_cal_finish(line_cal_t * p_cal)
{
    uint16_t white[LINE_CH_COUNT];
    uint16_t black[LINE_CH_COUNT];
    uint16_t worst = 0xFFFFu;
    bool     b_ok  = true;
    uint8_t  ch;

    for (ch = 0u; ch < LINE_CH_COUNT; ch++)
    {
        uint32_t lo = p_cal->min[ch];
        uint32_t hi = p_cal->max[ch];
        uint32_t contrast;

        if (ch == LINE_CH_BC)
        {
            /* Barcode sensor: fixed levels (line_cfg.h), never judged by
               the spin.  min/max are still kept for LOG 34.            */
            white[ch] = (uint16_t)LINE_BC_WHITE_RAW;
            black[ch] = (uint16_t)LINE_BC_BLACK_RAW;
            continue;
        }
        if ((lo > hi) || ((hi - lo) < LINE_CAL_MIN_SPAN))
        {
            b_ok = false;               /* never saw both black and white */
            contrast = 0u;
        }
        else
        {
            /* AO ratio of the two levels, x 10 (3:1 = 30) */
            contrast = (lo == 0u) ? 999u : ((hi * 10u) / lo);
        }
#if LINE_BLACK_IS_HIGH
        white[ch] = (uint16_t)lo;
        black[ch] = (uint16_t)hi;
#else
        white[ch] = (uint16_t)hi;
        black[ch] = (uint16_t)lo;
#endif
        if (contrast < worst)
        {
            worst = (uint16_t)contrast;
        }
    }
    if (worst < LINE_CONTRAST_MIN_X10)
    {
        b_ok = false;
    }
    p_cal->contrast_x10 = worst;
    if (b_ok)
    {
        for (ch = 0u; ch < LINE_CH_COUNT; ch++)
        {
            p_cal->white[ch] = white[ch];
            p_cal->black[ch] = black[ch];
        }
        p_cal->b_valid = true;
    }
    return b_ok;
}

uint16_t line_cal_norm(line_cal_t const * p_cal, uint8_t ch, uint16_t raw)
{
    int32_t const w = (int32_t)p_cal->white[ch];
    int32_t const b = (int32_t)p_cal->black[ch];
    int32_t       n;

    if (!p_cal->b_valid || (w == b))
    {
        return 0u;
    }
    n = (((int32_t)raw - w) * 1000) / (b - w);
    return (uint16_t)app_clamp_i32(n, 0, 1000);
}

/* ------------------------------------------------------------------ */
void line_track_init(line_track_t * p_t)
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
    p_t->v                 = 0;
    p_t->both_black_um     = 0u;
    p_t->both_white_um     = 0u;
    p_t->straight_um       = 0u;
    p_t->since_junction_um = LINE_JUNCTION_REFRACT_UM;
}

/* Steering ramp: move the turn command at most LINE_OMEGA_STEP per block
   (10 ms), so a flip of the line error does not jump the wheel speeds. */
static int32_t steer_toward(int32_t now, int32_t target)
{
    return app_clamp_i32(target, now - LINE_OMEGA_STEP, now + LINE_OMEGA_STEP);
}

static bool is_black(bool b_black, uint16_t norm)
{
    return b_black ? (norm >= LINE_BLACK_OFF) : (norm > LINE_BLACK_ON);
}

static bool is_white(bool b_white, uint16_t norm)
{
    return b_white ? (norm <= LINE_WHITE_OFF) : (norm < LINE_WHITE_ON);
}

uint8_t line_track_step(line_track_t * p_t, line_trk_cfg_t const * p_cfg,
                        uint16_t norm_l, uint16_t norm_r, uint32_t step_um)
{
    uint8_t ev  = (uint8_t)LINE_TRK_EV_NONE;
    int32_t sum = (int32_t)norm_l + (int32_t)norm_r;

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
        if (!p_t->b_junction && (p_t->both_black_um >= p_cfg->junction_um) &&
            (p_t->straight_um >= LINE_STRAIGHT_UM) &&
            (p_t->since_junction_um >= LINE_JUNCTION_REFRACT_UM))
        {
            p_t->b_junction        = true;
            p_t->since_junction_um = 0u;
            ev |= (uint8_t)LINE_TRK_EV_JUNCTION;
        }
    }
    else if (p_t->b_white_l && p_t->b_white_r)
    {
        p_t->both_black_um = 0u;
        p_t->b_junction    = false;
        if (p_t->both_white_um == 0u)
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
            ev |= (uint8_t)LINE_TRK_EV_LOST;
        }
    }
    else
    {
        p_t->both_black_um = 0u;
        p_t->both_white_um = 0u;
        p_t->b_junction    = false;
    }
    if (p_t->both_white_um == 0u)
    {
        p_t->line_err = p_t->err;
        p_t->peak_um += step_um;
        if ((app_abs_i32(p_t->err) >= app_abs_i32(p_t->peak_err)) ||
            (p_t->peak_um > LINE_EXIT_PEAK_UM))
        {
            p_t->peak_err = p_t->err;
            p_t->peak_um  = 0u;
        }
    }

    if (p_t->b_lost && (!p_t->b_white_l || !p_t->b_white_r))
    {
        p_t->b_lost = false;
        ev |= (uint8_t)LINE_TRK_EV_FOUND;
    }

    /* Output.  Hold the heading (omega = 0) across a junction and across a
       short white gap; stop when lost (MissionTask starts LINE_SEARCH). */
    if (p_t->b_lost)
    {
        p_t->v     = 0;
        p_t->omega = 0;
    }
    else if ((p_t->both_white_um > 0u) && (app_abs_i32(p_t->exit_err) > LINE_EXIT_ERR))
    {
        /* drifted off one side: turn back toward it, slowly */
        p_t->v     = (p_cfg->cruise_mm_s * LINE_V_MIN_PCT) / 100;
        p_t->omega = steer_toward(p_t->omega,
                                  (p_t->exit_err > 0) ? (LINE_STEER_SIGN * LINE_RECOVER_OMEGA)
                                                      : (-LINE_STEER_SIGN * LINE_RECOVER_OMEGA));
    }
    else if (p_t->b_junction || (p_t->both_white_um > 0u) || !p_t->b_have_err)
    {
        /* junction or short gap: hold the heading */
        p_t->v     = p_cfg->cruise_mm_s;
        p_t->omega = 0;
    }
    else
    {
        int32_t const derr  = p_t->err - p_t->prev_err;
        int32_t const omega = LINE_STEER_SIGN * (((LINE_KP * p_t->err) / 1000) + (LINE_KD * derr));
        int32_t const slow  = (LINE_SLOW_K * app_abs_i32(p_t->err)) / 1000;
        int32_t       v     = (p_cfg->cruise_mm_s * (1000 - slow)) / 1000;

        v = app_max_i32(v, (p_cfg->cruise_mm_s * LINE_V_MIN_PCT) / 100);
        p_t->omega = steer_toward(p_t->omega,
                                  app_clamp_i32(omega, -LINE_OMEGA_MAX, LINE_OMEGA_MAX));
        p_t->v     = v;
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
    return ev;
}

/* ---- Turn slip (3 Nov) ---------------------------------------------- */
void line_slip_start(line_slip_t * p_s, int32_t heading_cdeg)
{
    p_s->n         = 0u;
    p_s->prev_cdeg = heading_cdeg;
    p_s->acc_cdeg  = 0;
}

void line_slip_sample(line_slip_t * p_s, int32_t heading_cdeg, uint16_t raw_l, uint16_t raw_r)
{
    int32_t d = heading_cdeg - p_s->prev_cdeg;

    while (d > 18000)
    {
        d -= 36000;
    }
    while (d < -18000)
    {
        d += 36000;
    }
    p_s->prev_cdeg  = heading_cdeg;
    p_s->acc_cdeg  += d;
    if (p_s->n < LINE_SLIP_MAX_BLOCKS)
    {
        p_s->h_ddeg[p_s->n] = (int16_t)app_clamp_i32(app_abs_i32(p_s->acc_cdeg) / 10, 0, 32767);
        p_s->raw_l[p_s->n]  = raw_l;
        p_s->raw_r[p_s->n]  = raw_r;
        p_s->n++;
    }
}

/* Heading (0.1 deg) where L - R changes sign between the two sensors'
   darkest blocks lo..hi: there the pair is centred on the line. */
static int32_t slip_cross(line_slip_t const * p_s, line_cal_t const * p_cal, uint16_t lo, uint16_t hi)
{
    uint16_t k;

    for (k = lo; k < hi; k++)
    {
        int32_t const d0 = (int32_t)line_cal_norm(p_cal, LINE_CH_L, p_s->raw_l[k]) -
                           (int32_t)line_cal_norm(p_cal, LINE_CH_R, p_s->raw_r[k]);
        int32_t const d1 = (int32_t)line_cal_norm(p_cal, LINE_CH_L, p_s->raw_l[k + 1u]) -
                           (int32_t)line_cal_norm(p_cal, LINE_CH_R, p_s->raw_r[k + 1u]);

        if (d0 == 0)
        {
            return p_s->h_ddeg[k];
        }
        if ((d0 > 0) != (d1 > 0))
        {
            return p_s->h_ddeg[k] + (((int32_t)(p_s->h_ddeg[k + 1u] - p_s->h_ddeg[k]) * d0) / (d0 - d1));
        }
    }
    return ((int32_t)p_s->h_ddeg[lo] + (int32_t)p_s->h_ddeg[hi]) / 2;
}

uint16_t line_slip_gain(line_slip_t const * p_s, line_cal_t const * p_cal)
{
    bool     b_in  = false;
    uint16_t pk_l  = 0u;
    uint16_t pk_r  = 0u;
    uint16_t i_l   = 0u;
    uint16_t i_r   = 0u;
    uint16_t i_in  = 0u;
    uint16_t i;

    if (!p_cal->b_valid)
    {
        return 0u;
    }
    for (i = 0u; i < p_s->n; i++)
    {
        uint16_t const nl     = line_cal_norm(p_cal, LINE_CH_L, p_s->raw_l[i]);
        uint16_t const nr     = line_cal_norm(p_cal, LINE_CH_R, p_s->raw_r[i]);
        bool const     b_dark = (p_s->h_ddeg[i] >= LINE_SLIP_SKIP_DDEG) &&
                                ((nl >= LINE_SLIP_DARK) || (nr >= LINE_SLIP_DARK));

        if (b_dark)
        {
            if (!b_in)
            {
                b_in = true;
                i_in = i;
                pk_l = 0u;
                pk_r = 0u;
            }
            if (nl > pk_l)
            {
                pk_l = nl;
                i_l  = i;
            }
            if (nr > pk_r)
            {
                pk_r = nr;
                i_r  = i;
            }
        }
        if (b_in && (!b_dark || (i == (uint16_t)(p_s->n - 1u))))
        {
            b_in = false;
            if ((pk_l >= LINE_SLIP_DARK) && (pk_r >= LINE_SLIP_DARK) &&
                ((uint16_t)(i - i_in) >= LINE_SLIP_MIN_BLOCKS))
            {
                /* both sensors crossed it: the line, 180 real deg on */
                int32_t const h    = slip_cross(p_s, p_cal, (i_l < i_r) ? i_l : i_r, (i_l < i_r) ? i_r : i_l);
                int32_t const gain = ((h * 1000) + 900) / 1800;

                return ((gain >= LINE_SLIP_GAIN_MIN_PM) && (gain <= LINE_SLIP_GAIN_MAX_PM)) ? (uint16_t)gain : 0u;
            }
        }
    }
    return 0u;
}
