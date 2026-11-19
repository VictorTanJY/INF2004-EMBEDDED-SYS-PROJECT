/** @file imu_core.c
 *
 *  @brief Subsystem 4 -- ImuTask logic (pure).  See imu_core.h.  Owner:
 * Buddy 4.
 */

#include <stddef.h>
#include "imu_core.h"
#include "../app_pack.h"
#include "imu_cfg.h"
#include "../app_types.h"
#include "../app_math.h"

#define IMU_HORIZ_AXES 2u /* forward and left: the horizontal plane */

#define MG_PER_G_MM_S2 9807 /* 1 g = 9807 mm/s^2 */

/* Cast (uint16_t)(sizeof ...): the layout table has fewer than 64 entries. */
#define IMU_FIELDS ((uint16_t)(sizeof(g_imu_layout) / sizeof(g_imu_layout[0])))

/* ---------------------------------------------------------------------
 *  Change Requirement 1: data representation of IF-08 (IMU snapshot).
 * --------------------------------------------------------------------- */
static app_pack_field_t const g_imu_layout[] = {
    {offsetof(imu_types_snap_t, t_ms), sizeof(((imu_types_snap_t){0}).t_ms),
     32u, PK_W},
    {offsetof(imu_types_snap_t, pitch_cdeg),
     sizeof(((imu_types_snap_t){0}).pitch_cdeg), 15u,
     PK_S}, /* +-163.83 deg (pitch is +-90)  */
    {offsetof(imu_types_snap_t, roll_cdeg),
     sizeof(((imu_types_snap_t){0}).roll_cdeg), 16u,
     PK_S}, /* +-327.67 deg (roll is +-180)  */
    {offsetof(imu_types_snap_t, heading_cdeg),
     sizeof(((imu_types_snap_t){0}).heading_cdeg), 16u, PK_U}, /* 0..35999 */
    {offsetof(imu_types_snap_t, turn_rate_cdeg_s),
     sizeof(((imu_types_snap_t){0}).turn_rate_cdeg_s), 16u,
     PK_S}, /* the int16_t range, +-327 deg/s */
    {offsetof(imu_types_snap_t, accel_mg),
     sizeof(((imu_types_snap_t){0}).accel_mg), 15u,
     PK_S}, /* +-16 g (sensor range) */
    {offsetof(imu_types_snap_t, hump_max_mm),
     sizeof(((imu_types_snap_t){0}).hump_max_mm), 10u, PK_U}, /* 0..1023 mm */
    {offsetof(imu_types_snap_t, last_hump_mm),
     sizeof(((imu_types_snap_t){0}).last_hump_mm), 10u, PK_U},
    {offsetof(imu_types_snap_t, humps), sizeof(((imu_types_snap_t){0}).humps),
     8u, PK_U}, /* per run, saturates at 255 */
    {offsetof(imu_types_snap_t, i2c_errors),
     sizeof(((imu_types_snap_t){0}).i2c_errors), 16u, PK_W}, /* counter */
    {offsetof(imu_types_snap_t, hump_state),
     sizeof(((imu_types_snap_t){0}).hump_state), 2u, PK_U}, /* 4 states */
    {offsetof(imu_types_snap_t, motion), sizeof(((imu_types_snap_t){0}).motion),
     3u, PK_U}, /* 8 classes */
    {offsetof(imu_types_snap_t, b_ok), sizeof(((imu_types_snap_t){0}).b_ok), 1u,
     PK_U},
    {offsetof(imu_types_snap_t, cal), sizeof(((imu_types_snap_t){0}).cal), 2u,
     PK_U}, /* IMU_CAL_LEVEL | IMU_CAL_MAG */
    {offsetof(imu_types_snap_t, pitch_valid),
     sizeof(((imu_types_snap_t){0}).pitch_valid), 1u, PK_U},
    {offsetof(imu_types_snap_t, tilt), sizeof(((imu_types_snap_t){0}).tilt), 1u,
     PK_U}};

/* ---- hump results while the floor may have gone (14 Nov) ----------------- */
static void    hump_reset(imu_core_t * p_ic);
static void    side_reset(imu_core_t * p_ic);
static int32_t axis(int32_t const vec[3], int32_t axis_no, int32_t sign);
static void    integrate(imu_core_t * p_ic, int32_t theta, uint32_t ds_um);
static uint8_t hump_step(imu_core_t * p_ic, uint32_t ds_um);
static void    side_record(imu_core_t * p_ic, uint16_t height_mm);
static uint8_t side_step(imu_core_t * p_ic, uint32_t ds_um);
static uint8_t lift_step(imu_core_t * p_ic, int32_t up_mg, int32_t v_mm_s,
                         uint32_t now_ms);
static uint8_t classify(imu_core_t const * p_ic, int32_t v_mm_s,
                        int32_t yaw_cdeg_s);
static void    acc_filter(imu_core_t * p_ic, int32_t const acc_mg[IMU_AXES],
                          int32_t accel[IMU_AXES]);
static int32_t attitude(imu_core_t * p_ic, int32_t const accel[IMU_AXES]);
static void heading_update(imu_core_t * p_ic, int16_t const mag_raw[IMU_AXES]);
static uint8_t tilt_step(imu_core_t * p_ic, uint32_t now_ms);

bool imu_core_cal_mag_finish (imu_core_t * p_ic)
{
    int32_t const axes[IMU_HORIZ_AXES] = {IMU_AXIS_FWD,
                                          IMU_AXIS_LEFT}; /* horizontal plane */
    uint8_t       kdx;

    for (kdx = 0u; kdx < IMU_HORIZ_AXES; kdx++)
    {
        /* Cast (int32_t)p_ic->mag_max[axes[kdx]]: every int16_t value fits. */
        if (((int32_t)p_ic->mag_max[axes[kdx]] - p_ic->mag_min[axes[kdx]])
            < IMU_MAG_MIN_SPAN)
        {
            return false;
        }
    }
    for (kdx = 0u; kdx < IMU_AXES; kdx++)
    {
        /* Casts (int16_t)(((int32_t)p_ic->mag_min[k ...)): the value is
           -32768..32767; (int32_t)p_ic->mag_min[kdx]: every int16_t value fits.
         */
        p_ic->mag_off[kdx] =
            (int16_t)(((int32_t)p_ic->mag_min[kdx] + p_ic->mag_max[kdx]) / 2);
    }
    p_ic->cal |= IMU_CAL_MAG;
    return true;
}

uint8_t imu_core_step (imu_core_t * p_ic, int32_t const acc_mg[3],
                       int16_t const * p_mag, int32_t ds_um, int32_t v_mm_s,
                       int32_t yaw_cdeg_s, uint32_t dt_ms, uint32_t now_ms)
{
    uint8_t events = 0u;
    int32_t accel[IMU_AXES];
    int32_t up_mg;
    uint8_t cls;

    if (0u == dt_ms)
    {
        dt_ms = 10u;
    }

    acc_filter(p_ic, acc_mg, accel);

    /* Longitudinal acceleration from the encoders, filtered. */
    if (p_ic->b_have_v)
    {
        /* Cast (int32_t)dt_ms: dt_ms is the IMU loop period (0 replaced by 10
           above); a gap of 2^31 ms (24 days) cannot occur. */
        int32_t const raw = ((v_mm_s - p_ic->prev_v) * 1000) / (int32_t)dt_ms;

        p_ic->accel_mm_s2 += (raw - p_ic->accel_mm_s2) / 4;
    }
    p_ic->prev_v   = v_mm_s;
    p_ic->b_have_v = true;

    up_mg = attitude(p_ic, accel);
    if (p_mag != NULL)
    {
        heading_update(p_ic, p_mag);
    }

    events |= tilt_step(p_ic, now_ms);

    /* Hump detection runs on forward travel only (lifting the robot by
       hand with the wheels still is never a hump).  A pitch hump (both
       wheels) takes precedence; otherwise the roll detector looks for a
       hump under one wheel. */
    if (ds_um > 0)
    {
        /* Cast (uint32_t)ds_um: ds_um > 0 here (tested above). */
        events |= hump_step(p_ic, (uint32_t)ds_um);
        if ((IMU_HUMP_FLAT == p_ic->hump_state)
            && (0u == (events & IMU_EV_HUMP_START)))
        {
            /* Cast (uint32_t)ds_um: ds_um > 0 here (tested above). */
            events |= side_step(p_ic, (uint32_t)ds_um);
        }
        else
        {
            side_reset(p_ic);
        }
    }

    events |= lift_step(p_ic, up_mg, v_mm_s, now_ms);

    cls = classify(p_ic, v_mm_s, yaw_cdeg_s);
    if ((cls != p_ic->motion)
        && (app_types_elapsed_ms(now_ms, p_ic->motion_ms) >= IMU_CLASS_HOLD_MS))
    {
        p_ic->motion    = cls;
        p_ic->motion_ms = now_ms;
        events |= IMU_EV_MOTION;
    }
    return events;
}

void imu_core_init (imu_core_t * p_ic)
{
    /* Cast (uint8_t *)p_ic: byte view of the struct for clearing; no value
       converted. */
    uint8_t * p_b = (uint8_t *)p_ic;
    size_t    idx;

    for (idx = 0u; idx < sizeof(*p_ic); idx++)
    {
        p_b[idx] = 0u;
    }
    p_ic->hump_state = IMU_HUMP_FLAT;
    p_ic->motion     = IMU_MOTION_STATIONARY;
}

void imu_core_new_run (imu_core_t * p_ic)
{
    p_ic->hump_max_mm    = 0u;
    p_ic->last_hump_mm   = 0u;
    p_ic->humps          = 0u;
    p_ic->side_state     = 0u;
    p_ic->side_um        = 0u;
    p_ic->side_peak_cdeg = 0;
    p_ic->rise_peak_mg   = 0;
    p_ic->rise_max_mg    = 0;
    p_ic->b_hump_saved   = false;
}

void imu_core_hump_save (imu_core_t * p_ic)
{
    p_ic->saved_hump_max_mm  = p_ic->hump_max_mm;
    p_ic->saved_last_hump_mm = p_ic->last_hump_mm;
    p_ic->saved_humps        = p_ic->humps;
    p_ic->b_hump_saved       = true;
}

void imu_core_hump_drop (imu_core_t * p_ic)
{
    p_ic->b_hump_saved = false;
}

void imu_core_hump_restore (imu_core_t * p_ic)
{
    if (p_ic->b_hump_saved)
    {
        p_ic->hump_max_mm  = p_ic->saved_hump_max_mm;
        p_ic->last_hump_mm = p_ic->saved_last_hump_mm;
        p_ic->humps        = p_ic->saved_humps;
        p_ic->b_hump_saved = false;
    }
    hump_reset(p_ic);
    side_reset(p_ic);
}

/* ---- calibration ------------------------------------------------------ */
void imu_core_cal_level_start (imu_core_t * p_ic)
{
    uint8_t kdx;

    for (kdx = 0u; kdx < IMU_AXES; kdx++)
    {
        p_ic->lvl_sum[kdx] = 0;
        p_ic->lvl_min[kdx] = 0x7FFFFFFF;
        p_ic->lvl_max[kdx] = -0x7FFFFFFF;
    }
    p_ic->lvl_n = 0u;
}

void imu_core_cal_level_sample (imu_core_t * p_ic, int32_t const acc_mg[3])
{
    uint8_t kdx;

    if (0xFFFFu == p_ic->lvl_n)
    {
        return;
    }
    for (kdx = 0u; kdx < IMU_AXES; kdx++)
    {
        p_ic->lvl_sum[kdx] += acc_mg[kdx];
        p_ic->lvl_min[kdx] = app_types_min_i32(p_ic->lvl_min[kdx], acc_mg[kdx]);
        p_ic->lvl_max[kdx] = app_types_max_i32(p_ic->lvl_max[kdx], acc_mg[kdx]);
    }
    p_ic->lvl_n++;
}

bool imu_core_cal_level_finish (imu_core_t * p_ic)
{
    int32_t mean[3];
    int32_t fwd;
    int32_t left;
    int32_t up_mg;
    uint8_t kdx;

    /* Diagnostics for LOG 57-59 (why a calibration failed). */
    p_ic->lvl_spread_mg = 0;
    p_ic->lvl_up_mg     = 0;
    if (p_ic->lvl_n < 50u)
    {
        return false;
    }
    for (kdx = 0u; kdx < IMU_AXES; kdx++)
    {
        /* Casts (int32_t)(p_ic->lvl_sum[kdx] / ...): a mean of readings of the
           +-4 g sensor (|axis| <= 3998 mg); (int64_t)p_ic->lvl_n: every
           uint16_t value fits. */
        p_ic->lvl_spread_mg = app_types_max_i32(
            p_ic->lvl_spread_mg, p_ic->lvl_max[kdx] - p_ic->lvl_min[kdx]);
        mean[kdx] = (int32_t)(p_ic->lvl_sum[kdx] / (int64_t)p_ic->lvl_n);
    }
    fwd             = axis(mean, IMU_AXIS_FWD, IMU_SIGN_FWD);
    left            = axis(mean, IMU_AXIS_LEFT, IMU_SIGN_LEFT);
    up_mg           = axis(mean, IMU_AXIS_UP, IMU_SIGN_UP);
    p_ic->lvl_up_mg = up_mg;
    if (p_ic->lvl_spread_mg > IMU_LEVEL_MAX_SPREAD_MG)
    {
        return false; /* the robot was moving */
    }
    if (up_mg < 500)
    {
        return false; /* not right way up (or axes mis-set) */
    }
    if (up_mg > IMU_LEVEL_UP_MAX_MG)
    {
        return false; /* stuck / saturated output (2 Nov) */
    }
    /* Casts (int32_t)app_math_isqrt64(...): the root of two squares of +-4 g
       readings (|axis| <= 3998 mg): below 5655;
       (uint64_t)(((int64_t)left * left) + ...): a sum of squares is not
       negative; (int64_t)left, (int64_t)up_mg: every int32_t value fits. */
    p_ic->pitch0_cdeg = app_math_atan2_cdeg(
        fwd, (int32_t)app_math_isqrt64((uint64_t)(((int64_t)left * left)
                                                  + ((int64_t)up_mg * up_mg))));
    p_ic->roll0_cdeg = app_math_atan2_cdeg(left, up_mg);
    p_ic->cal |= IMU_CAL_LEVEL;
    return true;
}

void imu_core_cal_mag_start (imu_core_t * p_ic)
{
    uint8_t kdx;

    for (kdx = 0u; kdx < IMU_AXES; kdx++)
    {
        p_ic->mag_min[kdx] = 32767;
        p_ic->mag_max[kdx] = -32768;
    }
}

void imu_core_cal_mag_sample (imu_core_t * p_ic, int16_t const mag[3])
{
    uint8_t kdx;

    for (kdx = 0u; kdx < IMU_AXES; kdx++)
    {
        if (mag[kdx] < p_ic->mag_min[kdx])
        {
            p_ic->mag_min[kdx] = mag[kdx];
        }
        if (mag[kdx] > p_ic->mag_max[kdx])
        {
            p_ic->mag_max[kdx] = mag[kdx];
        }
    }
}

void imu_core_snap_pack (imu_types_snap_t const * p_snap, uint32_t * p_words)
{
    app_pack_pack_rec(g_imu_layout, IMU_FIELDS, p_snap, p_words,
                      IMU_SNAP_WORDS);
}

void imu_core_snap_unpack (uint32_t const * p_words, imu_types_snap_t * p_snap)
{
    p_snap->reserved[0] = 0u;
    p_snap->reserved[1] = 0u;
    app_pack_unpack_rec(g_imu_layout, IMU_FIELDS, p_words, p_snap);
}

uint32_t imu_core_snap_layout_bits (void)
{
    return app_pack_layout_bits(g_imu_layout, IMU_FIELDS);
}

/** Low-pass filter (alpha 1/4), mg x 16; accel = the filtered mg.  |a| of
   the raw sample for the snapshot. */
static void acc_filter (imu_core_t * p_ic, int32_t const acc_mg[IMU_AXES],
                        int32_t accel[IMU_AXES])
{
    int32_t mag_mg;
    uint8_t kdx;

    for (kdx = 0u; kdx < IMU_AXES; kdx++)
    {
        if (!p_ic->b_have_acc)
        {
            p_ic->acc_f[kdx] = acc_mg[kdx] * 16;
        }
        p_ic->acc_f[kdx] +=
            ((acc_mg[kdx] * 16) - p_ic->acc_f[kdx]) / IMU_ACC_LPF_DIV;
        accel[kdx] = p_ic->acc_f[kdx] / 16;
    }

    /* |a| for the snapshot (14 Nov: no longer an impact check). */
    /* Casts (int32_t)app_math_isqrt64(...): the root of three squares of +-4 g
       readings (|axis| <= 3998 mg): below 6925;
       (uint64_t)(((int64_t)acc_mg[0] * ...)): a sum of squares is not negative;
       (int64_t)acc_mg[0], (int64_t)acc_mg[1], (int64_t)acc_mg[2]: every int32_t
       value fits. */
    mag_mg = (int32_t)app_math_isqrt64(
        (uint64_t)(((int64_t)acc_mg[0] * acc_mg[0])
                   + ((int64_t)acc_mg[1] * acc_mg[1])
                   + ((int64_t)acc_mg[2] * acc_mg[2])));
    p_ic->prev_mag_mg = mag_mg;
    p_ic->b_have_acc  = true;
}

/** Attitude.  The forward axis also sees the robot's own acceleration, so
   the encoder estimate is subtracted before taking the angle.  Returns the
   up-axis acceleration, mg. */
static int32_t attitude (imu_core_t * p_ic, int32_t const accel[IMU_AXES])
{
    int32_t fwd;
    int32_t left;
    int32_t up_mg;

    /* Casts (int32_t)app_math_isqrt64(...): the root of two squares of +-4 g
       readings (|axis| <= 3998 mg): below 5655;
       (uint64_t)(((int64_t)left * left) + ...): a sum of squares is not
       negative; (int64_t)left, (int64_t)up_mg: every int32_t value fits. */
    fwd   = axis(accel, IMU_AXIS_FWD, IMU_SIGN_FWD)
            - ((app_types_clamp_i32(p_ic->accel_mm_s2, -IMU_ACC_COMP_MAX_MM_S2,
                                    IMU_ACC_COMP_MAX_MM_S2)
                * 1000)
               / MG_PER_G_MM_S2);
    left  = axis(accel, IMU_AXIS_LEFT, IMU_SIGN_LEFT);
    up_mg = axis(accel, IMU_AXIS_UP, IMU_SIGN_UP);
    p_ic->pitch_cdeg = app_math_wrap_cdeg(
        app_math_atan2_cdeg(fwd, (int32_t)app_math_isqrt64(
                                     (uint64_t)(((int64_t)left * left)
                                                + ((int64_t)up_mg * up_mg))))
        - p_ic->pitch0_cdeg);
    p_ic->roll_cdeg =
        app_math_wrap_cdeg(app_math_atan2_cdeg(left, up_mg) - p_ic->roll0_cdeg);
    return up_mg;
}

/** Magnetic heading, clockwise from north, once the hard-iron offsets are
   known. */
static void heading_update (imu_core_t * p_ic, int16_t const mag_raw[IMU_AXES])
{
    if ((p_ic->cal & IMU_CAL_MAG) != 0u)
    {
        /* Casts (int32_t)mag_raw[0], (int32_t)mag_raw[1], (int32_t)mag_raw[2]:
           every int16_t value fits. */
        int32_t const mag[3] = {(int32_t)mag_raw[0] - p_ic->mag_off[0],
                                (int32_t)mag_raw[1] - p_ic->mag_off[1],
                                (int32_t)mag_raw[2] - p_ic->mag_off[2]};
        int32_t       heading =
            app_math_atan2_cdeg(axis(mag, IMU_AXIS_LEFT, IMU_SIGN_LEFT),
                                axis(mag, IMU_AXIS_FWD, IMU_SIGN_FWD));

        p_ic->heading_cdeg = (heading < 0) ? (heading + 36000)
                                           : heading; /* clockwise from north */
    }
}

/** Tilt (FR15): either axis beyond 30 deg for IMU_TILT_HOLD_MS; off again
   below 25 deg on both axes. */
static uint8_t tilt_step (imu_core_t * p_ic, uint32_t now_ms)
{
    uint8_t    events = 0u;
    bool const b_over =
        (app_types_abs_i32(p_ic->pitch_cdeg) > IMU_TILT_ON_CDEG)
        || (app_types_abs_i32(p_ic->roll_cdeg) > IMU_TILT_ON_CDEG);

    if (!b_over)
    {
        p_ic->b_tilt_over = false;
    }
    else if (!p_ic->b_tilt_over)
    {
        p_ic->b_tilt_over   = true;
        p_ic->tilt_since_ms = now_ms;
    }
    else
    {
        /* still beyond the limit */
    }

    if (!p_ic->b_tilt && p_ic->b_tilt_over
        && (app_types_elapsed_ms(now_ms, p_ic->tilt_since_ms)
            >= IMU_TILT_HOLD_MS))
    {
        p_ic->b_tilt = true;
        events |= IMU_EV_TILT_ON;
    }
    else if (p_ic->b_tilt
             && (app_types_abs_i32(p_ic->pitch_cdeg) < IMU_TILT_OFF_CDEG)
             && (app_types_abs_i32(p_ic->roll_cdeg) < IMU_TILT_OFF_CDEG))
    {
        p_ic->b_tilt = false;
        events |= IMU_EV_TILT_OFF;
    }
    else
    {
        /* no change */
    }
    return events;
}

/* ---- hump sequence ------------------------------------------------------ */
/** Back to flat ground: forget the hump in progress. */
static void hump_reset (imu_core_t * p_ic)
{
    p_ic->hump_state     = IMU_HUMP_FLAT;
    p_ic->dwell_um       = 0u;
    p_ic->hump_um        = 0u;
    p_ic->h_um           = 0;
    p_ic->h_peak_um      = 0;
    p_ic->theta_max_cdeg = 0;
}

/** Add the height gained over ds_um of travel at pitch theta (0.01 deg) and
    track the peak. */
static void integrate (imu_core_t * p_ic, int32_t theta, uint32_t ds_um)
{
    /* Casts (int64_t)app_math_sin_q15(theta): every int32_t value fits;
       (int64_t)ds_um: every uint32_t value fits. */
    p_ic->h_um += ((int64_t)app_math_sin_q15(theta) * (int64_t)ds_um) / 32768;
    if (p_ic->h_um > p_ic->h_peak_um)
    {
        p_ic->h_peak_um = p_ic->h_um;
    }
    if (theta > p_ic->theta_max_cdeg)
    {
        p_ic->theta_max_cdeg = theta;
    }
}

/** Pitch hump detection on forward travel ds_um: climb, crest, descent,
    height.  Returns IMU_EV_* bits. */
static uint8_t hump_step (imu_core_t * p_ic, uint32_t ds_um)
{
    int32_t const theta  = p_ic->pitch_cdeg;
    uint8_t       events = 0u;

    switch (p_ic->hump_state)
    {
        case IMU_HUMP_FLAT:
            if ((theta > IMU_HUMP_ON_CDEG)
                && (app_types_abs_i32(p_ic->accel_mm_s2) < IMU_ACC_GATE_MM_S2))
            {
                integrate(p_ic, theta, ds_um);
                p_ic->dwell_um += ds_um;
                if (p_ic->dwell_um >= IMU_DWELL_UM)
                {
                    p_ic->hump_state = IMU_HUMP_CLIMBING;
                    p_ic->hump_um    = p_ic->dwell_um;
                    p_ic->dwell_um   = 0u;
                    events |= IMU_EV_HUMP_START;
                }
            }
            else
            {
                hump_reset(p_ic);
            }
        break;

        case IMU_HUMP_CLIMBING:
        case IMU_HUMP_CREST:
            integrate(p_ic, theta, ds_um);
            p_ic->hump_um += ds_um;
            if (p_ic->hump_um > IMU_RAMP_UM)
            {
                hump_reset(p_ic); /* a long slope, not a hump */
                events |= IMU_EV_RAMP;
            }
            else if (theta < IMU_DESC_CDEG)
            {
                p_ic->hump_state = IMU_HUMP_DESCENDING;
                p_ic->dwell_um   = 0u;
            }
            else if (theta < IMU_CREST_CDEG)
            {
                p_ic->hump_state = IMU_HUMP_CREST;
            }
            else if (theta > IMU_HUMP_ON_CDEG)
            {
                p_ic->hump_state = IMU_HUMP_CLIMBING;
            }
            else
            {
                /* between crest and climb thresholds: stay */
            }
        break;

        default: /* IMU_HUMP_DESCENDING */
            integrate(p_ic, theta, ds_um);
            p_ic->hump_um += ds_um;
            if (app_types_abs_i32(theta) < IMU_FLAT_CDEG)
            {
                p_ic->dwell_um += ds_um;
            }
            else
            {
                p_ic->dwell_um = 0u;
            }
            if ((p_ic->dwell_um >= IMU_DWELL_UM)
                || (p_ic->hump_um > (2u * IMU_RAMP_UM)))
            {
                /* HUMP_DONE: integrated peak, or the wheelbase geometry
                   check for humps shorter than the wheelbase. */
                /* Cast (int64_t)IMU_WHEELBASE_MM: a constant that fits. */
                int64_t const geo_um =
                    ((int64_t)IMU_WHEELBASE_MM * 1000
                     * app_math_sin_q15(p_ic->theta_max_cdeg))
                    / 32768;
                int64_t const peak =
                    (p_ic->h_peak_um > geo_um) ? p_ic->h_peak_um : geo_um;

                /* Casts (uint16_t)app_types_clamp_i32(...): the value is
                   0..65535; (int32_t)((peak + 500) / 1000): the peak is at most
                   the wheelbase or the climb (<= 2 x IMU_RAMP_UM): below 2000
                   mm. */
                p_ic->last_hump_mm = (uint16_t)app_types_clamp_i32(
                    (int32_t)((peak + 500) / 1000), 0, 65535);
                if (p_ic->last_hump_mm > p_ic->hump_max_mm)
                {
                    p_ic->hump_max_mm = p_ic->last_hump_mm;
                }
                p_ic->humps++;
                hump_reset(p_ic);
                events |= IMU_EV_HUMP_END;
            }
        break;
    }
    return events;
}

/* ---- one-wheel hump (roll) ------------------------------------------------
 */
/** Forget the one-wheel (roll) hump in progress. */
static void side_reset (imu_core_t * p_ic)
{
    p_ic->side_state     = 0u;
    p_ic->side_um        = 0u;
    p_ic->side_peak_cdeg = 0;
}

/** Record a finished one-wheel hump of height_mm (last and largest). */
static void side_record (imu_core_t * p_ic, uint16_t height_mm)
{
    p_ic->last_hump_mm = height_mm;
    if (height_mm > p_ic->hump_max_mm)
    {
        p_ic->hump_max_mm = height_mm;
    }
    p_ic->humps++;
}

/** Runs on forward travel only, and only while no pitch hump is in
   progress.  Returns IMU_EV_* bits. */
static uint8_t side_step (imu_core_t * p_ic, uint32_t ds_um)
{
    int32_t const roll_abs = app_types_abs_i32(p_ic->roll_cdeg);
    uint8_t       events   = 0u;

    if ((0u == p_ic->side_state) && (roll_abs > IMU_SIDE_ON_CDEG)
        && (app_types_abs_i32(p_ic->accel_mm_s2) < IMU_ACC_GATE_MM_S2))
    {
        p_ic->side_um += ds_um;
        p_ic->side_peak_cdeg =
            app_types_max_i32(p_ic->side_peak_cdeg, roll_abs);
        if (p_ic->side_um >= IMU_SIDE_DWELL_UM)
        {
            p_ic->side_state = 1u;
            events |= IMU_EV_HUMP_START;
        }
    }
    else if (0u == p_ic->side_state)
    {
        side_reset(p_ic);
    }
    else
    {
        p_ic->side_um += ds_um;
        p_ic->side_peak_cdeg =
            app_types_max_i32(p_ic->side_peak_cdeg, roll_abs);
        if (roll_abs < (p_ic->side_peak_cdeg - IMU_SIDE_DESC_CDEG))
        {
            p_ic->side_state = 2u; /* coming down */
        }
        if (p_ic->side_um > IMU_SIDE_MAX_UM)
        {
            side_reset(p_ic); /* a long side slope, not a hump */
            events |= IMU_EV_RAMP;
        }
        else if (roll_abs < IMU_SIDE_OFF_CDEG)
        {
            /* Cast (int64_t)IMU_SIDE_LEVER_MM: a constant that fits. */
            int64_t const h_um = ((int64_t)IMU_SIDE_LEVER_MM * 1000
                                  * app_math_sin_q15(p_ic->side_peak_cdeg))
                                 / 32768;

            /* Casts (uint16_t)app_types_clamp_i32(...): the value is 0..65535;
               (int32_t)((h_um + 500) / 1000): IMU_SIDE_LEVER_MM (76) x sin: at
               most 76 mm. */
            side_record(p_ic, (uint16_t)app_types_clamp_i32(
                                  (int32_t)((h_um + 500) / 1000), 0, 65535));
            side_reset(p_ic);
            events |= IMU_EV_HUMP_END;
        }
        else
        {
            /* still on the hump */
        }
    }
    return events;
}

/* ---- lift (14 Nov) ------------------------------------------------------ */
/** up_mg: low-pass filtered up axis.  A lift is the up axis staying
   IMU_LIFT_RISE_MG above the level calibration's mean for
   IMU_LIFT_HOLD_MS on flat ground (a hump also pushes the robot up).
   While driving, the largest rise of each excursion is kept, and a new
   run maximum is reported (IMU_EV_RISE_MAX), so the threshold can be
   checked against the robot's own bumps.  Returns IMU_EV_* bits. */
static uint8_t lift_step (imu_core_t * p_ic, int32_t up_mg, int32_t v_mm_s,
                          uint32_t now_ms)
{
    uint8_t       events = 0u;
    int32_t const base   = (p_ic->lvl_up_mg > 0) ? p_ic->lvl_up_mg : 1000;
    int32_t const rise   = up_mg - base;
    bool const    b_flat =
        (IMU_HUMP_FLAT == p_ic->hump_state) && (0u == p_ic->side_state);

    if (b_flat && (rise > IMU_LIFT_RISE_MG))
    {
        if (!p_ic->b_lift_over)
        {
            p_ic->b_lift_over   = true;
            p_ic->lift_since_ms = now_ms;
        }
        else if (!p_ic->b_lift
                 && (app_types_elapsed_ms(now_ms, p_ic->lift_since_ms)
                     >= IMU_LIFT_HOLD_MS))
        {
            p_ic->b_lift       = true;
            p_ic->lift_rise_mg = rise;
            events |= IMU_EV_LIFT;
        }
        else
        {
            /* waiting, or already reported */
        }
    }
    else
    {
        p_ic->b_lift_over = false;
        if (p_ic->b_lift && (rise < IMU_LIFT_REARM_MG))
        {
            p_ic->b_lift = false;
        }
    }

    /* largest rise while driving on the flat, per excursion (LOG 78) */
    if (b_flat && !p_ic->b_lift
        && (app_types_abs_i32(v_mm_s) > IMU_STATIONARY_MM_S))
    {
        p_ic->rise_peak_mg = app_types_max_i32(p_ic->rise_peak_mg, rise);
    }
    if ((rise < IMU_RISE_END_MG) && (p_ic->rise_peak_mg > 0))
    {
        if ((p_ic->rise_peak_mg > IMU_RISE_LOG_MG)
            && (p_ic->rise_peak_mg > p_ic->rise_max_mg))
        {
            p_ic->rise_max_mg = p_ic->rise_peak_mg;
            events |= IMU_EV_RISE_MAX;
        }
        p_ic->rise_peak_mg = 0;
    }
    return events;
}

/* ---- classification ------------------------------------------------------ */
/** Motion class from speed, acceleration, yaw rate and humps: stationary,
    cruising, accelerating, decelerating, turning, climbing or descending. */
static uint8_t classify (imu_core_t const * p_ic, int32_t v_mm_s,
                         int32_t yaw_cdeg_s)
{
    /* Change of SPEED, not of signed velocity: reversing away from a stop
       is accelerating, slowing down while reversing is decelerating. */
    int32_t const a_speed =
        (v_mm_s < 0) ? -p_ic->accel_mm_s2 : p_ic->accel_mm_s2;

    if (1u == p_ic->side_state)
    {
        return IMU_MOTION_CLIMBING;
    }
    if (2u == p_ic->side_state)
    {
        return IMU_MOTION_DESCENDING;
    }
    if (IMU_HUMP_CLIMBING == p_ic->hump_state)
    {
        return IMU_MOTION_CLIMBING;
    }
    if (IMU_HUMP_DESCENDING == p_ic->hump_state)
    {
        return IMU_MOTION_DESCENDING;
    }
    if ((app_types_abs_i32(v_mm_s) < IMU_STATIONARY_MM_S)
        && (app_types_abs_i32(yaw_cdeg_s) < 500))
    {
        return IMU_MOTION_STATIONARY;
    }
    if (app_types_abs_i32(yaw_cdeg_s) > IMU_TURNING_CDEG_S)
    {
        return IMU_MOTION_TURNING;
    }
    if (a_speed > IMU_ACCEL_MM_S2)
    {
        return IMU_MOTION_ACCELERATING;
    }
    if (a_speed < -IMU_ACCEL_MM_S2)
    {
        return IMU_MOTION_DECELERATING;
    }
    return IMU_MOTION_CRUISING;
}

/** body-frame value of one raw axis triple */
static int32_t axis (int32_t const vec[3], int32_t axis_no, int32_t sign)
{
    return sign * vec[axis_no];
}

/*** end of file ***/
