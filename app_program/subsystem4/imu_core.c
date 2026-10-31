/*
 *  imu_core.c
 *  Subsystem 4 -- ImuTask logic (pure).  See imu_core.h.  Owner: Buddy 4.
 */

#include <stddef.h>
#include "imu_core.h"
#include "imu_cfg.h"
#include "../app_types.h"
#include "../app_math.h"

#define MG_PER_G_MM_S2      9807        /* 1 g = 9807 mm/s^2 */

/* body-frame value of one raw axis triple */
static int32_t axis(int32_t const v[3], int axis_no, int32_t sign)
{
    return sign * v[axis_no];
}

void imu_core_init(imu_core_t * p_ic)
{
    uint8_t * p_b = (uint8_t *)p_ic;
    size_t    i;

    for (i = 0u; i < sizeof(*p_ic); i++)
    {
        p_b[i] = 0u;
    }
    p_ic->hump_state = (uint8_t)IMU_HUMP_FLAT;
    p_ic->motion     = (uint8_t)IMU_MOTION_STATIONARY;
}

void imu_core_new_run(imu_core_t * p_ic)
{
    p_ic->hump_max_mm  = 0u;
    p_ic->last_hump_mm = 0u;
    p_ic->humps        = 0u;
}

/* ---- calibration ------------------------------------------------------ */
void imu_cal_level_start(imu_core_t * p_ic)
{
    uint8_t k;

    for (k = 0u; k < 3u; k++)
    {
        p_ic->lvl_sum[k] = 0;
        p_ic->lvl_min[k] = 0x7FFFFFFF;
        p_ic->lvl_max[k] = -0x7FFFFFFF;
    }
    p_ic->lvl_n = 0u;
}

void imu_cal_level_sample(imu_core_t * p_ic, int32_t const acc_mg[3])
{
    uint8_t k;

    if (p_ic->lvl_n == 0xFFFFu)
    {
        return;
    }
    for (k = 0u; k < 3u; k++)
    {
        p_ic->lvl_sum[k] += acc_mg[k];
        p_ic->lvl_min[k]  = app_min_i32(p_ic->lvl_min[k], acc_mg[k]);
        p_ic->lvl_max[k]  = app_max_i32(p_ic->lvl_max[k], acc_mg[k]);
    }
    p_ic->lvl_n++;
}

bool imu_cal_level_finish(imu_core_t * p_ic)
{
    int32_t mean[3];
    int32_t fwd;
    int32_t left;
    int32_t up;
    uint8_t k;

    if (p_ic->lvl_n < 50u)
    {
        return false;
    }
    for (k = 0u; k < 3u; k++)
    {
        if ((p_ic->lvl_max[k] - p_ic->lvl_min[k]) > IMU_LEVEL_MAX_SPREAD_MG)
        {
            return false;               /* the robot was moving */
        }
        mean[k] = (int32_t)(p_ic->lvl_sum[k] / p_ic->lvl_n);
    }
    fwd  = axis(mean, IMU_AXIS_FWD, IMU_SIGN_FWD);
    left = axis(mean, IMU_AXIS_LEFT, IMU_SIGN_LEFT);
    up   = axis(mean, IMU_AXIS_UP, IMU_SIGN_UP);
    if (up < 500)
    {
        return false;                   /* not right way up (or axes mis-set) */
    }
    p_ic->pitch0_cdeg = app_atan2_cdeg(fwd, (int32_t)app_isqrt64((uint64_t)((int64_t)left * left +
                                                                         (int64_t)up * up)));
    p_ic->roll0_cdeg  = app_atan2_cdeg(left, up);
    p_ic->cal |= IMU_CAL_LEVEL;
    return true;
}

void imu_cal_mag_start(imu_core_t * p_ic)
{
    uint8_t k;

    for (k = 0u; k < 3u; k++)
    {
        p_ic->mag_min[k] = 32767;
        p_ic->mag_max[k] = -32768;
    }
}

void imu_cal_mag_sample(imu_core_t * p_ic, int16_t const mag[3])
{
    uint8_t k;

    for (k = 0u; k < 3u; k++)
    {
        if (mag[k] < p_ic->mag_min[k])
        {
            p_ic->mag_min[k] = mag[k];
        }
        if (mag[k] > p_ic->mag_max[k])
        {
            p_ic->mag_max[k] = mag[k];
        }
    }
}

bool imu_cal_mag_finish(imu_core_t * p_ic)
{
    int const axes[2] = { IMU_AXIS_FWD, IMU_AXIS_LEFT };   /* horizontal plane */
    uint8_t   k;

    for (k = 0u; k < 2u; k++)
    {
        if (((int32_t)p_ic->mag_max[axes[k]] - p_ic->mag_min[axes[k]]) < IMU_MAG_MIN_SPAN)
        {
            return false;
        }
    }
    for (k = 0u; k < 3u; k++)
    {
        p_ic->mag_off[k] = (int16_t)(((int32_t)p_ic->mag_min[k] + p_ic->mag_max[k]) / 2);
    }
    p_ic->cal |= IMU_CAL_MAG;
    return true;
}

/* ---- hump sequence ------------------------------------------------------ */
static void hump_reset(imu_core_t * p_ic)
{
    p_ic->hump_state     = (uint8_t)IMU_HUMP_FLAT;
    p_ic->dwell_um       = 0u;
    p_ic->hump_um        = 0u;
    p_ic->h_um           = 0;
    p_ic->h_peak_um      = 0;
    p_ic->theta_max_cdeg = 0;
}

static void integrate(imu_core_t * p_ic, int32_t theta, uint32_t ds_um)
{
    p_ic->h_um += ((int64_t)app_sin_q15(theta) * ds_um) / 32768;
    if (p_ic->h_um > p_ic->h_peak_um)
    {
        p_ic->h_peak_um = p_ic->h_um;
    }
    if (theta > p_ic->theta_max_cdeg)
    {
        p_ic->theta_max_cdeg = theta;
    }
}

static uint8_t hump_step(imu_core_t * p_ic, uint32_t ds_um)
{
    int32_t const theta = p_ic->pitch_cdeg;
    uint8_t       ev    = 0u;

    switch (p_ic->hump_state)
    {
        case IMU_HUMP_FLAT:
            if ((theta > IMU_HUMP_ON_CDEG) && (app_abs_i32(p_ic->accel_mm_s2) < IMU_ACC_GATE_MM_S2))
            {
                integrate(p_ic, theta, ds_um);
                p_ic->dwell_um += ds_um;
                if (p_ic->dwell_um >= IMU_DWELL_UM)
                {
                    p_ic->hump_state = (uint8_t)IMU_HUMP_CLIMBING;
                    p_ic->hump_um    = p_ic->dwell_um;
                    p_ic->dwell_um   = 0u;
                    ev |= IMU_EV_HUMP_START;
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
                hump_reset(p_ic);           /* a long slope, not a hump */
                ev |= IMU_EV_RAMP;
            }
            else if (theta < IMU_DESC_CDEG)
            {
                p_ic->hump_state = (uint8_t)IMU_HUMP_DESCENDING;
                p_ic->dwell_um   = 0u;
            }
            else if (theta < IMU_CREST_CDEG)
            {
                p_ic->hump_state = (uint8_t)IMU_HUMP_CREST;
            }
            else if (theta > IMU_HUMP_ON_CDEG)
            {
                p_ic->hump_state = (uint8_t)IMU_HUMP_CLIMBING;
            }
            else
            {
                /* between crest and climb thresholds: stay */
            }
            break;

        default:                            /* IMU_HUMP_DESCENDING */
            integrate(p_ic, theta, ds_um);
            p_ic->hump_um += ds_um;
            if (app_abs_i32(theta) < IMU_FLAT_CDEG)
            {
                p_ic->dwell_um += ds_um;
            }
            else
            {
                p_ic->dwell_um = 0u;
            }
            if ((p_ic->dwell_um >= IMU_DWELL_UM) || (p_ic->hump_um > (2u * IMU_RAMP_UM)))
            {
                /* HUMP_DONE: integrated peak, or the wheelbase geometry
                   check for humps shorter than the wheelbase. */
                int64_t const geo_um = ((int64_t)IMU_WHEELBASE_MM * 1000 *
                                        app_sin_q15(p_ic->theta_max_cdeg)) / 32768;
                int64_t const peak   = (p_ic->h_peak_um > geo_um) ? p_ic->h_peak_um : geo_um;

                p_ic->last_hump_mm = (uint16_t)app_clamp_i32((int32_t)((peak + 500) / 1000), 0, 65535);
                if (p_ic->last_hump_mm > p_ic->hump_max_mm)
                {
                    p_ic->hump_max_mm = p_ic->last_hump_mm;
                }
                p_ic->humps++;
                hump_reset(p_ic);
                ev |= IMU_EV_HUMP_END;
            }
            break;
    }
    return ev;
}

/* ---- classification ------------------------------------------------------ */
static uint8_t classify(imu_core_t const * p_ic, int32_t v_mm_s, int32_t yaw_cdeg_s)
{
    if (p_ic->b_impact)
    {
        return (uint8_t)IMU_MOTION_IMPACT;
    }
    if (p_ic->hump_state == (uint8_t)IMU_HUMP_CLIMBING)
    {
        return (uint8_t)IMU_MOTION_CLIMBING;
    }
    if (p_ic->hump_state == (uint8_t)IMU_HUMP_DESCENDING)
    {
        return (uint8_t)IMU_MOTION_DESCENDING;
    }
    if ((app_abs_i32(v_mm_s) < IMU_STATIONARY_MM_S) && (app_abs_i32(yaw_cdeg_s) < 500))
    {
        return (uint8_t)IMU_MOTION_STATIONARY;
    }
    if (app_abs_i32(yaw_cdeg_s) > IMU_TURNING_CDEG_S)
    {
        return (uint8_t)IMU_MOTION_TURNING;
    }
    if (p_ic->accel_mm_s2 > IMU_ACCEL_MM_S2)
    {
        return (uint8_t)IMU_MOTION_ACCELERATING;
    }
    if (p_ic->accel_mm_s2 < -IMU_ACCEL_MM_S2)
    {
        return (uint8_t)IMU_MOTION_BRAKING;
    }
    return (uint8_t)IMU_MOTION_CRUISING;
}

uint8_t imu_core_step(imu_core_t * p_ic, int32_t const acc_mg[3], int16_t const * p_mag,
                      int32_t ds_um, int32_t v_mm_s, int32_t yaw_cdeg_s,
                      uint32_t dt_ms, uint32_t now_ms)
{
    uint8_t ev = 0u;
    int32_t a[3];
    int32_t fwd;
    int32_t left;
    int32_t up;
    int32_t mag_mg;
    uint8_t k;
    uint8_t cls;

    if (dt_ms == 0u)
    {
        dt_ms = 10u;
    }

    /* Low-pass filter (alpha 1/4), mg x 16. */
    for (k = 0u; k < 3u; k++)
    {
        if (!p_ic->b_have_acc)
        {
            p_ic->acc_f[k] = acc_mg[k] * 16;
        }
        p_ic->acc_f[k] += ((acc_mg[k] * 16) - p_ic->acc_f[k]) / (1 << IMU_ACC_LPF_SHIFT);
        a[k] = p_ic->acc_f[k] / 16;
    }

    /* Impact: a jump of |a| between samples (unfiltered). */
    mag_mg = (int32_t)app_isqrt64((uint64_t)((int64_t)acc_mg[0] * acc_mg[0] +
                                             (int64_t)acc_mg[1] * acc_mg[1] +
                                             (int64_t)acc_mg[2] * acc_mg[2]));
    if (p_ic->b_have_acc && (app_abs_i32(mag_mg - p_ic->prev_mag_mg) > IMU_IMPACT_MG))
    {
        p_ic->b_impact  = true;
        p_ic->impact_ms = now_ms;
        ev |= IMU_EV_IMPACT;
    }
    else if (p_ic->b_impact && (app_elapsed_ms(now_ms, p_ic->impact_ms) >= IMU_IMPACT_HOLD_MS))
    {
        p_ic->b_impact = false;
    }
    else
    {
        /* no change */
    }
    p_ic->prev_mag_mg = mag_mg;
    p_ic->b_have_acc  = true;

    /* Longitudinal acceleration from the encoders, filtered. */
    if (p_ic->b_have_v)
    {
        int32_t const raw = ((v_mm_s - p_ic->prev_v) * 1000) / (int32_t)dt_ms;

        p_ic->accel_mm_s2 += (raw - p_ic->accel_mm_s2) / 4;
    }
    p_ic->prev_v   = v_mm_s;
    p_ic->b_have_v = true;

    /* Attitude.  The forward axis also sees the robot's own acceleration,
       so subtract the encoder estimate before taking the angle. */
    fwd  = axis(a, IMU_AXIS_FWD, IMU_SIGN_FWD) - ((p_ic->accel_mm_s2 * 1000) / MG_PER_G_MM_S2);
    left = axis(a, IMU_AXIS_LEFT, IMU_SIGN_LEFT);
    up   = axis(a, IMU_AXIS_UP, IMU_SIGN_UP);
    p_ic->pitch_cdeg = app_wrap_cdeg(app_atan2_cdeg(fwd, (int32_t)app_isqrt64((uint64_t)((int64_t)left * left +
                                                                 (int64_t)up * up))) - p_ic->pitch0_cdeg);
    p_ic->roll_cdeg  = app_wrap_cdeg(app_atan2_cdeg(left, up) - p_ic->roll0_cdeg);

    if ((p_mag != NULL) && ((p_ic->cal & IMU_CAL_MAG) != 0u))
    {
        int32_t const m[3] = { (int32_t)p_mag[0] - p_ic->mag_off[0],
                               (int32_t)p_mag[1] - p_ic->mag_off[1],
                               (int32_t)p_mag[2] - p_ic->mag_off[2] };
        int32_t h = app_atan2_cdeg(axis(m, IMU_AXIS_LEFT, IMU_SIGN_LEFT), axis(m, IMU_AXIS_FWD, IMU_SIGN_FWD));

        p_ic->heading_cdeg = (h < 0) ? (h + 36000) : h;         /* clockwise from north */
    }

    /* Tilt (FR15): either axis beyond 30 deg. */
    if (!p_ic->b_tilt && ((app_abs_i32(p_ic->pitch_cdeg) > IMU_TILT_ON_CDEG) ||
                          (app_abs_i32(p_ic->roll_cdeg) > IMU_TILT_ON_CDEG)))
    {
        p_ic->b_tilt = true;
        ev |= IMU_EV_TILT_ON;
    }
    else if (p_ic->b_tilt && (app_abs_i32(p_ic->pitch_cdeg) < IMU_TILT_OFF_CDEG) &&
             (app_abs_i32(p_ic->roll_cdeg) < IMU_TILT_OFF_CDEG))
    {
        p_ic->b_tilt = false;
        ev |= IMU_EV_TILT_OFF;
    }
    else
    {
        /* no change */
    }

    /* Hump detection runs on forward travel only. */
    if (ds_um > 0)
    {
        ev |= hump_step(p_ic, (uint32_t)ds_um);
    }

    cls = classify(p_ic, v_mm_s, yaw_cdeg_s);
    if ((cls != p_ic->motion) &&
        ((app_elapsed_ms(now_ms, p_ic->motion_ms) >= IMU_CLASS_HOLD_MS) || (cls == (uint8_t)IMU_MOTION_IMPACT)))
    {
        p_ic->motion    = cls;
        p_ic->motion_ms = now_ms;
        ev |= IMU_EV_MOTION;
    }
    return ev;
}
