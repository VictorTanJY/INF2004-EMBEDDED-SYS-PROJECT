/*
 *  imu_core.h
 *  Subsystem 4 -- ImuTask logic as pure functions (Week 6 Section 6.3):
 *  pitch / roll, level and hard-iron calibration, the hump sequence with
 *  height = sum(sin(theta) x ds) over ENCODER distance, motion
 *  classification, tilt and impact.  Owner: Buddy 4.  Host-tested.
 */

#ifndef IMU_CORE_H
#define IMU_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"

/* imu_core_step() result bits */
#define IMU_EV_HUMP_START       0x01u
#define IMU_EV_HUMP_END         0x02u   /* imu_core_t.last_hump_mm valid */
#define IMU_EV_TILT_ON          0x04u
#define IMU_EV_TILT_OFF         0x08u
#define IMU_EV_IMPACT           0x10u
#define IMU_EV_MOTION           0x20u   /* imu_core_t.motion changed     */
#define IMU_EV_RAMP             0x40u   /* a climb > 600 mm was a ramp   */

typedef struct
{
    int32_t  acc_f[3];          /* filtered acceleration, mg x 16 */
    bool     b_have_acc;
    int32_t  prev_mag_mg;       /* |a| of the previous sample      */

    /* level calibration */
    int32_t  pitch0_cdeg;
    int32_t  roll0_cdeg;
    int64_t  lvl_sum[3];
    int32_t  lvl_min[3];
    int32_t  lvl_max[3];
    uint16_t lvl_n;

    /* hard-iron calibration */
    int16_t  mag_min[3];
    int16_t  mag_max[3];
    int16_t  mag_off[3];
    uint8_t  cal;               /* IMU_CAL_* */

    /* attitude */
    int32_t  pitch_cdeg;
    int32_t  roll_cdeg;
    int32_t  heading_cdeg;
    int32_t  accel_mm_s2;       /* dv/dt from odometry, filtered */
    int32_t  prev_v;
    bool     b_have_v;

    /* hump sequence */
    uint8_t  hump_state;        /* imu_hump_state_t */
    uint32_t dwell_um;          /* guard distance in the current condition */
    uint32_t hump_um;           /* distance since the climb started */
    int64_t  h_um;              /* integrated height */
    int64_t  h_peak_um;
    int32_t  theta_max_cdeg;
    uint16_t last_hump_mm;
    uint16_t hump_max_mm;
    uint16_t humps;

    /* safety / classification */
    bool     b_tilt;
    bool     b_tilt_over;       /* beyond the limit, waiting for IMU_TILT_HOLD_MS */
    uint32_t tilt_since_ms;
    uint32_t impact_ms;
    bool     b_impact;
    uint8_t  motion;            /* imu_motion_t */
    uint32_t motion_ms;
} imu_core_t;

void    imu_core_init(imu_core_t * p_ic);

/* Level offsets: stand still ~1 s (CALIBRATING STILL phase). */
void    imu_cal_level_start(imu_core_t * p_ic);
void    imu_cal_level_sample(imu_core_t * p_ic, int32_t const acc_mg[3]);
bool    imu_cal_level_finish(imu_core_t * p_ic);

/* Hard-iron offsets: 360 deg spin (CALIBRATING SPIN phase). */
void    imu_cal_mag_start(imu_core_t * p_ic);
void    imu_cal_mag_sample(imu_core_t * p_ic, int16_t const mag[3]);
bool    imu_cal_mag_finish(imu_core_t * p_ic);

/*
 * One 10 ms step.  acc_mg: raw axes in mg; mag: raw counts (NULL = no new
 * sample); ds_um: encoder distance since the last step (signed); v_mm_s
 * and yaw_cdeg_s from the odometry snapshot.  Returns IMU_EV_* bits.
 */
uint8_t imu_core_step(imu_core_t * p_ic, int32_t const acc_mg[3], int16_t const * p_mag,
                      int32_t ds_um, int32_t v_mm_s, int32_t yaw_cdeg_s,
                      uint32_t dt_ms, uint32_t now_ms);

/* New run: clear the run's hump maximum and count (MissionTask start). */
void    imu_core_new_run(imu_core_t * p_ic);

#endif /* IMU_CORE_H */
