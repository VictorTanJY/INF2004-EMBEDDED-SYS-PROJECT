/*
 *  imu_task.c
 *  Subsystem 4 -- ImuTask (priority 10, core 0), woken by imu_cyc every
 *  10 ms (Week 6 Sections 3 and 6.3).  Owner: Buddy 4.
 *
 *  Every cycle: heartbeat; read the accelerometer (and the magnetometer
 *  every 2nd cycle) over I2C1 -- blocking ~0.3-0.6 ms, which is why this
 *  task is on core 0, away from the control loops; take the encoder
 *  distance from the odometry snapshot; run imu_core; report:
 *    HUMP_START / HUMP_END(peak) -> mbf_mission (MissionTask echoes them
 *    to telemetry), TILT -> flg_safety, motion class -> mbf_telem.
 *  Three consecutive I2C errors: IMU_FAULT (report only), snapshot not
 *  valid, bus re-initialised every 100 ms; pitch_valid only after
 *  IMU_RECOVER_SAMPLES good samples (Tribunal 1 gap 6).
 */

#include "subsystem4.h"
#include "imu_core.h"
#include "imu_cfg.h"
#include "drv_i2c1.h"
#include "drv_lsm303.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define IMU_WAKE_TIMEOUT_MS     40
#define IMU_RETRY_CYCLES        10u
#define IMU_LOG_MOTION          51u     /* comms_rec_t code: data = imu_motion_t */
#define IMU_LOG_RAMP            52u

static imu_core_t g_ic;

static bool imu_hw_init(void)
{
    return (drv_i2c1_init() == E_OK) && (drv_lsm303_init() == E_OK);
}

EXPORT void imu_task(INT stacd, void * p_exinf)
{
    imu_snap_t     snap = {0};
    motion_odom_t  odom;
    mission_snap_t ms;
    int32_t        prev_dist_um = 0;
    bool           b_have_dist = false;
    bool           b_hw_ok;
    bool           b_fault = false;
    uint8_t        errors = 0u;
    uint8_t        good = 0u;
    uint8_t        prev_phase = (uint8_t)MISSION_CAL_PH_NONE;
    uint16_t       prev_runs = 0u;
    uint32_t       cycle = 0u;
    uint32_t       prev_ms;

    (void)stacd;
    (void)p_exinf;

    imu_core_init(&g_ic);
    b_hw_ok = imu_hw_init();
    prev_ms = (uint32_t)app_time_ms_hw();

    for (;;)
    {
        int32_t  acc[3];
        int16_t  mag[3];
        bool     b_mag = false;
        uint32_t now_ms;
        int32_t  ds_um;
        int32_t  dist_um;
        uint8_t  ev;
        ER       er;

        (void)tk_slp_tsk(IMU_WAKE_TIMEOUT_MS);
        safety_heartbeat(SAFETY_HB_IMU);
        cycle++;

        if (!b_hw_ok)
        {
            if ((cycle % IMU_RETRY_CYCLES) == 0u)
            {
                b_hw_ok = imu_hw_init();
            }
            er = E_IO;
        }
        else
        {
            er = drv_lsm303_read_accel(acc);
            if ((er == E_OK) && ((cycle % 2u) == 0u))
            {
                b_mag = (drv_lsm303_read_mag(mag) == E_OK);
            }
        }

        /* ---- I2C error handling --------------------------------------- */
        if (er != E_OK)
        {
            errors = (errors < 255u) ? (uint8_t)(errors + 1u) : errors;
            snap.i2c_errors++;
            good = 0u;
            if ((errors >= IMU_I2C_FAULT_COUNT) && !b_fault)
            {
                b_fault = true;
                (void)tk_set_flg(g_flg_safety, SAFETY_FLT_IMU_FAULT);
                (void)comms_telem_post((uint8_t)COMMS_KIND_FAULT, (uint8_t)APP_SRC_SAFETY_IMU,
                                       0u, (int32_t)SAFETY_FLT_IMU_FAULT);
            }
            if (b_fault && b_hw_ok && ((cycle % IMU_RETRY_CYCLES) == 0u))
            {
                b_hw_ok = imu_hw_init();        /* bus recovery */
            }
            snap.ok          = 0u;
            snap.pitch_valid = 0u;
            snap.t_ms        = (uint32_t)app_time_ms();
            imu_put_snap(&snap);
            continue;
        }
        errors = 0u;
        if (good < 255u)
        {
            good++;
        }
        if (b_fault && (good >= IMU_RECOVER_SAMPLES))
        {
            b_fault = false;
            (void)tk_clr_flg(g_flg_safety, ~(UINT)SAFETY_FLT_IMU_FAULT);
        }

        /* ---- odometry -------------------------------------------------- */
        motion_get_odom(&odom);
        mission_get_snap(&ms);
        dist_um = odom.dist_mm * 1000;          /* 1 mm resolution is enough per 10 ms */
        ds_um   = b_have_dist ? (int32_t)((uint32_t)dist_um - (uint32_t)prev_dist_um) : 0;
        prev_dist_um = dist_um;
        b_have_dist  = true;
        now_ms  = (uint32_t)app_time_ms_hw();

        if (ms.runs != prev_runs)
        {
            prev_runs = ms.runs;
            imu_core_new_run(&g_ic);            /* hump maximum is per run (FR7) */
        }

        /* ---- calibration (Week 6 6.3) --------------------------------- */
        if (ms.cal_phase != prev_phase)
        {
            if (ms.cal_phase == (uint8_t)MISSION_CAL_PH_STILL)
            {
                imu_cal_level_start(&g_ic);
            }
            else if (ms.cal_phase == (uint8_t)MISSION_CAL_PH_SPIN)
            {
                (void)imu_cal_level_finish(&g_ic);
                imu_cal_mag_start(&g_ic);
            }
            else if (ms.cal_phase == (uint8_t)MISSION_CAL_PH_FINISH)
            {
                bool b_level;

                if (prev_phase == (uint8_t)MISSION_CAL_PH_STILL)
                {
                    (void)imu_cal_level_finish(&g_ic);  /* no spin phase */
                }
                else
                {
                    (void)imu_cal_mag_finish(&g_ic);    /* heading only: not required */
                }
                b_level = ((g_ic.cal & IMU_CAL_LEVEL) != 0u);
                (void)mission_post((uint8_t)MISSION_EV_CAL_DONE, (uint8_t)APP_SRC_SAFETY_IMU,
                                   (uint16_t)APP_SRC_SAFETY_IMU, b_level ? 1 : 0);
            }
            else
            {
                /* NONE: nothing */
            }
            prev_phase = ms.cal_phase;
        }
        if (ms.cal_phase == (uint8_t)MISSION_CAL_PH_STILL)
        {
            imu_cal_level_sample(&g_ic, acc);
        }
        else if ((ms.cal_phase == (uint8_t)MISSION_CAL_PH_SPIN) && b_mag)
        {
            imu_cal_mag_sample(&g_ic, mag);
        }
        else
        {
            /* not calibrating */
        }

        /* ---- step ------------------------------------------------------ */
        ev = imu_core_step(&g_ic, acc, b_mag ? mag : NULL, ds_um, odom.v_mm_s,
                           odom.yaw_rate_cdeg_s, now_ms - prev_ms, now_ms);
        prev_ms = now_ms;

        if ((ev & IMU_EV_HUMP_START) != 0u)
        {
            (void)mission_post((uint8_t)MISSION_EV_HUMP_START, (uint8_t)APP_SRC_SAFETY_IMU, 0u, 0);
        }
        if ((ev & IMU_EV_HUMP_END) != 0u)
        {
            (void)mission_post((uint8_t)MISSION_EV_HUMP_END, (uint8_t)APP_SRC_SAFETY_IMU, 0u,
                               (int32_t)g_ic.last_hump_mm);
        }
        if ((ev & IMU_EV_TILT_ON) != 0u)
        {
            (void)tk_set_flg(g_flg_safety, SAFETY_FLT_TILT);
        }
        if ((ev & IMU_EV_MOTION) != 0u)
        {
            (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_SAFETY_IMU,
                                   IMU_LOG_MOTION, (int32_t)g_ic.motion);
        }
        if ((ev & IMU_EV_RAMP) != 0u)
        {
            (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_SAFETY_IMU,
                                   IMU_LOG_RAMP, 0);
        }

        /* ---- snapshot -------------------------------------------------- */
        snap.t_ms             = (uint32_t)app_time_ms();
        snap.pitch_cdeg       = (int16_t)g_ic.pitch_cdeg;
        snap.roll_cdeg        = (int16_t)g_ic.roll_cdeg;
        snap.heading_cdeg     = (int16_t)g_ic.heading_cdeg;
        snap.turn_rate_cdeg_s = odom.yaw_rate_cdeg_s;
        snap.accel_mg         = (int16_t)(g_ic.prev_mag_mg - 1000);
        snap.hump_max_mm      = g_ic.hump_max_mm;
        snap.last_hump_mm     = g_ic.last_hump_mm;
        snap.humps            = g_ic.humps;
        snap.hump_state       = g_ic.hump_state;
        snap.motion           = g_ic.motion;
        snap.cal              = g_ic.cal;
        snap.ok               = b_fault ? 0u : 1u;
        snap.pitch_valid      = (!b_fault && (good >= IMU_RECOVER_SAMPLES)) ? 1u : 0u;
        imu_put_snap(&snap);
    }
}
