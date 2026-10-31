/*
 * imu_task.c
 * Subsystem 4 -- ImuTask (priority 10, core 0).
 * Woken by imu_cyc every 10 ms. Owner: Buddy 4.
 *
 * Three consecutive I2C errors raise IMU_FAULT.
 * Recovery requires IMU_RECOVER_SAMPLES good samples.
 */

#include "subsystem4.h"
#include "imu_core.h"
#include "imu_cfg.h"
#include "drv_i2c1.h"
#include "drv_lsm303.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define IMU_WAKE_TIMEOUT_MS 40
#define IMU_RETRY_CYCLES    10u
#define IMU_LOG_MOTION      51u
#define IMU_LOG_RAMP        52u
#define IMU_LOG_INIT        53u

static imu_core_t g_ic;

static bool imu_hw_init(void)
{
    static bool reported = false;
    static ER last_bus;
    static ER last_read;
    static ER last_sensor;
    static uint8_t last_id;
    static uint32_t last_diag;

    uint8_t id = 0u;
    ER bus_er = drv_i2c1_init();
    ER read_er = E_IO;
    ER sensor_er = E_IO;
    uint32_t diagnostic = 0xFF000000u;

    if (bus_er == E_OK)
    {
        read_er = drv_i2c1_read(0x1Eu, 0x0Fu, &id, 1u);
        sensor_er = drv_lsm303_init();
        diagnostic = drv_lsm303_diag();
    }

    /* Report once initially, then only when a result changes. */
    if (!reported ||
        bus_er != last_bus ||
        read_er != last_read ||
        sensor_er != last_sensor ||
        id != last_id ||
        diagnostic != last_diag)
    {
        reported = true;
        last_bus = bus_er;
        last_read = read_er;
        last_sensor = sensor_er;
        last_id = id;
        last_diag = diagnostic;

        (void)comms_telem_post(
            COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
            90u, (int32_t)bus_er);

        (void)comms_telem_post(
            COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
            91u, (int32_t)read_er);

        (void)comms_telem_post(
            COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
            92u, (int32_t)id);

        (void)comms_telem_post(
            COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
            93u, (int32_t)sensor_er);

        (void)comms_telem_post(
            COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
            IMU_LOG_INIT, (int32_t)diagnostic);
    }

    return (bus_er == E_OK) && (sensor_er == E_OK);
}

EXPORT void imu_task(INT stacd, void *p_exinf)
{
    imu_snap_t snap = {0};
    motion_odom_t odom;
    mission_snap_t ms;
    int32_t prev_dist_um = 0;
    bool b_have_dist = false;
    bool b_hw_ok;
    bool b_fault = false;
    uint8_t errors = 0u;
    uint8_t good = 0u;
    uint8_t prev_phase = (uint8_t)MISSION_CAL_PH_NONE;
    uint16_t prev_runs = 0u;
    uint32_t cycle = 0u;
    uint32_t prev_ms;

    (void)stacd;
    (void)p_exinf;

    imu_core_init(&g_ic);
    b_hw_ok = imu_hw_init();
    prev_ms = (uint32_t)app_time_ms_hw();

    for (;;)
    {
        int32_t acc[3];
        int16_t mag[3];
        bool b_mag = false;
        uint32_t now_ms;
        int32_t ds_um;
        int32_t dist_um;
        uint8_t ev;
        ER er;

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

        /* ---- I2C error handling ---- */
        if (er != E_OK)
        {
            errors = (errors < 255u)
                ? (uint8_t)(errors + 1u)
                : errors;

            snap.i2c_errors++;
            good = 0u;

            if ((errors >= IMU_I2C_FAULT_COUNT) && !b_fault)
            {
                b_fault = true;

                /* One report per fault episode. */
                (void)comms_telem_post(
                    COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                    110u, (int32_t)er);

                (void)comms_telem_post(
                    COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                    111u,
                    (int32_t)(
                        ((in_w(GPIO_IN) >> S4_PIN_IMU_SDA) & 1u) |
                        (((in_w(GPIO_IN) >> S4_PIN_IMU_SCL) & 1u)
                         << 1)));

                (void)tk_set_flg(
                    g_flg_safety, SAFETY_FLT_IMU_FAULT);

                (void)comms_telem_post(
                    (uint8_t)COMMS_KIND_FAULT,
                    (uint8_t)APP_SRC_SAFETY_IMU,
                    0u, (int32_t)SAFETY_FLT_IMU_FAULT);
            }

            if (b_fault && b_hw_ok &&
                ((cycle % IMU_RETRY_CYCLES) == 0u))
            {
                b_hw_ok = imu_hw_init();
            }

            snap.ok = 0u;
            snap.pitch_valid = 0u;
            snap.t_ms = (uint32_t)app_time_ms();
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

            (void)comms_telem_post(
                COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                112u, 1);

            (void)tk_clr_flg(
                g_flg_safety, ~(UINT)SAFETY_FLT_IMU_FAULT);
        }

        /* ---- Odometry ---- */
        motion_get_odom(&odom);
        mission_get_snap(&ms);

        dist_um = odom.dist_mm * 1000;
        ds_um = b_have_dist
            ? (int32_t)((uint32_t)dist_um -
                        (uint32_t)prev_dist_um)
            : 0;

        prev_dist_um = dist_um;
        b_have_dist = true;
        now_ms = (uint32_t)app_time_ms_hw();

        if (ms.runs != prev_runs)
        {
            prev_runs = ms.runs;
            imu_core_new_run(&g_ic);
        }

        /* ---- Calibration ---- */
        if (ms.cal_phase != prev_phase)
        {
            if (ms.cal_phase ==
                (uint8_t)MISSION_CAL_PH_STILL)
            {
                imu_cal_level_start(&g_ic);
            }
            else if (ms.cal_phase ==
                     (uint8_t)MISSION_CAL_PH_SPIN)
            {
                (void)imu_cal_level_finish(&g_ic);
                imu_cal_mag_start(&g_ic);
            }
            else if (ms.cal_phase ==
                     (uint8_t)MISSION_CAL_PH_FINISH)
            {
                bool b_level;

                if (prev_phase ==
                    (uint8_t)MISSION_CAL_PH_STILL)
                {
                    (void)imu_cal_level_finish(&g_ic);
                }
                else
                {
                    (void)imu_cal_mag_finish(&g_ic);
                }

                b_level =
                    ((g_ic.cal & IMU_CAL_LEVEL) != 0u);

                (void)mission_post(
                    (uint8_t)MISSION_EV_CAL_DONE,
                    (uint8_t)APP_SRC_SAFETY_IMU,
                    (uint16_t)APP_SRC_SAFETY_IMU,
                    b_level ? 1 : 0);
            }

            prev_phase = ms.cal_phase;
        }

        if (ms.cal_phase ==
            (uint8_t)MISSION_CAL_PH_STILL)
        {
            imu_cal_level_sample(&g_ic, acc);
        }
        else if ((ms.cal_phase ==
                  (uint8_t)MISSION_CAL_PH_SPIN) && b_mag)
        {
            imu_cal_mag_sample(&g_ic, mag);
        }

        /* ---- IMU processing ---- */
        ev = imu_core_step(
            &g_ic,
            acc,
            b_mag ? mag : NULL,
            ds_um,
            odom.v_mm_s,
            odom.yaw_rate_cdeg_s,
            now_ms - prev_ms,
            now_ms);

        prev_ms = now_ms;

        if ((ev & IMU_EV_HUMP_START) != 0u)
        {
            (void)mission_post(
                (uint8_t)MISSION_EV_HUMP_START,
                (uint8_t)APP_SRC_SAFETY_IMU,
                0u, 0);
        }

        if ((ev & IMU_EV_HUMP_END) != 0u)
        {
            (void)mission_post(
                (uint8_t)MISSION_EV_HUMP_END,
                (uint8_t)APP_SRC_SAFETY_IMU,
                0u, (int32_t)g_ic.last_hump_mm);
        }

        if ((ev & IMU_EV_TILT_ON) != 0u)
        {
            (void)tk_set_flg(
                g_flg_safety, SAFETY_FLT_TILT);
        }

        if ((ev & IMU_EV_MOTION) != 0u)
        {
            (void)comms_telem_post(
                (uint8_t)COMMS_KIND_LOG,
                (uint8_t)APP_SRC_SAFETY_IMU,
                IMU_LOG_MOTION,
                (int32_t)g_ic.motion);
        }

        if ((ev & IMU_EV_RAMP) != 0u)
        {
            (void)comms_telem_post(
                (uint8_t)COMMS_KIND_LOG,
                (uint8_t)APP_SRC_SAFETY_IMU,
                IMU_LOG_RAMP, 0);
        }

        /* ---- Snapshot ---- */
        snap.t_ms = (uint32_t)app_time_ms();
        snap.pitch_cdeg = (int16_t)g_ic.pitch_cdeg;
        snap.roll_cdeg = (int16_t)g_ic.roll_cdeg;
        snap.heading_cdeg = (int16_t)g_ic.heading_cdeg;
        snap.turn_rate_cdeg_s = odom.yaw_rate_cdeg_s;
        snap.accel_mg =
            (int16_t)(g_ic.prev_mag_mg - 1000);
        snap.hump_max_mm = g_ic.hump_max_mm;
        snap.last_hump_mm = g_ic.last_hump_mm;
        snap.humps = g_ic.humps;
        snap.hump_state = g_ic.hump_state;
        snap.motion = g_ic.motion;
        snap.cal = g_ic.cal;
        snap.ok = b_fault ? 0u : 1u;
        snap.pitch_valid =
            (!b_fault && (good >= IMU_RECOVER_SAMPLES))
            ? 1u : 0u;

        /* First successful sample, then every 100 task cycles. */
        {
            static bool first_sample = true;

            if (first_sample || ((cycle % 100u) == 0u))
            {
                first_sample = false;

                (void)comms_telem_post(
                    COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                    100u, acc[0]);

                (void)comms_telem_post(
                    COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                    101u, acc[1]);

                (void)comms_telem_post(
                    COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                    102u, acc[2]);

                (void)comms_telem_post(
                    COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                    103u, (int32_t)snap.pitch_cdeg);

                (void)comms_telem_post(
                    COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                    104u, (int32_t)snap.roll_cdeg);
            }
        }

        imu_put_snap(&snap);
    }
}