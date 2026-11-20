/** @file imu_task.c
 *
 *  @brief Subsystem 4 -- ImuTask (priority 10, core 0).
 *  Woken by imu_cyc every 10 ms. Owner: Buddy 4.
 *
 *  Three consecutive I2C errors raise IMU_FAULT.
 *  Recovery requires IMU_RECOVER_SAMPLES good samples.
 */

#include "imu_task.h"
#include "subsystem4.h"
#include "imu_core.h"
#include "imu_cfg.h"
#include "drv_i2c1.h"
#include "drv_lsm303.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem5/subsystem5.h"
#include "../app_types.h"

#define IMU_WAKE_TIMEOUT_MS 40
#define IMU_RETRY_CYCLES    10u
#define IMU_LOG_MOTION      51u
#define IMU_LOG_RAMP        52u
#define IMU_LOG_INIT        53u

/* tilt warning: data = larger of |pitch|, |roll|, 0.01 deg */
#define IMU_LOG_TILT_ON 55u

/* tilt warning cleared: back below IMU_TILT_OFF_CDEG */
#define IMU_LOG_TILT_OFF 56u

/* level calibration: samples (needs >= 50) */
#define IMU_LOG_LVL_N 57u

/* level calibration: largest axis spread, mg (<= 60) */
#define IMU_LOG_LVL_SPREAD 58u

/* lift seen: data = up-axis rise above level, mg (14 Nov) */
#define IMU_LOG_LIFT 77u

/* largest up-axis rise while driving this run, mg */
#define IMU_LOG_RISE_MAX 78u

/* hump events held while all three IR sensors are black (20 Nov) */
#define IMU_HELD_MAX 4u

/* level calibration: up-axis mean, mg (>= 500) */
#define IMU_LOG_LVL_UP 59u

/** ImuTask's state, kept from one 10 ms cycle to the next. */
typedef struct
{
    imu_types_snap_t snap;
    int32_t          prev_dist_um;
    bool             b_have_dist;
    bool             b_hw_ok;
    bool             b_fault;
    uint8_t          errors;
    uint8_t          good;
    uint8_t          prev_phase;
    bool     b_tilt_sent; /**< SAFETY_FLT_TILT set for this tilt (14 Nov) */
    bool     b_was_black; /**< line snapshot: all IR black last cycle     */
    bool     b_was_air;   /**< line snapshot: no floor last cycle         */
    bool     b_air_start; /**< the IR sensors just found no floor         */
    bool     b_release;   /**< the floor is back: post the held events    */
    uint8_t  held_n;      /**< hump events held while all IR black        */
    uint8_t  held_type[IMU_HELD_MAX]; /**< MISSION_EV_HUMP_START or _END  */
    uint16_t held_mm[IMU_HELD_MAX];   /**< peak of a held HUMP_END, mm    */
    UINT     safety_bits; /**< flg_safety bits to set after the snapshot  */
    uint32_t still_ms;    /**< start of the stand-still phase             */
    uint16_t prev_runs;
    uint32_t cycle;
    uint32_t prev_ms;
} imu_task_state_t;

static imu_core_t g_ic;

static bool imu_hw_init(void);
static void imu_read_failed(imu_task_state_t * p_st, ER err);
static void imu_read_ok(imu_task_state_t * p_st, int32_t const acc[IMU_AXES],
                        int16_t const * p_mag);
static void imu_cal_phase(imu_task_state_t * p_st, uint8_t new_phase,
                          uint32_t now_ms);
static void imu_cal_finish(uint8_t prev_phase);
static void imu_cal_sample(imu_task_state_t const * p_st, uint8_t phase,
                           uint32_t now_ms, int32_t const acc[IMU_AXES],
                           int16_t const * p_mag);
static bool imu_floor_check(imu_task_state_t * p_st);
static void imu_report(imu_task_state_t * p_st, uint8_t events,
                       mission_types_snap_t const * p_mission);
static void imu_hump_event(imu_task_state_t * p_st, uint8_t type,
                           uint16_t peak_mm);
static void imu_hump_release(imu_task_state_t * p_st);
static void imu_safety_bits(imu_task_state_t * p_st, uint8_t events,
                            mission_types_snap_t const * p_mission);
static void imu_log_events(uint8_t events);
static void imu_snapshot(imu_task_state_t *          p_st,
                         motion_types_odom_t const * p_odom);
static void imu_debug_logs(imu_task_state_t const * p_st,
                           int32_t const            acc[IMU_AXES]);

EXPORT void imu_task_thread (INT stacd, void * p_exinf)
{
    imu_task_state_t state = {0};

    (void)stacd;
    (void)p_exinf;

    state.prev_phase = MISSION_CAL_PH_NONE;
    imu_core_init(&g_ic);
    state.b_hw_ok = imu_hw_init();
    state.prev_ms = app_time_ms_hw();

    for (;;)
    {
        int32_t acc[IMU_AXES];
        int16_t mag[IMU_AXES];
        bool    b_mag = false;
        ER      err;

        (void)tk_slp_tsk(IMU_WAKE_TIMEOUT_MS);
        subsystem4_heartbeat(SAFETY_HB_IMU);
        state.cycle++;

        if (!state.b_hw_ok)
        {
            if (0u == (state.cycle % IMU_RETRY_CYCLES))
            {
                state.b_hw_ok = imu_hw_init();
            }

            err = E_IO;
        }
        else
        {
            err = drv_lsm303_read_accel(acc);

            if ((E_OK == err) && (0u == (state.cycle % 2u)))
            {
                b_mag = (E_OK == drv_lsm303_read_mag(mag));
            }
        }

        if (err != E_OK)
        {
            imu_read_failed(&state, err);
        }
        else
        {
            imu_read_ok(&state, acc, b_mag ? mag : NULL);
        }
    }
}

/** I2C error handling: three in a row raise IMU_FAULT, once per episode. */
static void imu_read_failed (imu_task_state_t * p_st, ER err)
{
    /* Cast (uint8_t)(p_st->errors + 1u): errors < 255 on this branch. */
    p_st->errors =
        (p_st->errors < 255u) ? (uint8_t)(p_st->errors + 1u) : p_st->errors;

    p_st->snap.i2c_errors++;
    p_st->good = 0u;

    if ((p_st->errors >= IMU_I2C_FAULT_COUNT) && !p_st->b_fault)
    {
        p_st->b_fault = true;

        /* One report per fault episode. */
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 110u,
                                    err);

        /* Cast (int32_t)(((in_w(APP_REGS_GPIO_IN) ...))): two pin levels:
           0..3. */
        (void)subsystem1_telem_post(
            COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 111u,
            (int32_t)(((in_w(APP_REGS_GPIO_IN) >> S4_PIN_IMU_SDA) & 1u)
                      | (((in_w(APP_REGS_GPIO_IN) >> S4_PIN_IMU_SCL) & 1u)
                         << 1)));

        (void)tk_set_flg(subsystem4_flg_safety(), SAFETY_FLT_IMU_FAULT);

        (void)subsystem1_telem_post(COMMS_KIND_FAULT, APP_SRC_SAFETY_IMU, 0u,
                                    SAFETY_FLT_IMU_FAULT);
    }

    if (p_st->b_fault && p_st->b_hw_ok
        && (0u == (p_st->cycle % IMU_RETRY_CYCLES)))
    {
        p_st->b_hw_ok = imu_hw_init();
    }

    p_st->snap.b_ok        = 0u;
    p_st->snap.pitch_valid = 0u;
    p_st->snap.t_ms        = app_time_ms();
    subsystem4_put_imu_snap(&p_st->snap);
}

/** One good sample: recovery, calibration, humps, events and the snapshot.
   p_mag is NULL when no magnetometer sample was read this cycle. */
static void imu_read_ok (imu_task_state_t * p_st, int32_t const acc[IMU_AXES],
                         int16_t const * p_mag)
{
    motion_types_odom_t  odom;
    mission_types_snap_t mission;
    uint32_t             now_ms;
    int32_t              ds_um;
    int32_t              dist_um;
    uint8_t              events;

    p_st->errors = 0u;

    if (p_st->good < 255u)
    {
        p_st->good++;
    }

    if (p_st->b_fault && (p_st->good >= IMU_RECOVER_SAMPLES))
    {
        p_st->b_fault = false;

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 112u,
                                    1);

        /* Cast (UINT)SAFETY_FLT_IMU_FAULT: a constant that fits. */
        (void)tk_clr_flg(subsystem4_flg_safety(), ~(UINT)SAFETY_FLT_IMU_FAULT);
    }

    /* ---- Odometry ---- */
    subsystem2_get_odom(&odom);
    subsystem5_get_mission_snap(&mission);

    /* Casts (int32_t)((uint32_t)dist_um - ...): difference of two wrapping
       32-bit counters, read as signed: they are less than 2^31 apart;
       (uint32_t)dist_um, (uint32_t)p_st->prev_dist_um: distances as a wrapping
       32-bit pattern for the difference. */
    dist_um = odom.dist_mm * 1000;
    ds_um   = p_st->b_have_dist
                  ? (int32_t)((uint32_t)dist_um - (uint32_t)p_st->prev_dist_um)
                  : 0;

    p_st->prev_dist_um = dist_um;
    p_st->b_have_dist  = true;
    now_ms             = app_time_ms_hw();

    if (mission.runs != p_st->prev_runs)
    {
        p_st->prev_runs = mission.runs;
        imu_core_new_run(&g_ic);
    }

    /* ---- Calibration ---- */
    if (mission.cal_phase != p_st->prev_phase)
    {
        imu_cal_phase(p_st, mission.cal_phase, now_ms);
        p_st->prev_phase = mission.cal_phase;
    }

    imu_cal_sample(p_st, mission.cal_phase, now_ms, acc, p_mag);

    /* ---- 14 Nov: humps while the floor may have gone ---- */
    p_st->b_air_start = false;
    p_st->b_release   = false;
    p_st->safety_bits = 0u;

    if (imu_floor_check(p_st))
    {
        ds_um = 0; /* no hump progress in the air */
    }

    /* ---- IMU processing ---- */
    events =
        imu_core_step(&g_ic, acc, p_mag, ds_um, odom.v_mm_s,
                      odom.yaw_rate_cdeg_s, now_ms - p_st->prev_ms, now_ms);

    p_st->prev_ms = now_ms;

    imu_report(p_st, events, &mission);

    /* ---- Snapshot ---- */
    imu_snapshot(p_st, &odom);

    imu_debug_logs(p_st, acc);

    subsystem4_put_imu_snap(&p_st->snap);

    if (p_st->safety_bits != 0u)
    {
        (void)tk_set_flg(subsystem4_flg_safety(), p_st->safety_bits);
    }
}

/** Calibration: MissionTask entered phase new_phase. */
static void imu_cal_phase (imu_task_state_t * p_st, uint8_t new_phase,
                           uint32_t now_ms)
{
    if (MISSION_CAL_PH_STILL == new_phase)
    {
        imu_core_cal_level_start(&g_ic);
        p_st->still_ms = now_ms;
    }
    else if (MISSION_CAL_PH_SPIN == new_phase)
    {
        (void)imu_core_cal_level_finish(&g_ic);
        imu_core_cal_mag_start(&g_ic);
    }
    else if (MISSION_CAL_PH_FINISH == new_phase)
    {
        imu_cal_finish(p_st->prev_phase);
    }
    else
    {
        /* No action required. */
    }
}

/** Calibration finished: close the phase that was running, report. */
static void imu_cal_finish (uint8_t prev_phase)
{
    bool b_level;

    if (MISSION_CAL_PH_STILL == prev_phase)
    {
        (void)imu_core_cal_level_finish(&g_ic);
    }
    else
    {
        (void)imu_core_cal_mag_finish(&g_ic);
    }

    b_level = ((g_ic.cal & IMU_CAL_LEVEL) != 0u);

    (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                IMU_LOG_LVL_N, g_ic.lvl_n);
    (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                IMU_LOG_LVL_SPREAD, g_ic.lvl_spread_mg);
    (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                IMU_LOG_LVL_UP, g_ic.lvl_up_mg);

    (void)subsystem5_mission_post(MISSION_EV_CAL_DONE, APP_SRC_SAFETY_IMU,
                                  APP_SRC_SAFETY_IMU, b_level ? 1 : 0);
}

/** Calibration: feed this cycle's sample to the running phase. */
static void imu_cal_sample (imu_task_state_t const * p_st, uint8_t phase,
                            uint32_t now_ms, int32_t const acc[IMU_AXES],
                            int16_t const * p_mag)
{
    if ((MISSION_CAL_PH_STILL == phase)
        && (app_types_elapsed_ms(now_ms, p_st->still_ms)
            >= IMU_LEVEL_SETTLE_MS))
    {
        /* low-pass filtered (IMU_ACC_LPF_DIV), not single samples */
        int32_t filt[IMU_AXES];
        uint8_t kdx;

        for (kdx = 0u; kdx < IMU_AXES; kdx++)
        {
            filt[kdx] = g_ic.b_have_acc ? (g_ic.acc_f[kdx] / 16) : acc[kdx];
        }

        imu_core_cal_level_sample(&g_ic, filt);
    }
    else if ((MISSION_CAL_PH_SPIN == phase) && (p_mag != NULL))
    {
        imu_core_cal_mag_sample(&g_ic, p_mag);
    }
    else
    {
        /* No action required. */
    }
}

/** All three IR sensors black: the robot may be in the air (wheels
   spinning, chassis tipped: read as humps).  Keep the hump results as they
   are; if it turns out lifted, undo every hump since and stop measuring
   humps until it is back on the floor.  Returns true while there is no
   floor under the IR sensors. */
static bool imu_floor_check (imu_task_state_t * p_st)
{
    line_types_snap_t line;
    bool              b_black;
    bool              b_air;

    subsystem3_get_snap(&line);
    b_black = (line.sens & LINE_SENS_ALL_BLACK) != 0u;
    b_air   = (line.sens & LINE_SENS_NO_FLOOR) != 0u;

    if (b_black && !p_st->b_was_black)
    {
        imu_core_hump_save(&g_ic);
    }

    if (b_air && !p_st->b_was_air)
    {
        imu_core_hump_restore(&g_ic);
        p_st->b_air_start = true;
        p_st->held_n      = 0u; /* humps measured in the air: dropped */
    }

    if (!b_black && p_st->b_was_black && !p_st->b_was_air)
    {
        imu_core_hump_drop(&g_ic);
        p_st->b_release = true; /* it was the track, not a lift */
    }
    else if (!b_black && p_st->b_was_black)
    {
        p_st->held_n = 0u; /* back on the floor after a lift */
    }
    else
    {
        /* still black, or still not */
    }

    p_st->b_was_black = b_black;
    p_st->b_was_air   = b_air;
    return b_air;
}

/** Events of this cycle: mission events, safety bits and LOG records. */
static void imu_report (imu_task_state_t * p_st, uint8_t events,
                        mission_types_snap_t const * p_mission)
{
    if (p_st->b_release)
    {
        imu_hump_release(p_st);
    }

    if ((events & IMU_EV_HUMP_START) != 0u)
    {
        imu_hump_event(p_st, MISSION_EV_HUMP_START, 0u);
    }

    if ((events & IMU_EV_HUMP_END) != 0u)
    {
        imu_hump_event(p_st, MISSION_EV_HUMP_END, g_ic.last_hump_mm);
    }

    imu_safety_bits(p_st, events, p_mission);
    imu_log_events(events);
}

/** A hump event for MissionTask (and the dashboard).  While all three IR
    sensors are black the robot may have been lifted, so the event is held:
    it is posted when the floor comes back, and dropped if the sensors find
    no floor (humps are only recorded on the ground, 20 Nov). */
static void imu_hump_event (imu_task_state_t * p_st, uint8_t type,
                            uint16_t peak_mm)
{
    if (!p_st->b_was_black)
    {
        (void)subsystem5_mission_post(type, APP_SRC_SAFETY_IMU, 0u, peak_mm);
    }
    else if (p_st->held_n < IMU_HELD_MAX)
    {
        p_st->held_type[p_st->held_n] = type;
        p_st->held_mm[p_st->held_n]   = peak_mm;
        p_st->held_n++;
    }
    else
    {
        /* more than IMU_HELD_MAX in one black stretch: not a real hump */
    }
}

/** The floor came back without a lift: post the held hump events in order.
 */
static void imu_hump_release (imu_task_state_t * p_st)
{
    uint8_t idx;

    for (idx = 0u; idx < p_st->held_n; idx++)
    {
        (void)subsystem5_mission_post(p_st->held_type[idx], APP_SRC_SAFETY_IMU,
                                      0u, p_st->held_mm[idx]);
    }

    p_st->held_n = 0u;
}

/** 14 Nov: tilt (overturned) and lift brake the motors while a run is
   active (SafetyTask latches the fault); idle, they are only logged, so
   handling the robot does not latch a fault.  The bits are set after the
   snapshot, so the run summary (MissionTask, core 1) reads the humps as
   corrected for a lift. */
static void imu_safety_bits (imu_task_state_t * p_st, uint8_t events,
                             mission_types_snap_t const * p_mission)
{
    bool const b_run = (p_mission->state >= MISSION_ST_LINE_FOLLOW)
                       && (p_mission->state <= MISSION_ST_LINE_SEARCH);

    if (b_run && g_ic.b_tilt && !p_st->b_tilt_sent)
    {
        p_st->safety_bits |= SAFETY_FLT_TILT;
    }

    p_st->b_tilt_sent = b_run && g_ic.b_tilt;

    if (b_run && (((events & IMU_EV_LIFT) != 0u) || p_st->b_air_start))
    {
        /* IMU lift, or no floor under the IR sensors */
        p_st->safety_bits |= SAFETY_FLT_LIFT;
    }
}

/** LOG records for the lift, tilt, motion and ramp events. */
static void imu_log_events (uint8_t events)
{
    if ((events & IMU_EV_LIFT) != 0u)
    {
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                    IMU_LOG_LIFT, g_ic.lift_rise_mg);
    }

    if ((events & IMU_EV_RISE_MAX) != 0u)
    {
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                    IMU_LOG_RISE_MAX, g_ic.rise_max_mg);
    }

    if ((events & IMU_EV_TILT_ON) != 0u)
    {
        (void)subsystem1_telem_post(
            COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, IMU_LOG_TILT_ON,
            app_types_max_i32(app_types_abs_i32(g_ic.pitch_cdeg),
                              app_types_abs_i32(g_ic.roll_cdeg)));
    }

    if ((events & IMU_EV_TILT_OFF) != 0u)
    {
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                    IMU_LOG_TILT_OFF, 0);
    }

    if ((events & IMU_EV_MOTION) != 0u)
    {
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                    IMU_LOG_MOTION, g_ic.motion);
    }

    if ((events & IMU_EV_RAMP) != 0u)
    {
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                    IMU_LOG_RAMP, 0);
    }
}

/** Fill the IMU snapshot (IF-08) from the IMU core and the odometry. */
static void imu_snapshot (imu_task_state_t *          p_st,
                          motion_types_odom_t const * p_odom)
{
    /* Casts (int16_t)g_ic.pitch_cdeg, (int16_t)g_ic.roll_cdeg: a wrapped angle:
       +-18000 0.01 deg; (uint16_t)g_ic.heading_cdeg: the heading is 0..35999
       (imu_core wraps it); (int16_t)(g_ic.prev_mag_mg - 1000): with the +-4 g
       sensor |a| <= 6925 mg, so -1000..5925. */
    p_st->snap.t_ms             = app_time_ms();
    p_st->snap.pitch_cdeg       = (int16_t)g_ic.pitch_cdeg;
    p_st->snap.roll_cdeg        = (int16_t)g_ic.roll_cdeg;
    p_st->snap.heading_cdeg     = (uint16_t)g_ic.heading_cdeg;
    p_st->snap.turn_rate_cdeg_s = p_odom->yaw_rate_cdeg_s;
    p_st->snap.accel_mg         = (int16_t)(g_ic.prev_mag_mg - 1000);
    p_st->snap.tilt             = g_ic.b_tilt ? 1u : 0u;
    p_st->snap.hump_max_mm      = g_ic.hump_max_mm;
    p_st->snap.last_hump_mm     = g_ic.last_hump_mm;
    p_st->snap.humps            = g_ic.humps;
    p_st->snap.hump_state       = g_ic.hump_state;
    p_st->snap.motion           = g_ic.motion;
    p_st->snap.cal              = g_ic.cal;
    p_st->snap.b_ok             = p_st->b_fault ? 0u : 1u;
    p_st->snap.pitch_valid =
        (!p_st->b_fault && (p_st->good >= IMU_RECOVER_SAMPLES)) ? 1u : 0u;
}

/** Bring-up only (IMU_DEBUG_LOGS in imu_cfg.h): raw acceleration, pitch and
   roll as LOG 100..104 on the first good sample, then every 100 cycles.
   Off by default: five QoS 1 events per second exceed lwIP's MQTT
   in-flight limit and fill the laptop's log. */
static void imu_debug_logs (imu_task_state_t const * p_st,
                            int32_t const            acc[IMU_AXES])
{
#if IMU_DEBUG_LOGS
    static bool b_first_sample = true;

    if (b_first_sample || (0u == (p_st->cycle % 100u)))
    {
        b_first_sample = false;

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 100u,
                                    acc[0]);

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 101u,
                                    acc[1]);

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 102u,
                                    acc[2]);

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 103u,
                                    (int32_t)p_st->snap.pitch_cdeg);

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 104u,
                                    (int32_t)p_st->snap.roll_cdeg);
    }
#else
    (void)p_st; /* used by the bring-up logs only */
    (void)acc;
#endif
}

/** Initialise I2C1 and the LSM303.  Returns true if both succeeded; the
    results are logged (LOG 90..93 and 53) the first time and whenever they
    change. */
static bool imu_hw_init (void)
{
    static bool     b_reported = false;
    static ER       last_bus;
    static ER       last_read;
    static ER       last_sensor;
    static uint8_t  last_id;
    static uint32_t last_diag;

    uint8_t  chip_id    = 0u;
    ER       bus_er     = drv_i2c1_init();
    ER       read_er    = E_IO;
    ER       sensor_er  = E_IO;
    uint32_t diagnostic = 0xFF000000u;

    if (E_OK == bus_er)
    {
        read_er    = drv_i2c1_read(DRV_LSM303_ADDR, 0x0Fu, &chip_id, 1u);
        sensor_er  = drv_lsm303_init();
        diagnostic = drv_lsm303_diag();
    }

    /* Report once initially, then only when a result changes. */
    if (!b_reported || (bus_er != last_bus) || (read_er != last_read)
        || (sensor_er != last_sensor) || (chip_id != last_id)
        || (diagnostic != last_diag))
    {
        b_reported  = true;
        last_bus    = bus_er;
        last_read   = read_er;
        last_sensor = sensor_er;
        last_id     = chip_id;
        last_diag   = diagnostic;

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 90u,
                                    bus_er);

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 91u,
                                    read_er);

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 92u,
                                    chip_id);

        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU, 93u,
                                    sensor_er);

        /* Cast (int32_t)diagnostic: bit pattern of the packed diagnosis
           word. */
        (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_SAFETY_IMU,
                                    IMU_LOG_INIT, (int32_t)diagnostic);
    }

    return (E_OK == bus_er) && (E_OK == sensor_er);
}

/*** end of file ***/
