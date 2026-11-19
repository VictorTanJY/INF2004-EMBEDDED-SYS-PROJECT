/** @file subsystem4_test.c
 *
 *  @brief On-target tests for subsystem 4.  Built only with: make TEST=1
 *  Record results in testing/subsystem4/TEST_LOG.md.
 *  IMU and heartbeat logic are tested on a PC (testing/subsystem4/host).
 *
 *  S4-U08/U09 raise a real fault and brake the motors, then reset it as
 *  the dashboard Reset button does.  They run last in subsystem 4, after
 *  the motion tests of subsystem 2.  S4-U07 wants the robot on a level
 *  surface (on the stand is fine if the stand is level).
 */

#include "subsystem4_test.h"
#include "safety_task.h"
#include "subsystem4.h"

/* subsystem2_motor_estop timestamp, odometry */
#include "../subsystem2/subsystem2.h"

/* subsystem5_mission_post / subsystem5_get_mission_snap */
#include "../subsystem5/subsystem5.h"
#include "../test_report.h"

#define T_RESET_WAIT_MS 8000u /* MissionTask reaches IDLE well within */

#if APP_TEST

#define WD_CTRL_REG  0x40058000u
#define SIO_GPIO_OUT (APP_REGS_SIO_BASE + 0x010u)

#define T_NFR8_US    20000u /* NFR8: brake within 20 ms of a fault       */
#define T_LEVEL_CDEG 500    /* level bench: |pitch|, |roll| <= 5.0 deg   */

/* not re-checked on reset (live_causes) */
#define T_FAULT_BIT SAFETY_FLT_STALL

#endif

#if APP_TEST

#endif

#if APP_TEST
static bool    brake_pins_low(void);
static int32_t iabs(int32_t value);
static void    test_estop(test_report_t * p_report);
#endif

#if APP_TEST

EXPORT void subsystem4_test_run (void)
{
    T_RFLG           rflg;
    uint32_t         before;
    imu_types_snap_t snap_a;
    imu_types_snap_t snap_b;

    test_report_t report = {0, 0, 0};

    TEST_CHECK(&report, "S4-U01",
               E_OK == tk_ref_flg(subsystem4_flg_safety(), &rflg),
               "flg_safety exists");

    before = subsystem4_heartbeat_count(SAFETY_HB_MOTION);
    (void)tk_dly_tsk(50);
    TEST_CHECK(&report, "S4-U02",
               ((subsystem4_heartbeat_count(SAFETY_HB_MOTION) - before) & 0xFFu)
                   >= 3u,
               "MotionTask heartbeat advances (8-bit counter, modular)");

    /* Casts (UB *)"[INFO] timer: %d us per ...: tm_printf() takes UB *; the
       string bytes are unchanged; (INT)safety_task_tick_us(): printed with %d:
       the value is a 32-bit count or measurement below 2^31. */
    subsystem4_get_imu_snap(&snap_a);
    (void)tk_dly_tsk(100);
    subsystem4_get_imu_snap(&snap_b);
    TEST_CHECK(&report, "S4-U04", snap_b.t_ms != snap_a.t_ms,
               "ImuTask publishes its snapshot");
    TEST_CHECK(&report, "S4-U05", safety_task_timer_tick_ok(),
               "1 us timer ticks at 1 MHz (watchdog tick = 12)");
    tm_printf((UB *)"[INFO] timer: %d us per 100 ms of CPU clock\n",
              (INT)safety_task_tick_us());
    TEST_CHECK(&report, "S4-U06", (in_w(WD_CTRL_REG) & (1uL << 30)) != 0u,
               "watchdog enabled and being fed");

    /* ---- S4-U07: IMU reads level, no I2C errors in 1 s ----------------- */
    subsystem4_get_imu_snap(&snap_a);
    (void)tk_dly_tsk(1000);
    subsystem4_get_imu_snap(&snap_b);
    if (0u == snap_b.b_ok)
    {
        test_report_skip(&report, "S4-U07", "IMU level and I2C clean",
                         "IMU not answering: wiring, or emulator");
    }
    else
    {
        /* Casts (UB *)"[INFO] S4-U07 pitch %d ...: tm_printf() takes UB *; the
           string bytes are unchanged; (INT)snap_b.pitch_cdeg,
           (INT)snap_b.roll_cdeg, (INT)snap_b.accel_mg: every int16_t value
           fits; (INT)snap_b.cal: every uint8_t value fits. */
        tm_printf(
            (UB *)"[INFO] S4-U07 pitch %d cdeg, roll %d cdeg, |a|-1g %d mg, "
                  "I2C errors %d in 1 s, cal 0x%x\n",
            (INT)snap_b.pitch_cdeg, (INT)snap_b.roll_cdeg, (INT)snap_b.accel_mg,
            (snap_b.i2c_errors - snap_a.i2c_errors), (INT)snap_b.cal);
        TEST_CHECK(
            &report, "S4-U07",
            (iabs(snap_b.pitch_cdeg) <= T_LEVEL_CDEG)
                && (iabs(snap_b.roll_cdeg) <= T_LEVEL_CDEG)
                && (snap_b.i2c_errors == snap_a.i2c_errors)
                && (0u == snap_b.tilt),
            "IMU reads level (pitch, roll within 5 deg), no I2C errors in 1 s");
    }

    test_estop(&report);

    test_report_summary(&report, "subsystem4");
}

#endif

#if APP_TEST

/** S4-U08 / U09: a fault bit brakes both motors; Reset clears it. */
static void test_estop (test_report_t * p_report)
{
    /* ---- S4-U08: fault bit -> both motors braked (NFR8) ---------------- */
    mission_types_snap_t mission;
    motion_types_odom_t  odom;
    uint32_t             t_start;
    uint32_t             lat;
    uint32_t             elapsed;
    bool                 b_ok = false;
    uint8_t              pre;

    /* Casts (UB *)"[INFO] S4-U08 fault flag ...: tm_printf() takes UB *; the
       string bytes are unchanged; (INT)lat: printed with %d: the value is a
       32-bit count or measurement below 2^31; (INT)T_NFR8_US: a constant that
       fits. */
    subsystem5_get_mission_snap(&mission);
    pre = mission.state;
    subsystem2_estop_test_clear();
    t_start = app_time_us();
    (void)tk_set_flg(subsystem4_flg_safety(), T_FAULT_BIT);
    (void)tk_dly_tsk(50);
    lat = subsystem2_estop_test_us() - t_start;
    subsystem2_get_odom(&odom);
    tm_printf((UB *)"[INFO] S4-U08 fault flag -> motor_estop() %d us "
                    "(limit %d us)\n",
              (INT)lat, (INT)T_NFR8_US);
    TEST_CHECK(p_report, "S4-U08",
               (subsystem2_estop_test_us() != 0u) && (lat <= T_NFR8_US)
                   && subsystem2_estop_latched() && brake_pins_low()
                   && ((subsystem4_get_latched() & T_FAULT_BIT) != 0u),
               "fault flag brakes both motors within 20 ms (GP8-11 SIO, "
               "LOW; latched)");

    /* ---- S4-U09: Reset clears it, as the dashboard button does ---- */
    (void)subsystem5_mission_post(MISSION_EV_REMOTE_RESET, APP_SRC_SAFETY_IMU,
                                  0u, 0);
    for (elapsed = 0u; elapsed < T_RESET_WAIT_MS; elapsed += 50u)
    {
        (void)tk_dly_tsk(50);
        subsystem5_get_mission_snap(&mission);
        if (!subsystem2_estop_latched() && (0u == subsystem4_get_latched())
            && ((pre != MISSION_ST_IDLE) || (MISSION_ST_IDLE == mission.state)))
        {
            b_ok = true;
            break;
        }
    }
    /* Casts (UB *)"[INFO] S4-U09 cleared ...: tm_printf() takes UB *; the
       string bytes are unchanged; (INT)elapsed, (INT)subsystem4_get_latched():
       printed with %d: the value is a 32-bit count or measurement below 2^31;
       (INT)mission.state, (INT)pre: every uint8_t value fits. */
    subsystem2_get_odom(&odom);
    tm_printf((UB *)"[INFO] S4-U09 cleared after %d ms; mission state %d "
                    "(was %d), brake %d, latched 0x%x\n",
              (INT)elapsed, (INT)mission.state, (INT)pre,
              (subsystem2_estop_latched() ? 1 : 0),
              (INT)subsystem4_get_latched());
    TEST_CHECK(p_report, "S4-U09",
               b_ok && (MOTION_ST_IDLE_BRAKED == odom.state)
                   && (APP_REGS_FUNCSEL_PWM
                       == (in_w(app_regs_gpio_ctrl(S2_PIN_MOT_L_A)) & 0x1Fu)),
               "Reset clears the latch, PWM reconnected, mission back to "
               "IDLE (if it was IDLE)");
}

/** True if GP8-GP11 are SIO outputs driven low (the emergency brake). */
static bool brake_pins_low (void)
{
    uint32_t const mask = (1uL << S2_PIN_MOT_L_A) | (1uL << S2_PIN_MOT_L_B)
                          | (1uL << S2_PIN_MOT_R_A) | (1uL << S2_PIN_MOT_R_B);

    return (0u == (in_w(SIO_GPIO_OUT) & mask))
           && (APP_REGS_FUNCSEL_SIO
               == (in_w(app_regs_gpio_ctrl(S2_PIN_MOT_L_A)) & 0x1Fu))
           && (APP_REGS_FUNCSEL_SIO
               == (in_w(app_regs_gpio_ctrl(S2_PIN_MOT_L_B)) & 0x1Fu))
           && (APP_REGS_FUNCSEL_SIO
               == (in_w(app_regs_gpio_ctrl(S2_PIN_MOT_R_A)) & 0x1Fu))
           && (APP_REGS_FUNCSEL_SIO
               == (in_w(app_regs_gpio_ctrl(S2_PIN_MOT_R_B)) & 0x1Fu));
}

/** Absolute value (value must not be INT32_MIN). */
static int32_t iabs (int32_t value)
{
    return (value < 0) ? -value : value;
}

#endif

/*** end of file ***/
