/*
 *  subsystem4_test.c
 *  On-target tests for subsystem 4.  Built only with: make TEST=1
 *  Record results in testing/subsystem4/TEST_LOG.md.
 *  IMU and heartbeat logic are tested on a PC (testing/subsystem4/host).
 *
 *  S4-U08/U09 raise a real fault and brake the motors, then reset it as
 *  the dashboard Reset button does.  They run last in subsystem 4, after
 *  the motion tests of subsystem 2.  S4-U07 wants the robot on a level
 *  surface (on the stand is fine if the stand is level).
 */

#include "subsystem4.h"
#include "../subsystem2/subsystem2.h"       /* motor_estop timestamp, odometry */
#include "../subsystem5/subsystem5.h"       /* mission_post / mission_get_snap */
#include "../test_report.h"

#if APP_TEST

#define WD_CTRL_REG     0x40058000u
#define SIO_GPIO_OUT    (SIO_BASE + 0x010u)

#define T_NFR8_US       20000u      /* NFR8: brake within 20 ms of a fault       */
#define T_LEVEL_CDEG    500         /* level bench: |pitch|, |roll| <= 5.0 deg   */
#define T_FAULT_BIT     SAFETY_FLT_STALL  /* not re-checked on reset (live_causes) */

static bool brake_pins_low(void)
{
    UW const mask = (1uL << S2_PIN_MOT_L_A) | (1uL << S2_PIN_MOT_L_B)
                  | (1uL << S2_PIN_MOT_R_A) | (1uL << S2_PIN_MOT_R_B);

    return ((in_w(SIO_GPIO_OUT) & mask) == 0u) &&
           ((in_w(GPIO_CTRL(S2_PIN_MOT_L_A)) & 0x1Fu) == GPIO_CTRL_FUNCSEL_SIO) &&
           ((in_w(GPIO_CTRL(S2_PIN_MOT_L_B)) & 0x1Fu) == GPIO_CTRL_FUNCSEL_SIO) &&
           ((in_w(GPIO_CTRL(S2_PIN_MOT_R_A)) & 0x1Fu) == GPIO_CTRL_FUNCSEL_SIO) &&
           ((in_w(GPIO_CTRL(S2_PIN_MOT_R_B)) & 0x1Fu) == GPIO_CTRL_FUNCSEL_SIO);
}

static int32_t iabs(int32_t v)
{
    return (v < 0) ? -v : v;
}

IMPORT volatile UW g_safety_tick_us;   /* safety_task.c */

EXPORT void subsystem4_test(void)
{
    T_RFLG      rflg;
    uint32_t    before;
    imu_snap_t  a;
    imu_snap_t  b;

    TEST_INIT();

    TEST_CHECK("S4-U01", tk_ref_flg(g_flg_safety, &rflg) == E_OK, "flg_safety exists");

    before = safety_heartbeat_count(SAFETY_HB_MOTION);
    (void)tk_dly_tsk(50);
    TEST_CHECK("S4-U02", safety_heartbeat_count(SAFETY_HB_MOTION) >= (before + 3u),
               "MotionTask heartbeat advances");

    imu_get_snap(&a);
    (void)tk_dly_tsk(100);
    imu_get_snap(&b);
    TEST_CHECK("S4-U04", b.t_ms != a.t_ms, "ImuTask publishes its snapshot");
    TEST_CHECK("S4-U05", safety_timer_tick_ok(), "1 us timer ticks at 1 MHz (watchdog tick = 12)");
    tm_printf((UB *)"[INFO] timer: %d us per 100 ms of CPU clock\n", (INT)g_safety_tick_us);
    TEST_CHECK("S4-U06", (in_w(WD_CTRL_REG) & (1uL << 30)) != 0u, "watchdog enabled and being fed");

    /* ---- S4-U07: IMU reads level, no I2C errors in 1 s ----------------- */
    imu_get_snap(&a);
    (void)tk_dly_tsk(1000);
    imu_get_snap(&b);
    if (b.ok == 0u)
    {
        TEST_SKIP("S4-U07", "IMU level and I2C clean", "IMU not answering: wiring, or emulator");
    }
    else
    {
        tm_printf((UB *)"[INFO] S4-U07 pitch %d cdeg, roll %d cdeg, |a|-1g %d mg, I2C errors %d in 1 s, cal 0x%x\n",
                  (INT)b.pitch_cdeg, (INT)b.roll_cdeg, (INT)b.accel_mg,
                  (INT)(b.i2c_errors - a.i2c_errors), (INT)b.cal);
        TEST_CHECK("S4-U07", (iabs(b.pitch_cdeg) <= T_LEVEL_CDEG) && (iabs(b.roll_cdeg) <= T_LEVEL_CDEG) &&
                             (b.i2c_errors == a.i2c_errors) && (b.tilt == 0u),
                   "IMU reads level (pitch, roll within 5 deg), no I2C errors in 1 s");
    }

    /* ---- S4-U08: fault bit -> both motors braked (NFR8) ---------------- */
    {
        mission_snap_t ms;
        motion_odom_t  o;
        uint32_t       t0;
        uint32_t       lat;
        uint32_t       t;
        bool           b_ok = false;
        uint8_t        pre;

        mission_get_snap(&ms);
        pre = ms.state;
        g_estop_test_us = 0u;
        t0 = (uint32_t)app_time_us();
        (void)tk_set_flg(g_flg_safety, T_FAULT_BIT);
        (void)tk_dly_tsk(50);
        lat = g_estop_test_us - t0;
        motion_get_odom(&o);
        tm_printf((UB *)"[INFO] S4-U08 fault flag -> motor_estop() %d us (limit %d us)\n",
                  (INT)lat, (INT)T_NFR8_US);
        TEST_CHECK("S4-U08", (g_estop_test_us != 0u) && (lat <= T_NFR8_US) && motion_estop_latched() &&
                             brake_pins_low() && ((safety_get_latched() & T_FAULT_BIT) != 0u),
                   "fault flag brakes both motors within 20 ms (GP8-11 SIO, LOW; latched)");

        /* ---- S4-U09: Reset clears it, as the dashboard button does ---- */
        (void)mission_post((uint8_t)MISSION_EV_REMOTE_RESET, (uint8_t)APP_SRC_SAFETY_IMU, 0u, 0);
        for (t = 0u; t < 8000u; t += 50u)
        {
            (void)tk_dly_tsk(50);
            mission_get_snap(&ms);
            if (!motion_estop_latched() && (safety_get_latched() == 0u) &&
                ((pre != (uint8_t)MISSION_ST_IDLE) || (ms.state == (uint8_t)MISSION_ST_IDLE)))
            {
                b_ok = true;
                break;
            }
        }
        motion_get_odom(&o);
        tm_printf((UB *)"[INFO] S4-U09 cleared after %d ms; mission state %d (was %d), brake %d, latched 0x%x\n",
                  (INT)t, (INT)ms.state, (INT)pre, (INT)(motion_estop_latched() ? 1 : 0),
                  (INT)safety_get_latched());
        TEST_CHECK("S4-U09", b_ok && (o.state == (uint8_t)MOTION_ST_IDLE_BRAKED) &&
                             ((in_w(GPIO_CTRL(S2_PIN_MOT_L_A)) & 0x1Fu) == GPIO_CTRL_FUNCSEL_PWM),
                   "Reset clears the latch, PWM reconnected, mission back to IDLE (if it was IDLE)");
    }

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
