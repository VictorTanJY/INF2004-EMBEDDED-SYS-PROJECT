/*
 *  subsystem2.c
 *  Subsystem 2 -- Motion control.  Owner: Buddy 2 (Jedrek).
 *
 *  STATUS: shared-base stub.  Creates the objects other subsystems use
 *  (mbf_motion, the odometry snapshot) and provides motor_estop().
 *  TODO (owner): motion_cyc, MotionTask, drv_motor (PWM 20 kHz, TOP 6249,
 *  DIV 1; write both CC halves in one 32-bit store), drv_encoder (PIO0
 *  quadrature at offset 0; check that offset 0 is free first), PID.
 *  Remember: PWM is held in reset at boot (USE_PTMR = 0), so release
 *  RESETS_RESET_PWM in drv_motor before using it.
 */

#include "subsystem2.h"

EXPORT ID g_mbf_motion;

static app_lock_t       g_odom_lock;
static motion_odom_t    g_odom;

/* Written by motor_estop() (SafetyTask) and cleared by MotionTask, both
   on core 1.  A byte store/load is indivisible on the Cortex-M0+. */
static volatile uint8_t g_estop_latch;

EXPORT ER subsystem2_init(void)
{
    T_CMBF  cmbf = {0};

    app_lock_init(&g_odom_lock);

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S2_MBF_MOTION_DEPTH * (sizeof(INT) + sizeof(motion_cmd_t));
    cmbf.maxmsz = sizeof(motion_cmd_t);
    g_mbf_motion = tk_cre_mbf(&cmbf);

    return (g_mbf_motion < E_OK) ? g_mbf_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
    /* TODO (owner): create motion_cyc and start MotionTask here:
       app_start_task(motion_task, S2_PRI_MOTION, S2_CORE_MOTION, "motion"); */
    return E_OK;
}

EXPORT ER motion_send(motion_cmd_t const * p_cmd)
{
    return tk_snd_mbf(g_mbf_motion, (void *)p_cmd, sizeof(*p_cmd), TMO_POL);
}

EXPORT void motion_get_odom(motion_odom_t * p_out)
{
    UINT sts = app_lock(&g_odom_lock);

    *p_out = g_odom;
    app_unlock(&g_odom_lock, sts);
}

EXPORT void motion_put_odom(motion_odom_t const * p_in)
{
    UINT sts = app_lock(&g_odom_lock);

    g_odom = *p_in;
    app_unlock(&g_odom_lock, sts);
}

EXPORT void motor_estop(void)
{
    UW const mask = (1uL << S2_PIN_MOT_L_A) | (1uL << S2_PIN_MOT_L_B)
                  | (1uL << S2_PIN_MOT_R_A) | (1uL << S2_PIN_MOT_R_B);
    UINT     intsts;

    DI(intsts);
    g_estop_latch = 1u;
    out_w(GPIO_OUT_CLR, mask);      /* LOW first, so no glitch HIGH  */
    out_w(GPIO_OE_SET, mask);       /* drive the pins                 */
    out_w(GPIO_CTRL(S2_PIN_MOT_L_A), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(S2_PIN_MOT_L_B), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(S2_PIN_MOT_R_A), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(S2_PIN_MOT_R_B), GPIO_CTRL_FUNCSEL_SIO);
    EI(intsts);
}

EXPORT bool motion_estop_latched(void)
{
    return (g_estop_latch != 0u);
}
