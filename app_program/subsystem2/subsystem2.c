/*
 *  subsystem2.c
 *  Subsystem 2 -- Motion control: kernel objects, odometry snapshot and
 *  the E-stop latch.  Owner: Buddy 2 (Jedrek).
 *
 *  The task is in motion_task.c, the controller in motion_core.c, the
 *  drivers in drv_motor.c / drv_encoder.c.
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "../subsystem4/subsystem4.h"

EXPORT ID g_mbf_motion;

static app_lock_t       g_odom_lock;
static motion_odom_t    g_odom;
static ID               g_tid_motion;
static ID               g_cyc_motion;

/* Written by motor_estop() (SafetyTask) and cleared by MotionTask, both
   on core 1.  A byte store/load is indivisible on the Cortex-M0+, and both
   writers run inside DI/EI on the same core, so no update is lost. */
static volatile uint8_t g_estop_latch;

/* motion_cyc: every 10 ms, in core 0's tick interrupt; wakes MotionTask. */
static void motion_cyc_handler(void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_tid_motion);
}

EXPORT ER subsystem2_init(void)
{
    T_CMBF  cmbf = {0};
    T_CCYC  ccyc = {0};

    app_lock_init(&g_odom_lock);

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = S2_MBF_MOTION_DEPTH * (sizeof(INT) + sizeof(motion_cmd_t));
    cmbf.maxmsz = sizeof(motion_cmd_t);
    g_mbf_motion = tk_cre_mbf(&cmbf);
    if (g_mbf_motion < E_OK)
    {
        return g_mbf_motion;
    }

    ccyc.cycatr = TA_HLNG;                  /* started in subsystem2_start() */
    ccyc.cychdr = (FP)motion_cyc_handler;
    ccyc.cyctim = S2_PERIOD_MS;
    ccyc.cycphs = S2_PERIOD_MS;
    g_cyc_motion = tk_cre_cyc(&ccyc);
    return (g_cyc_motion < E_OK) ? g_cyc_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
    g_tid_motion = app_start_task((FP)motion_task, S2_PRI_MOTION, S2_CORE_MOTION, "motion");
    if (g_tid_motion < E_OK)
    {
        return g_tid_motion;
    }
    return tk_sta_cyc(g_cyc_motion);
}

EXPORT ER motion_send(motion_cmd_t const * p_cmd)
{
    return tk_snd_mbf(g_mbf_motion, (void *)p_cmd, sizeof(*p_cmd), TMO_POL);
}

EXPORT ER motion_post(uint8_t src, uint8_t seq, uint8_t cmd, int16_t p0, int16_t p1)
{
    motion_cmd_t c;

    c.src      = src;
    c.seq      = seq;
    c.cmd      = cmd;
    c.reserved = 0u;
    c.p0       = p0;
    c.p1       = p1;
    return motion_send(&c);
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

/*
 * CLEAR_ESTOP (Tribunal 1 gap 3): the latch check, the latch clear and the
 * PWM reconnect run inside ONE DI/EI section on core 1, and only while
 * SafetyTask's latched-fault word is zero, so a fault raised meanwhile
 * cannot be undone by a reconnect already in progress.
 */
EXPORT bool motion_estop_clear_and_reconnect(void)
{
    bool b_ok = false;
    UINT intsts;

    DI(intsts);
    if ((g_estop_latch != 0u) && (safety_get_latched() == 0u))
    {
        drv_motor_write(0, 0);
        drv_motor_connect();
        g_estop_latch = 0u;
        b_ok = true;
    }
    EI(intsts);
    return b_ok;
}
