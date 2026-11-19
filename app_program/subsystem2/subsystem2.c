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
#include "motion_core.h"
#include "../subsystem4/subsystem4.h"

EXPORT ID g_mbf_motion;

static app_lock_t g_odom_lock;
static uint32_t g_odom[MOTION_ODOM_WORDS]; /* packed (CR1), under g_odom_lock */
static ID       g_tid_motion;
static ID       g_cyc_motion;

/* Written by subsystem2_motor_estop() (SafetyTask) and cleared by MotionTask,
   both on core 1.  A byte store/load is indivisible on the Cortex-M0+, and both
   writers run inside DI/EI on the same core, so no update is lost. */
static uint8_t volatile g_estop_latch;

#if APP_TEST

EXPORT volatile uint32_t g_estop_test_us; /* written here only (core 1) */

#endif

static void motion_cyc_handler(void * p_exinf);

EXPORT ER subsystem2_init (void)
{
    T_CMBF cmbf = {0};
    T_CCYC ccyc = {0};

    app_lock_init(&g_odom_lock);
    {
        motion_types_odom_t const zero = {0};

        motion_core_odom_pack(&zero,
                              g_odom); /* packed zero (offset-binary fields) */
    }

    cmbf.mbfatr  = TA_TFIFO;
    cmbf.bufsz   = S2_MBF_MOTION_DEPTH * (sizeof(INT) + sizeof(uint32_t));
    cmbf.maxmsz  = sizeof(uint32_t); /* one packed command word (CR1) */
    g_mbf_motion = tk_cre_mbf(&cmbf);
    if (g_mbf_motion < E_OK)
    {
        return g_mbf_motion;
    }

    ccyc.cycatr  = TA_HLNG; /* started in subsystem2_start() */
    ccyc.cychdr  = (FP)motion_cyc_handler;
    ccyc.cyctim  = S2_PERIOD_MS;
    ccyc.cycphs  = S2_PERIOD_MS;
    g_cyc_motion = tk_cre_cyc(&ccyc);
    return (g_cyc_motion < E_OK) ? g_cyc_motion : E_OK;
}

EXPORT ER subsystem2_start (void)
{
    g_tid_motion = app_main_start_task((FP)motion_task_thread, S2_PRI_MOTION,
                                       S2_CORE_MOTION, "motion");
    if (g_tid_motion < E_OK)
    {
        return g_tid_motion;
    }
    return tk_sta_cyc(g_cyc_motion);
}

EXPORT ER subsystem2_send (motion_types_cmd_t const * p_cmd)
{
    uint32_t word = motion_core_cmd_pack(p_cmd);

    return tk_snd_mbf(g_mbf_motion, &word, sizeof(word), TMO_POL);
}

EXPORT ER subsystem2_post (uint8_t src, uint8_t seq, uint8_t cmd, int16_t arg0,
                           int16_t arg1)
{
    motion_types_cmd_t msg;

    msg.src      = src;
    msg.seq      = seq;
    msg.cmd      = cmd;
    msg.reserved = 0u;
    msg.arg0     = arg0;
    msg.arg1     = arg1;
    return subsystem2_send(&msg);
}

/* The snapshot is stored packed (MOTION_ODOM_WORDS words).  Writer and
   readers pack / unpack in their own task and copy the whole record under
   the lock, so no task ever does a read-modify-write on the shared words. */
EXPORT void subsystem2_get_odom (motion_types_odom_t * p_out)
{
    uint32_t words[MOTION_ODOM_WORDS];
    UINT     sts = app_lock(&g_odom_lock);
    uint32_t idx;

    for (idx = 0u; idx < MOTION_ODOM_WORDS; idx++)
    {
        words[idx] = g_odom[idx];
    }
    app_unlock(&g_odom_lock, sts);
    motion_core_odom_unpack(words, p_out);
}

EXPORT void subsystem2_put_odom (motion_types_odom_t const * p_in)
{
    uint32_t words[MOTION_ODOM_WORDS];
    UINT     sts;
    uint32_t idx;

    motion_core_odom_pack(p_in, words);
    sts = app_lock(&g_odom_lock);
    for (idx = 0u; idx < MOTION_ODOM_WORDS; idx++)
    {
        g_odom[idx] = words[idx];
    }
    app_unlock(&g_odom_lock, sts);
}

EXPORT void subsystem2_motor_estop (void)
{
    UW const mask = (1uL << S2_PIN_MOT_L_A) | (1uL << S2_PIN_MOT_L_B)
                    | (1uL << S2_PIN_MOT_R_A) | (1uL << S2_PIN_MOT_R_B);
    UINT     intsts;

    DI(intsts);
    g_estop_latch = 1u;
#if APP_TEST
    g_estop_test_us = (uint32_t)app_time_us();
#endif
    out_w(APP_REGS_GPIO_OUT_CLR, mask); /* LOW first, so no glitch HIGH  */
    out_w(APP_REGS_GPIO_OE_SET, mask);  /* drive the pins                 */
    out_w(app_regs_gpio_ctrl(S2_PIN_MOT_L_A), APP_REGS_FUNCSEL_SIO);
    out_w(app_regs_gpio_ctrl(S2_PIN_MOT_L_B), APP_REGS_FUNCSEL_SIO);
    out_w(app_regs_gpio_ctrl(S2_PIN_MOT_R_A), APP_REGS_FUNCSEL_SIO);
    out_w(app_regs_gpio_ctrl(S2_PIN_MOT_R_B), APP_REGS_FUNCSEL_SIO);
    EI(intsts);
}

EXPORT bool subsystem2_estop_latched (void)
{
    return (g_estop_latch != 0u);
}

/*
 * CLEAR_ESTOP (Tribunal 1 gap 3): the latch check, the latch clear and the
 * PWM reconnect run inside ONE DI/EI section on core 1, and only while
 * SafetyTask's latched-fault word is zero, so a fault raised meanwhile
 * cannot be undone by a reconnect already in progress.
 */
EXPORT bool subsystem2_estop_reconnect (void)
{
    bool b_ok = false;
    UINT intsts;

    DI(intsts);
    if ((g_estop_latch != 0u) && (0u == subsystem4_get_latched()))
    {
        drv_motor_write(0, 0);
        drv_motor_connect();
        g_estop_latch = 0u;
        b_ok          = true;
    }
    EI(intsts);
    return b_ok;
}

/*** end of file ***/

/* motion_cyc: every 10 ms, in core 0's tick interrupt; wakes MotionTask. */
static void motion_cyc_handler (void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_tid_motion);
}

/*** end of file ***/
