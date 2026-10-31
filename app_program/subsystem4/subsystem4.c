/*
 *  subsystem4.c
 *  Subsystem 4 -- IMU and safety: kernel objects, heartbeat words, the
 *  latched-fault word and the IMU snapshot.  Owner: Buddy 4 (Purvez).
 *  Tasks: safety_task.c, imu_task.c.  Logic: imu_core.c, safety_core.c.
 */

#include "subsystem4.h"
#include "drv_watchdog.h"

EXPORT ID g_flg_safety;
EXPORT ID g_tid_imu;

/* Written only by SafetyTask (one writer, aligned word: indivisible). */
EXPORT volatile uint32_t g_safety_latched;

static volatile uint32_t g_hb[SAFETY_HB_COUNT];
static app_lock_t        g_imu_lock;
static imu_snap_t        g_imu_snap;
static ID                g_cyc_imu;

static void imu_cyc_handler(void * p_exinf)
{
    (void)p_exinf;
    (void)tk_wup_tsk(g_tid_imu);
}

EXPORT ER subsystem4_init(void)
{
    T_CFLG  cflg = {0};
    T_CCYC  ccyc = {0};

    /* First of all: the 1 us timer and the watchdog need the 1 MHz tick. */
    drv_watchdog_tick_init();
    app_lock_init(&g_imu_lock);

    /* TA_WMUL: a stray second waiter gets queued instead of E_OBJ.  By
       rule only SafetyTask waits (see subsystem4.h). */
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;
    g_flg_safety = tk_cre_flg(&cflg);
    if (g_flg_safety < E_OK)
    {
        return g_flg_safety;
    }

    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)imu_cyc_handler;
    ccyc.cyctim = S4_IMU_PERIOD_MS;
    ccyc.cycphs = S4_IMU_PERIOD_MS;
    g_cyc_imu = tk_cre_cyc(&ccyc);
    return (g_cyc_imu < E_OK) ? g_cyc_imu : E_OK;
}

EXPORT ER subsystem4_start(void)
{
    ID tid;

    tid = app_start_task((FP)safety_task, S4_PRI_SAFETY, S4_CORE_SAFETY, "safety");
    if (tid < E_OK)
    {
        return tid;
    }
    g_tid_imu = app_start_task((FP)imu_task, S4_PRI_IMU, S4_CORE_IMU, "imu");
    if (g_tid_imu < E_OK)
    {
        return g_tid_imu;
    }
    return tk_sta_cyc(g_cyc_imu);
}

EXPORT uint32_t safety_get_latched(void)
{
    return g_safety_latched;
}

EXPORT void safety_request_reset(void)
{
    (void)tk_set_flg(g_flg_safety, SAFETY_REQ_RESET);
}

EXPORT void safety_heartbeat(safety_hb_t task)
{
    if ((uint32_t)task < (uint32_t)SAFETY_HB_COUNT)
    {
        g_hb[task]++;           /* single writer per word */
    }
}

EXPORT uint32_t safety_heartbeat_count(safety_hb_t task)
{
    return ((uint32_t)task < (uint32_t)SAFETY_HB_COUNT) ? g_hb[task] : 0u;
}

EXPORT void imu_get_snap(imu_snap_t * p_out)
{
    UINT sts = app_lock(&g_imu_lock);

    *p_out = g_imu_snap;
    app_unlock(&g_imu_lock, sts);
}

EXPORT void imu_put_snap(imu_snap_t const * p_in)
{
    UINT sts = app_lock(&g_imu_lock);

    g_imu_snap = *p_in;
    app_unlock(&g_imu_lock, sts);
}
