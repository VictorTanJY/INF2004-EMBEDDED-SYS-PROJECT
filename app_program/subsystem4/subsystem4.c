/*
 *  subsystem4.c
 *  Subsystem 4 -- IMU and safety.  Owner: Buddy 4 (Purvez).
 *
 *  STATUS: shared-base stub.  Creates flg_safety and provides the
 *  heartbeat words and IMU snapshot.
 *  TODO (owner): SafetyTask (wait TWF_ORW | TWF_BITCLR on
 *  SAFETY_BRAKE_MASK with a 20 ms timeout, motor_estop(), latch, FAULT ->
 *  mbf_mission, heartbeat checks, watchdog feed), drv_watchdog (no SDK
 *  watchdog is built; 200 ms needs a load of 2 x 200000 because of
 *  RP2040-E1, and check the watchdog tick is 1 us), ImuTask, drv_imu on
 *  I2C1 (release RESETS_RESET_I2C1 yourself: it is not released at boot),
 *  imu_int1_isr in the shared IO_IRQ_BANK0 handler on core 1.
 */

#include "subsystem4.h"

EXPORT ID g_flg_safety;

static volatile uint32_t g_latched;
static volatile uint32_t g_hb[SAFETY_HB_COUNT];
static app_lock_t        g_imu_lock;
static imu_snap_t        g_imu_snap;

EXPORT ER subsystem4_init(void)
{
    T_CFLG  cflg = {0};

    app_lock_init(&g_imu_lock);

    /* TA_WMUL: a stray second waiter gets queued instead of E_OBJ.  By
       rule only SafetyTask waits (see subsystem4.h). */
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;
    g_flg_safety = tk_cre_flg(&cflg);

    return (g_flg_safety < E_OK) ? g_flg_safety : E_OK;
}

EXPORT ER subsystem4_start(void)
{
    /* TODO (owner): start SafetyTask (S4_PRI_SAFETY, S4_CORE_SAFETY) and
       ImuTask (S4_PRI_IMU, S4_CORE_IMU). */
    return E_OK;
}

EXPORT uint32_t safety_get_latched(void)
{
    return g_latched;
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
