/*
 *  safety_task.c
 *  Subsystem 4 -- SafetyTask (priority 5, core 1): fault supervisor
 *  (Week 6 Section 3, FR15, NFR8).  Owner: Buddy 4.
 *
 *  Loop (at least every 20 ms):
 *    - tk_wai_flg(flg_safety, SAFETY_BRAKE_MASK | SAFETY_REQ_RESET,
 *      TWF_ORW | TWF_BITCLR, 20 ms): the bits that woke it are taken in the
 *      same call, so they cannot re-wake it (Tribunal 1 gap 1);
 *    - any brake bit: subsystem2_motor_estop() at once, OR into the latched
 * word, FAULT -> mbf_mission, fault record -> mbf_telem;
 *    - heartbeat check (50 / 50 / 300 / 300 / 100 ms): a missed task is
 *      HEARTBEAT_TIMEOUT, handled like a brake cause;
 *    - feed the watchdog (the only feeder; 200 ms).
 *  REQ_RESET (operator reset via MissionTask): the latch is cleared only
 *  if no cause is still present (obstacle < 8 cm, missed heartbeat, robot
 *  still tilted beyond 30 deg).
 *
 *  14 Nov (instructor, course forum): IMU collision detection is removed.
 *  The INT1 impact path (imu_int1_isr on GP19, LOG 43) is gone; GP19 is
 *  left at its reset state (input, pull-down) and the INT1 wire can be
 *  unplugged.  Instead ImuTask sets LIFT or TILT while a run is active.
 */

#include "safety_task.h"
#include "subsystem4.h"
#include "safety_core.h"
#include "imu_cfg.h"
#include "drv_watchdog.h"
#include "../app_types.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define CHK_GUARD_SPINS 1000000u /* polls before the check gives up */

#define SAFETY_WD_TIMEOUT_MS 200u
#define SAFETY_TOO_CLOSE_MM  80u

/* comms_proto_rec_t code: data = missed-task mask */
#define SAFETY_LOG_HB_MISSED 41u

/* Boot check (NFR8): the 1 us timer must run at 1 MHz.  Reference: the
   SysTick counter, which counts clk_sys (125 MHz) in hardware.  Count
   250 000 decrements (2 ms) and compare the timer: 2000 us expected; a
   timer left 42.7 x slow by the cold-boot tick bug shows ~47 us.  Kernel
   time is not used as the reference: it loses ticks whenever interrupts
   stay masked for a whole tick (e.g. long tm_printf calls on the UART
   console).  Interrupts are masked for these 2 ms, once, at start-up.
   SysTick is private to each core, and on the dual-core build the kernel
   keeps core 1's SysTick disabled (core 0 is the only timekeeper,
   knl_smp_core1_entry).  SafetyTask runs on core 1, so if this core's
   SysTick is off, it is run free (no interrupt) for the check and then
   switched off again.  Core 0's SysTick, the kernel tick, is never
   touched: there it is already running and is only read.            */
#define CHK_SYST_CSR     0xE000E010u
#define CHK_SYST_RVR     0xE000E014u
#define CHK_SYST_CVR     0xE000E018u
#define CHK_SYST_ENABLE  0x1u
#define CHK_SYST_CLK_CPU 0x4u    /* CLKSOURCE = processor clock */
#define CHK_SYS_COUNTS   250000u /* 2 ms at 125 MHz */

/* Written only by SafetyTask (one writer, aligned word: indivisible). */
static uint32_t volatile g_safety_latched;

static bool volatile gb_tick_ok;
static bool volatile gb_tick_checked;
static volatile UW g_safety_tick_us; /* measured us per 100 ms of kernel time */

static void     read_heartbeats(uint32_t counts[SAFETY_HB_COUNT]);
static void     latch(uint32_t bits, uint32_t now_ms);
static uint32_t live_causes(safety_core_hb_state_t * p_hb, uint32_t now_ms);
static void     check_timer_tick(void);

EXPORT bool safety_task_timer_tick_ok (void)
{
    return gb_tick_checked && gb_tick_ok;
}

EXPORT void safety_task_thread (INT stacd, void * p_exinf)
{
    safety_core_hb_state_t hb_state = {0};
    uint32_t               counts[SAFETY_HB_COUNT];
    uint32_t               missed_logged = 0u;

    (void)stacd;
    (void)p_exinf;

    check_timer_tick();
    if (drv_watchdog_caused_reboot())
    {
        (void)subsystem1_telem_post(COMMS_KIND_LOG, (uint8_t)APP_SRC_SAFETY_IMU,
                                    42u, 1);
    }
    drv_watchdog_enable(SAFETY_WD_TIMEOUT_MS);

    read_heartbeats(counts);
    safety_core_hb_start(&hb_state, counts, (uint32_t)app_time_ms());

    for (;;)
    {
        UINT     ptn = 0u;
        ER       err;
        uint32_t now_ms;
        uint32_t missed;

        err = tk_wai_flg(subsystem4_flg_safety(),
                         SAFETY_BRAKE_MASK | SAFETY_REQ_RESET,
                         TWF_ORW | TWF_BITCLR, &ptn, S4_SAFETY_PERIOD_MS);
        drv_watchdog_feed();
        now_ms = (uint32_t)app_time_ms();

        if ((E_OK == err) && ((ptn & SAFETY_BRAKE_MASK) != 0u))
        {
            subsystem2_motor_estop(); /* NFR8: brake first */
            latch(ptn & SAFETY_BRAKE_MASK, now_ms);
        }

        read_heartbeats(counts);
        missed = safety_core_hb_check(&hb_state, counts, now_ms);
        if (missed != 0u)
        {
            subsystem2_motor_estop();
            latch(SAFETY_FLT_HEARTBEAT, now_ms);
            if (missed != missed_logged)
            {
                missed_logged = missed;
                (void)subsystem1_telem_post(
                    COMMS_KIND_LOG, (uint8_t)APP_SRC_SAFETY_IMU,
                    SAFETY_LOG_HB_MISSED, (int32_t)missed);
            }
        }

        if ((E_OK == err) && ((ptn & SAFETY_REQ_RESET) != 0u))
        {
            uint32_t const live = live_causes(&hb_state, now_ms);

            if (0u == live)
            {
                g_safety_latched = 0u;
                missed_logged    = 0u;
                read_heartbeats(counts);
                safety_core_hb_start(&hb_state, counts, now_ms);
                (void)subsystem5_mission_post(
                    MISSION_EV_SAFETY_OK, (uint8_t)APP_SRC_SAFETY_IMU, 0u, 0);
            }
            else
            {
                /* still present: stay latched and say why */
                g_safety_latched |= live;
                (void)subsystem5_mission_post(MISSION_EV_FAULT,
                                              (uint8_t)APP_SRC_SAFETY_IMU, 1u,
                                              (int32_t)g_safety_latched);
            }
        }
    }
}

EXPORT uint32_t safety_task_latched (void)
{
    return g_safety_latched;
}

#if APP_TEST

EXPORT UW safety_task_tick_us (void)
{
    return g_safety_tick_us;
}

#endif

static void read_heartbeats (uint32_t counts[SAFETY_HB_COUNT])
{
    uint8_t kdx;

    for (kdx = 0u; kdx < SAFETY_HB_COUNT; kdx++)
    {
        counts[kdx] = subsystem4_heartbeat_count((imu_types_safety_hb_t)kdx);
    }
}

static void latch (uint32_t bits, uint32_t now_ms)
{
    uint32_t const fresh = bits & ~g_safety_latched;

    (void)now_ms;
    g_safety_latched |= bits;
    if (fresh != 0u)
    {
        (void)subsystem5_mission_post(MISSION_EV_FAULT,
                                      (uint8_t)APP_SRC_SAFETY_IMU, 0u,
                                      (int32_t)g_safety_latched);
        (void)subsystem1_telem_post(
            COMMS_KIND_FAULT, (uint8_t)APP_SRC_SAFETY_IMU, 0u, (int32_t)fresh);
    }
}

/* Causes that are still present right now (for an operator reset). */
static uint32_t live_causes (safety_core_hb_state_t * p_hb, uint32_t now_ms)
{
    imu_types_snap_t          imu;
    mission_types_obst_snap_t obst;
    uint32_t                  counts[SAFETY_HB_COUNT];
    uint32_t                  live = 0u;

    subsystem4_get_imu_snap(&imu);
    subsystem5_get_obst_snap(&obst);
    read_heartbeats(counts);
    if ((obst.d_mm != OBST_RANGE_NONE) && (obst.d_mm < SAFETY_TOO_CLOSE_MM))
    {
        live |= SAFETY_FLT_TOO_CLOSE;
    }
    if (imu.tilt != 0u)
    {
        live |= SAFETY_FLT_TILT;
    }
    if (safety_core_hb_check(p_hb, counts, now_ms) != 0u)
    {
        live |= SAFETY_FLT_HEARTBEAT;
    }
    return live;
}

static void check_timer_tick (void)
{
    UW   reload;
    UW   prev;
    UW   acc = 0u;
    UW   us0;
    UW   delta_us;
    UW   guard = 0u;
    UINT intsts;
    bool b_borrowed = false;

    DI(intsts);
    if (0u == (in_w(CHK_SYST_CSR) & CHK_SYST_ENABLE))
    {
        out_w(CHK_SYST_RVR, 0x00FFFFFFu);
        out_w(CHK_SYST_CVR, 0u); /* any write clears it */
        out_w(CHK_SYST_CSR,
              CHK_SYST_CLK_CPU | CHK_SYST_ENABLE); /* TICKINT off */
        b_borrowed = true;
    }
    reload = (in_w(CHK_SYST_RVR) & 0x00FFFFFFu) + 1u;
    prev   = in_w(CHK_SYST_CVR) & 0x00FFFFFFu;
    us0    = app_time_us();
    while ((acc < CHK_SYS_COUNTS) && (guard < CHK_GUARD_SPINS))
    {
        UW const cur = in_w(CHK_SYST_CVR) & 0x00FFFFFFu;

        acc += (prev >= cur) ? (prev - cur)
                             : ((prev + reload) - cur); /* down counter */
        prev = cur;
        guard++;
    }
    delta_us = app_time_us() - us0;
    if (b_borrowed)
    {
        out_w(CHK_SYST_CSR, 0u); /* back to the kernel's state */
        out_w(CHK_SYST_CVR, 0u);
    }
    EI(intsts);

    g_safety_tick_us =
        (UW)(((UD)delta_us * 100000u) / 2000u); /* scaled to us per 100 ms */
    gb_tick_ok =
        (acc >= CHK_SYS_COUNTS) && (delta_us > 1900u) && (delta_us < 2100u);
    gb_tick_checked = true;
}

/*** end of file ***/
