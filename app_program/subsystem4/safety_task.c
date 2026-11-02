/*
 *  safety_task.c
 *  Subsystem 4 -- SafetyTask (priority 5, core 1): fault supervisor
 *  (Week 6 Section 3, FR15, NFR8).  Owner: Buddy 4.
 *
 *  Loop (at least every 20 ms):
 *    - tk_wai_flg(flg_safety, SAFETY_BRAKE_MASK | SAFETY_REQ_RESET,
 *      TWF_ORW | TWF_BITCLR, 20 ms): the bits that woke it are taken in the
 *      same call, so they cannot re-wake it (Tribunal 1 gap 1);
 *    - any brake bit: motor_estop() at once, OR into the latched word,
 *      FAULT -> mbf_mission, fault record -> mbf_telem;
 *    - heartbeat check (50 / 50 / 300 / 300 / 100 ms): a missed task is
 *      HEARTBEAT_TIMEOUT, handled like a brake cause;
 *    - feed the watchdog (the only feeder; 200 ms).
 *  REQ_RESET (operator reset via MissionTask): the latch is cleared only
 *  if no cause is still present (obstacle < 8 cm, missed heartbeat).
 *
 *  imu_int1_isr (GP19 rising edge, core 1) sets REQ_IMPACT_CHK.  SafetyTask
 *  then brakes and latches IMPACT only if ImuTask's software check also saw
 *  an impact within IMU_IMPACT_AGREE_MS of the edge (it polls every 10 ms
 *  for at most that long); otherwise it logs LOG 43 (peak jolt, mg).  2 Nov:
 *  search-sweep wheel reversals fired INT1 with no jolt in the readings.
 */

#include "subsystem4.h"
#include "safety_core.h"
#include "imu_cfg.h"
#include "drv_watchdog.h"
#include "../app_gpio_irq.h"
#include "../app_types.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"

#define SAFETY_WD_TIMEOUT_MS    200u
#define SAFETY_TOO_CLOSE_MM     80u
#define SAFETY_LOG_HB_MISSED    41u     /* comms_rec_t code: data = missed-task mask */
#define SAFETY_LOG_IMPACT_UNCONF 43u    /* INT1 without a software impact: data = peak jolt, mg */
#define SAFETY_CHK_PERIOD_MS    10u     /* poll period while an INT1 edge is checked */

IMPORT volatile uint32_t g_safety_latched;   /* subsystem4.c */

static volatile bool g_b_tick_ok;
static volatile bool g_b_tick_checked;
EXPORT volatile UW   g_safety_tick_us;  /* measured us per 100 ms of kernel time */

/* imu_int1_isr: high-g impact from the GY-511 INT1 pin. */
static void imu_int1_isr(uint32_t pin, uint32_t events, uint32_t t_us)
{
    (void)pin;
    (void)events;
    (void)t_us;
    (void)tk_set_flg(g_flg_safety, SAFETY_REQ_IMPACT_CHK);
}

/* Did ImuTask's software check see an impact within IMU_IMPACT_AGREE_MS
   of the INT1 edge at edge_ms (both app_time_ms_hw)? */
static bool impact_confirmed(uint32_t edge_ms)
{
    imu_snap_t imu;
    int32_t    dt;

    imu_get_snap(&imu);
    dt = (int32_t)(imu.impact_ms - edge_ms);
    return (imu.impacts != 0u) && (dt >= -(int32_t)IMU_IMPACT_AGREE_MS) &&
           (dt <= (int32_t)IMU_IMPACT_AGREE_MS);
}

EXPORT bool safety_timer_tick_ok(void)
{
    return g_b_tick_checked && g_b_tick_ok;
}

static void read_heartbeats(uint32_t counts[SAFETY_HB_COUNT])
{
    uint8_t k;

    for (k = 0u; k < (uint8_t)SAFETY_HB_COUNT; k++)
    {
        counts[k] = safety_heartbeat_count((safety_hb_t)k);
    }
}

static void latch(uint32_t bits, uint32_t now_ms)
{
    uint32_t const fresh = bits & ~g_safety_latched;

    (void)now_ms;
    g_safety_latched |= bits;
    if (fresh != 0u)
    {
        (void)mission_post((uint8_t)MISSION_EV_FAULT, (uint8_t)APP_SRC_SAFETY_IMU, 0u,
                           (int32_t)g_safety_latched);
        (void)comms_telem_post((uint8_t)COMMS_KIND_FAULT, (uint8_t)APP_SRC_SAFETY_IMU, 0u,
                               (int32_t)fresh);
    }
}

/* Causes that are still present right now (for an operator reset). */
static uint32_t live_causes(safety_hb_state_t * p_hb, uint32_t now_ms)
{
    imu_snap_t  imu;
    obst_snap_t obst;
    uint32_t    counts[SAFETY_HB_COUNT];
    uint32_t    live = 0u;

    imu_get_snap(&imu);
    obst_get_snap(&obst);
    read_heartbeats(counts);
    if ((obst.d_mm != OBST_RANGE_NONE) && (obst.d_mm < SAFETY_TOO_CLOSE_MM))
    {
        live |= SAFETY_FLT_TOO_CLOSE;
    }
    if (safety_hb_check(p_hb, counts, now_ms) != 0u)
    {
        live |= SAFETY_FLT_HEARTBEAT;
    }
    return live;
}

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
#define CHK_SYST_CSR        0xE000E010u
#define CHK_SYST_RVR        0xE000E014u
#define CHK_SYST_CVR        0xE000E018u
#define CHK_SYST_ENABLE     0x1u
#define CHK_SYST_CLK_CPU    0x4u        /* CLKSOURCE = processor clock */
#define CHK_SYS_COUNTS      250000u     /* 2 ms at 125 MHz */

static void check_timer_tick(void)
{
    UW       reload;
    UW       prev;
    UW       acc = 0u;
    UW       us0;
    UW       du;
    UW       guard = 0u;
    UINT     intsts;
    bool     b_borrowed = false;

    DI(intsts);
    if ((in_w(CHK_SYST_CSR) & CHK_SYST_ENABLE) == 0u)
    {
        out_w(CHK_SYST_RVR, 0x00FFFFFFu);
        out_w(CHK_SYST_CVR, 0u);                        /* any write clears it */
        out_w(CHK_SYST_CSR, CHK_SYST_CLK_CPU | CHK_SYST_ENABLE);   /* TICKINT off */
        b_borrowed = true;
    }
    reload = (in_w(CHK_SYST_RVR) & 0x00FFFFFFu) + 1u;
    prev   = in_w(CHK_SYST_CVR) & 0x00FFFFFFu;
    us0  = app_time_us();
    while ((acc < CHK_SYS_COUNTS) && (guard < 1000000u))
    {
        UW const cur = in_w(CHK_SYST_CVR) & 0x00FFFFFFu;

        acc += (prev >= cur) ? (prev - cur) : ((prev + reload) - cur);   /* down counter */
        prev = cur;
        guard++;
    }
    du = app_time_us() - us0;
    if (b_borrowed)
    {
        out_w(CHK_SYST_CSR, 0u);                        /* back to the kernel's state */
        out_w(CHK_SYST_CVR, 0u);
    }
    EI(intsts);

    g_safety_tick_us = (UW)(((UD)du * 100000u) / 2000u);   /* scaled to us per 100 ms */
    g_b_tick_ok      = (acc >= CHK_SYS_COUNTS) && (du > 1900u) && (du < 2100u);
    g_b_tick_checked = true;
}

EXPORT void safety_task(INT stacd, void * p_exinf)
{
    safety_hb_state_t hb = {0};
    uint32_t          counts[SAFETY_HB_COUNT];
    uint32_t          missed_logged = 0u;
    bool              b_chk = false;    /* an INT1 edge is being confirmed */
    uint32_t          chk_edge_ms = 0u;

    (void)stacd;
    (void)p_exinf;

    /* GP19: input, rising edge on core 1.  Pull-down (2 Nov): with no pull
       a loose INT1 wire floated and motor noise gave false impacts; INT1
       is push-pull, so a real high still overrides the ~50 k pull-down. */
    out_w(GPIO_OE_CLR, 1uL << S4_PIN_IMU_INT1);
    out_w(GPIO(S4_PIN_IMU_INT1), GPIO_IE | GPIO_SHEMITT | GPIO_PDE);
    out_w(GPIO_CTRL(S4_PIN_IMU_INT1), GPIO_CTRL_FUNCSEL_SIO);
    (void)app_gpio_irq_attach(S4_PIN_IMU_INT1, APP_GPIO_EDGE_RISE, imu_int1_isr);

    check_timer_tick();
    if (drv_watchdog_caused_reboot())
    {
        (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_SAFETY_IMU, 42u, 1);
    }
    drv_watchdog_enable(SAFETY_WD_TIMEOUT_MS);

    read_heartbeats(counts);
    safety_hb_start(&hb, counts, (uint32_t)app_time_ms());

    for (;;)
    {
        UINT     ptn = 0u;
        ER       er;
        uint32_t now_ms;
        uint32_t missed;

        er = tk_wai_flg(g_flg_safety,
                        SAFETY_BRAKE_MASK | SAFETY_REQ_RESET | SAFETY_REQ_IMPACT_CHK,
                        TWF_ORW | TWF_BITCLR, &ptn,
                        b_chk ? SAFETY_CHK_PERIOD_MS : S4_SAFETY_PERIOD_MS);
        drv_watchdog_feed();
        now_ms = (uint32_t)app_time_ms();

        if ((er == E_OK) && ((ptn & SAFETY_BRAKE_MASK) != 0u))
        {
            motor_estop();                          /* NFR8: brake first */
            latch(ptn & SAFETY_BRAKE_MASK, now_ms);
        }

        /* INT1 edge: brake only when the readings agree (see the header). */
        if ((er == E_OK) && ((ptn & SAFETY_REQ_IMPACT_CHK) != 0u) && !b_chk)
        {
            b_chk       = true;
            chk_edge_ms = (uint32_t)app_time_ms_hw();
        }
        if (b_chk)
        {
            if (impact_confirmed(chk_edge_ms))
            {
                b_chk = false;
                motor_estop();
                latch(SAFETY_FLT_IMPACT, now_ms);
            }
            else if (app_elapsed_ms((uint32_t)app_time_ms_hw(), chk_edge_ms) >= IMU_IMPACT_AGREE_MS)
            {
                imu_snap_t imu;

                b_chk = false;
                imu_get_snap(&imu);
                (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_SAFETY_IMU,
                                       SAFETY_LOG_IMPACT_UNCONF, (int32_t)imu.jolt_peak_mg);
            }
            else
            {
                /* keep waiting, at most IMU_IMPACT_AGREE_MS */
            }
        }

        read_heartbeats(counts);
        missed = safety_hb_check(&hb, counts, now_ms);
        if (missed != 0u)
        {
            motor_estop();
            latch(SAFETY_FLT_HEARTBEAT, now_ms);
            if (missed != missed_logged)
            {
                missed_logged = missed;
                (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_SAFETY_IMU,
                                       SAFETY_LOG_HB_MISSED, (int32_t)missed);
            }
        }

        if ((er == E_OK) && ((ptn & SAFETY_REQ_RESET) != 0u))
        {
            uint32_t const live = live_causes(&hb, now_ms);

            if (live == 0u)
            {
                g_safety_latched = 0u;
                missed_logged    = 0u;
                read_heartbeats(counts);
                safety_hb_start(&hb, counts, now_ms);
                (void)mission_post((uint8_t)MISSION_EV_SAFETY_OK, (uint8_t)APP_SRC_SAFETY_IMU, 0u, 0);
            }
            else
            {
                /* still present: stay latched and say why */
                g_safety_latched |= live;
                (void)mission_post((uint8_t)MISSION_EV_FAULT, (uint8_t)APP_SRC_SAFETY_IMU, 1u,
                                   (int32_t)g_safety_latched);
            }
        }
    }
}
