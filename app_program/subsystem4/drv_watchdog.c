/*
 *  drv_watchdog.c
 *  Subsystem 4 -- watchdog driver.  See drv_watchdog.h.
 */

#include "drv_watchdog.h"

#define WATCHDOG_BASE_ADDR  0x40058000u
#define WD_CTRL             (WATCHDOG_BASE_ADDR + 0x00u)
#define WD_LOAD             (WATCHDOG_BASE_ADDR + 0x04u)
#define WD_REASON           (WATCHDOG_BASE_ADDR + 0x08u)
#define WD_TICK             (WATCHDOG_BASE_ADDR + 0x2Cu)

#define WD_CTRL_ENABLE      (1uL << 30)
#define WD_CTRL_PAUSE_ALL   ((1uL << 24) | (1uL << 25) | (1uL << 26))   /* JTAG, DBG0, DBG1 */
#define WD_TICK_ENABLE      (1uL << 9)
#define WD_TICK_CYCLES      12u                 /* clk_ref 12 MHz -> 1 MHz */
#define WD_LOAD_MAX         0x00FFFFFFu

#define PSM_WDSEL           (0x40010000u + 0x08u)
#define PSM_WDSEL_ALL       0x0001FFFCu         /* everything except ROSC / XOSC */

static uint32_t g_load;

EXPORT void drv_watchdog_tick_init(void)
{
    out_w(WD_TICK, WD_TICK_CYCLES | WD_TICK_ENABLE);
}

EXPORT void drv_watchdog_enable(uint32_t timeout_ms)
{
    uint32_t load = timeout_ms * 1000u * 2u;    /* RP2040-E1 */

    if (load > WD_LOAD_MAX)
    {
        load = WD_LOAD_MAX;
    }
    g_load = load;
    clr_w(WD_CTRL, WD_CTRL_ENABLE);
    set_w(PSM_WDSEL, PSM_WDSEL_ALL);            /* a timeout resets the whole chip */
    out_w(WD_LOAD, g_load);
    set_w(WD_CTRL, WD_CTRL_PAUSE_ALL);          /* a debugger halt does not reset */
    set_w(WD_CTRL, WD_CTRL_ENABLE);
}

EXPORT void drv_watchdog_feed(void)
{
    if (g_load != 0u)
    {
        out_w(WD_LOAD, g_load);
    }
}

EXPORT bool drv_watchdog_caused_reboot(void)
{
    return ((in_w(WD_REASON) & 0x3u) != 0u);
}
