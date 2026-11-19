/** @file drv_watchdog.h
 *
 *  @brief Subsystem 4 -- RP2040 watchdog and timer tick (NFR8).  Owner:
 * Buddy 4. Register-level (the port builds no SDK hardware_watchdog).
 *
 *  - drv_watchdog_tick_init(): the 1 us timer and the watchdog both count
 *  the watchdog tick, which must be clk_ref / 12 (XOSC 12 MHz).  After a
 *  cold power-on it is not set, and the timer runs 42.7 x slow;
 *  usermain() calls this first (subsystem4_init).
 *  - erratum RP2040-E1: the watchdog counts down 2 per tick, so a 200 ms
 *  timeout needs LOAD = 2 x 200 000.
 *  - Only SafetyTask feeds it.
 */

#ifndef DRV_WATCHDOG_H
#define DRV_WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

/** Start the watchdog's 1 us tick, which also drives the system timer. */
IMPORT void drv_watchdog_tick_init(void);
/** Enable the watchdog: the chip resets if it is not fed for timeout_ms. */
IMPORT void drv_watchdog_enable(uint32_t timeout_ms);
/** Feed the watchdog (restart its countdown). */
IMPORT void drv_watchdog_feed(void);
/** true if the last reset was caused by the watchdog */
IMPORT bool drv_watchdog_caused_reboot(void);

#endif /* DRV_WATCHDOG_H */

/*** end of file ***/
