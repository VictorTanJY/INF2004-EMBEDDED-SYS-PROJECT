/** @file safety_task.h
 *
 *  @brief SafetyTask: fault supervisor. Created and started by
 * subsystem4_start().
 */

#ifndef SAFETY_TASK_H
#define SAFETY_TASK_H

#include <tk/tkernel.h>

#include <stdbool.h>
#include <stdint.h>

/** SafetyTask: waits on flg_safety, brakes on a fault, checks the heartbeats.
    Created by subsystem4_start(). */
IMPORT void safety_task_thread(INT stacd, void * p_exinf);

/** Boot self-test: the 1 us timer really ticks at 1 MHz (NFR8). */
IMPORT bool safety_task_timer_tick_ok(void);

/** Latched faults (SAFETY_FLT_* bits); read through subsystem4_get_latched().
 */
IMPORT uint32_t safety_task_latched(void);

#if APP_TEST
/** Measured length of 100 ms of kernel time, us (S4-U02). */
IMPORT uint32_t safety_task_tick_us(void);
#endif

#endif /* SAFETY_TASK_H */

/*** end of file ***/
