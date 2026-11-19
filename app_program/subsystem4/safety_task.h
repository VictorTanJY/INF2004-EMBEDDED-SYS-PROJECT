/** @file safety_task.h
 *
 *  @brief SafetyTask: fault supervisor. Created and started by
 * subsystem4_start().
 */

#ifndef SAFETY_TASK_H
#define SAFETY_TASK_H

#include <tk/tkernel.h>

#include <stdbool.h>

IMPORT void safety_task_thread(INT stacd, void * p_exinf);

/* Boot self-test: the 1 us timer really ticks at 1 MHz (NFR8). */
IMPORT bool safety_task_timer_tick_ok(void);

#endif /* SAFETY_TASK_H */

/*** end of file ***/
