/** @file motion_task.h
 *
 *  @brief MotionTask: 10 ms wheel-speed control. Created and started by
 * subsystem2_start().
 */

#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include <tk/tkernel.h>

#include <stdint.h>
#include "../app_config.h"

/** MotionTask: 10 ms speed control, odometry and commands.  Created by
    subsystem2_start(). */
IMPORT void motion_task_thread(INT stacd, void * p_exinf);

#if APP_TEST
/** make TEST=1 only (S2-U07): MotionTask period and run time, since the
   last restart.  per_min_us = 0xFFFFFFFF until two cycles have run. */
typedef struct
{
    uint32_t per_min_us;
    uint32_t per_max_us;
    uint32_t exec_max_us; /**< wake-up to end of the cycle's work */
    uint32_t cycles;
} motion_task_timing_t;

/** Self-test: start a new measurement of the loop period and run time. */
IMPORT void motion_task_timing_restart(void);
/** Self-test: copy the loop period and run-time figures measured since the
    restart. */
IMPORT void motion_task_test_timing(motion_task_timing_t * p_out);
#endif

#endif /* MOTION_TASK_H */

/*** end of file ***/
