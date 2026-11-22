/** @file app_main.h
 *
 *  @brief Program entry point: usermain(), which micro T-Kernel calls once at
 * start-up, and the task-start helper every subsystem uses.
 */

#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <tk/tkernel.h>

#include <stdint.h>

/** Entry point called by the kernel's initial task (micro T-Kernel API). */
IMPORT INT usermain(void);

/** Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core". */
IMPORT ID app_main_start_task(FP entry, PRI pri, uint32_t prc,
                              char const * p_name);

#endif /* APP_MAIN_H */

/*** end of file ***/
