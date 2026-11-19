/** @file task_hmi.h
 *
 *  @brief HmiTask: the two buttons. Created and started by subsystem1_start().
 */

#ifndef TASK_HMI_H
#define TASK_HMI_H

#include <tk/tkernel.h>

/** HmiTask: samples both buttons every HMI_PERIOD_MS and posts their events.
    Created by subsystem1_start(). */
IMPORT void task_hmi_thread(INT stacd, void * p_exinf);

#endif /* TASK_HMI_H */

/*** end of file ***/
