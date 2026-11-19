/** @file mission_task.h
 *
 *  @brief MissionTask: the system statechart. Created and started by
 * subsystem5_start().
 */

#ifndef MISSION_TASK_H
#define MISSION_TASK_H

#include <tk/tkernel.h>

IMPORT void mission_task_thread(INT stacd, void * p_exinf);

#endif /* MISSION_TASK_H */

/*** end of file ***/
