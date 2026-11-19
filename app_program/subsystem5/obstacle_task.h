/** @file obstacle_task.h
 *
 *  @brief ObstacleTask: ultrasonic scan and bypass. Created and started by
 * subsystem5_start().
 */

#ifndef OBSTACLE_TASK_H
#define OBSTACLE_TASK_H

#include <tk/tkernel.h>

IMPORT void obstacle_task_thread(INT stacd, void * p_exinf);

#endif /* OBSTACLE_TASK_H */

/*** end of file ***/
