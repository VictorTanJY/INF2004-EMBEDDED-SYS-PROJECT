/** @file imu_task.h
 *
 *  @brief ImuTask: pitch, humps and IMU faults. Created and started by
 * subsystem4_start().
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <tk/tkernel.h>

IMPORT void imu_task_thread(INT stacd, void * p_exinf);

#endif /* IMU_TASK_H */

/*** end of file ***/
