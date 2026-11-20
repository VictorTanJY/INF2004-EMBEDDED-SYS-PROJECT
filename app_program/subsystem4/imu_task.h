/** @file imu_task.h
 *
 *  @brief ImuTask: pitch, humps and IMU faults. Created and started by
 * subsystem4_start().
 */

#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <tk/tkernel.h>

/** ImuTask: attitude, humps, tilt and lift every 10 ms.  Created by
    subsystem4_start(). */
IMPORT void imu_task_thread(INT stacd, void * p_exinf);

#endif /* IMU_TASK_H */

/*** end of file ***/
