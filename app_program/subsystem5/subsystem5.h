/*
 *  subsystem5.h
 *  Subsystem 5 -- Obstacle scanning and avoidance (ObstacleTask) and the
 *  system state machine (MissionTask).  Owner: Buddy 5 (Ryan).
 *
 *  PUBLIC interface of subsystem 5.  Week 6 design review: Sections 3,
 *  4, 5 and 6.4.  States, events, requests and snapshots: mission_types.h.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <stdint.h>
#include "../app.h"
#include "mission_types.h"

#define S5_PRI_MISSION  8 /* MissionTask, core 1 */
#define S5_PRI_OBSTACLE 9 /* ObstacleTask, core 1 */
#define S5_CORE         APP_CORE1

#define S5_PIN_US_TRIG 0u  /* GP0 (free with CRASH_REPORT=0) */
#define S5_PIN_US_ECHO 1u  /* GP1, bank-0 IRQ on core 1 */
#define S5_PIN_SERVO   15u /* PWM7 B, 50 Hz */

#define S5_MISSION_TICK_MS 100 /* tk_rcv_mbf timeout (supervision) */
#define S5_RANGE_PERIOD_MS 60u /* range_cyc: HC-SR04 minimum cycle */

/* ---------------------------------------------------------------------
 *  IF-06  mbf_mission: every event to MissionTask (owned by S5).
 * --------------------------------------------------------------------- */
#define S5_MBF_MISSION_DEPTH 16u /* events (Week 6: 16 deep) */

/* ID of mbf_mission (design documents); 0 until subsystem5_init() has run. */
IMPORT ID subsystem5_mbf_mission(void);

/* Post an event without waiting.  E_OK, or E_TMOUT when full (dropped and
   counted; MissionTask treats a drop as a fault, Week 6 8.3). */
IMPORT ER       subsystem5_mission_post(uint8_t type, uint8_t src, uint16_t arg,
                                        int32_t data);
IMPORT uint32_t subsystem5_mission_drops(void);

/* ---------------------------------------------------------------------
 *  IF-09  mbf_scan + flg_obst: requests from MissionTask to ObstacleTask.
 *  subsystem5_scan_request() sends the request, then sets OBST_FLG_REQ.
 * --------------------------------------------------------------------- */
#define S5_MBF_SCAN_DEPTH 4u

#define OBST_FLG_RANGE 0x01u /* range_cyc, every 60 ms */
#define OBST_FLG_ECHO  0x02u /* echo_isr               */
#define OBST_FLG_REQ   0x04u /* MissionTask request    */

IMPORT ID subsystem5_mbf_scan(void); /* ID of mbf_scan */
IMPORT ID subsystem5_flg_obst(void); /* ID of flg_obst */

IMPORT ER subsystem5_scan_request(uint8_t type, uint8_t seq, int16_t param);

/* ---------------------------------------------------------------------
 *  IF-10  Mission and obstacle snapshots.
 * --------------------------------------------------------------------- */
IMPORT void subsystem5_get_mission_snap(mission_types_snap_t * p_out);
IMPORT void subsystem5_put_mission_snap(
    mission_types_snap_t const * p_in); /* MissionTask only */
IMPORT void subsystem5_get_obst_snap(mission_types_obst_snap_t * p_out);
IMPORT void subsystem5_put_obst_snap(
    mission_types_obst_snap_t const * p_in); /* ObstacleTask only */

/* Private to subsystem 5 */
#include "mission_task.h"
#include "obstacle_task.h"

/* Called by usermain() in this order: every subsystem's _init() first,
   then every subsystem's _start(), so every kernel object exists before
   any task can use it. */
IMPORT ER subsystem5_init(void);  /* create this subsystem's kernel objects */
IMPORT ER subsystem5_start(void); /* create and start its tasks             */

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
