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

#define S5_PRI_MISSION          8           /* MissionTask, core 1 */
#define S5_PRI_OBSTACLE         9           /* ObstacleTask, core 1 */
#define S5_CORE                 APP_CORE1

#define S5_PIN_US_TRIG          0u          /* GP0 (free with CRASH_REPORT=0) */
#define S5_PIN_US_ECHO          1u          /* GP1, bank-0 IRQ on core 1 */
#define S5_PIN_SERVO            15u         /* PWM7 B, 50 Hz */

#define S5_MISSION_TICK_MS      100         /* tk_rcv_mbf timeout (supervision) */
#define S5_RANGE_PERIOD_MS      60          /* range_cyc: HC-SR04 minimum cycle */

/* ---------------------------------------------------------------------
 *  IF-06  mbf_mission: every event to MissionTask (owned by S5).
 * --------------------------------------------------------------------- */
#define S5_MBF_MISSION_DEPTH    16u         /* events (Week 6: 16 deep) */

IMPORT ID g_mbf_mission;        /* mbf_mission in the design documents */

/* Post an event without waiting.  E_OK, or E_TMOUT when full (dropped and
   counted; MissionTask treats a drop as a fault, Week 6 8.3). */
IMPORT ER       mission_post(uint8_t type, uint8_t src, uint16_t a, int32_t data);
IMPORT uint32_t mission_post_drops(void);

/* ---------------------------------------------------------------------
 *  IF-09  mbf_scan + flg_obst: requests from MissionTask to ObstacleTask.
 *  scan_request() sends the request, then sets OBST_FLG_REQ.
 * --------------------------------------------------------------------- */
#define S5_MBF_SCAN_DEPTH       4u

#define OBST_FLG_RANGE          0x01u   /* range_cyc, every 60 ms */
#define OBST_FLG_ECHO           0x02u   /* echo_isr               */
#define OBST_FLG_REQ            0x04u   /* MissionTask request    */

IMPORT ID g_mbf_scan;           /* mbf_scan */
IMPORT ID g_flg_obst;           /* flg_obst */

IMPORT ER scan_request(uint8_t type, uint8_t seq, int16_t param);

/* ---------------------------------------------------------------------
 *  IF-10  Mission and obstacle snapshots.
 * --------------------------------------------------------------------- */
IMPORT void mission_get_snap(mission_snap_t * p_out);
IMPORT void mission_put_snap(mission_snap_t const * p_in);     /* MissionTask only */
IMPORT void obst_get_snap(obst_snap_t * p_out);
IMPORT void obst_put_snap(obst_snap_t const * p_in);           /* ObstacleTask only */

/* Private to subsystem 5 */
IMPORT void mission_task(INT stacd, void * p_exinf);
IMPORT void obstacle_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM5_H */
