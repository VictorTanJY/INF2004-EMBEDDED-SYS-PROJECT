/*
 *  subsystem4.h
 *  Subsystem 4 -- IMU terrain monitoring (ImuTask) and fault supervision
 *  (SafetyTask).  Owner: Buddy 4 (Purvez).
 *
 *  PUBLIC interface of subsystem 4.  Week 6 design review: Sections 3,
 *  4 and 6.3.  Fault bits and the IMU snapshot format are in imu_types.h.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdint.h>
#include "../app.h"
#include "imu_types.h"

#define S4_PRI_SAFETY           5           /* SafetyTask, core 1 (highest app task) */
#define S4_CORE_SAFETY          APP_CORE1
#define S4_PRI_IMU              10          /* ImuTask, core 0 */
#define S4_CORE_IMU             APP_CORE0

#define S4_PIN_IMU_SDA          2u          /* I2C1 */
#define S4_PIN_IMU_SCL          3u
#define S4_PIN_IMU_INT1         19u         /* high-g interrupt, bank-0 IRQ on core 1 */

#define S4_SAFETY_PERIOD_MS     20          /* SafetyTask wait timeout */
#define S4_IMU_PERIOD_MS        10          /* imu_cyc */

/* ---------------------------------------------------------------------
 *  IF-05  flg_safety (owned by S4).  Setters use tk_set_flg().  Only
 *  SafetyTask waits on it (TWF_ORW | TWF_BITCLR on SAFETY_BRAKE_MASK |
 *  SAFETY_REQ_RESET); every other task reads it with tk_ref_flg().
 * --------------------------------------------------------------------- */
IMPORT ID g_flg_safety;         /* flg_safety in the design documents */

/* Latched faults (SAFETY_FLT_* bits), written only by SafetyTask. */
IMPORT uint32_t safety_get_latched(void);

/* MissionTask: ask SafetyTask to clear the latch after an operator reset.
   SafetyTask answers MISSION_EV_SAFETY_OK, or MISSION_EV_FAULT if a cause
   is still present. */
IMPORT void safety_request_reset(void);

/* ---------------------------------------------------------------------
 *  IF-07  Heartbeats.  Each monitored task calls safety_heartbeat() on
 *  every pass: one 32-bit word per task, one writer each (an aligned word
 *  store is indivisible), so no lock is needed.  SafetyTask checks them
 *  every 20 ms against 50/50/300/300/100 ms.
 * --------------------------------------------------------------------- */
IMPORT void     safety_heartbeat(safety_hb_t task);
IMPORT uint32_t safety_heartbeat_count(safety_hb_t task);

/* ---------------------------------------------------------------------
 *  IF-08  IMU snapshot (written by ImuTask).
 * --------------------------------------------------------------------- */
IMPORT void imu_get_snap(imu_snap_t * p_out);           /* any task       */
IMPORT void imu_put_snap(imu_snap_t const * p_in);      /* ImuTask only   */

/* Boot self-test: the 1 us timer really ticks at 1 MHz (NFR8). */
IMPORT bool safety_timer_tick_ok(void);

/* Private to subsystem 4 */
IMPORT void safety_task(INT stacd, void * p_exinf);
IMPORT void imu_task(INT stacd, void * p_exinf);
IMPORT ID   g_tid_imu;

#endif /* SUBSYSTEM4_H */
