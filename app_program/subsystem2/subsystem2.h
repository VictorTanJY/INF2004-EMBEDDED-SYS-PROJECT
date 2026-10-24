/*
 *  subsystem2.h
 *  Subsystem 2 -- Motion control (MotionTask).  Owner: Buddy 2 (Jedrek).
 *
 *  PUBLIC interface of subsystem 2: what other subsystems may use.
 *  Week 6 design review: Sections 3, 4 and 6.1.  Message and snapshot
 *  formats are in motion_types.h; tuning values in motion_cfg.h.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"
#include "motion_types.h"

/* Task placement (TEAM.md section 4) */
#define S2_PRI_MOTION           6           /* MotionTask, core 1 */
#define S2_CORE_MOTION          APP_CORE1
#define S2_PERIOD_MS            10          /* motion_cyc period */

/* Pins (TEAM.md section 2) */
#define S2_PIN_MOT_L_A          8u          /* PWM4 A */
#define S2_PIN_MOT_L_B          9u          /* PWM4 B */
#define S2_PIN_MOT_R_A          10u         /* PWM5 A */
#define S2_PIN_MOT_R_B          11u         /* PWM5 B */
#define S2_PIN_ENC_L_A          4u          /* PIO0 SM0, B = A + 1 */
#define S2_PIN_ENC_R_A          16u         /* PIO0 SM1, B = A + 1 */
#define S2_PIN_TIMING           6u          /* spare GP6: toggled every cycle when
                                               built with APP_TIMING_PINS=1 (NFR1) */

/* ---------------------------------------------------------------------
 *  IF-01  mbf_motion: motion commands to MotionTask (owned by S2).
 *  Senders: MissionTask (S5), LineBarcodeTask (S3), ObstacleTask (S5).
 *  Always sent with TMO_POL; MotionTask drains it every 10 ms cycle.
 * --------------------------------------------------------------------- */
#define S2_MBF_MOTION_DEPTH     8u  /* commands; 12 B each in the buffer */

IMPORT ID g_mbf_motion;         /* mbf_motion in the design documents */

/* Post a command without waiting.  Returns E_OK, or E_TMOUT when the
   buffer is full (the command is dropped; the caller decides). */
IMPORT ER motion_send(motion_cmd_t const * p_cmd);

/* Helper for senders: fills and posts one command. */
IMPORT ER motion_post(uint8_t src, uint8_t seq, uint8_t cmd, int16_t p0, int16_t p1);

/* ---------------------------------------------------------------------
 *  IF-02  Odometry snapshot (written by MotionTask, read by everyone).
 * --------------------------------------------------------------------- */
IMPORT void motion_get_odom(motion_odom_t * p_out);     /* any task      */
IMPORT void motion_put_odom(motion_odom_t const * p_in); /* MotionTask only */

/* ---------------------------------------------------------------------
 *  IF-03  Emergency brake (the one documented motor write outside
 *  MotionTask).  Called by SafetyTask on core 1 only.  Switches GP8-GP11
 *  from PWM to SIO outputs driven LOW (driver brake) and sets the latch.
 *  MotionTask checks the latch immediately before every PWM write and
 *  reconnects PWM only on CLEAR_ESTOP, doing check-clear-reconnect inside
 *  one DI/EI section (Tribunal 1 gap 3).  Idempotent.
 * --------------------------------------------------------------------- */
IMPORT void motor_estop(void);
IMPORT bool motion_estop_latched(void);

/* Private to subsystem 2 */
IMPORT void motion_task(INT stacd, void * p_exinf);
IMPORT bool motion_estop_clear_and_reconnect(void);   /* MotionTask only */

#endif /* SUBSYSTEM2_H */
