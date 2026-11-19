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
#define S2_PRI_MOTION  6 /* MotionTask, core 1 */
#define S2_CORE_MOTION APP_CORE1
#define S2_PERIOD_MS   10u /* motion_cyc period */

/* Pins (TEAM.md section 2) */
#define S2_PIN_MOT_L_A 8u  /* PWM4 A */
#define S2_PIN_MOT_L_B 9u  /* PWM4 B */
#define S2_PIN_MOT_R_A 10u /* PWM5 A */
#define S2_PIN_MOT_R_B 11u /* PWM5 B */
#define S2_PIN_ENC_L_A 4u  /* PIO0 SM0, B = A + 1 */
#define S2_PIN_ENC_R_A 16u /* PIO0 SM1, B = A + 1 */
/* spare GP6: toggled every cycle when built with APP_TIMING_PINS=1 (NFR1) */
#define S2_PIN_TIMING 6u

/* ---------------------------------------------------------------------
 *  IF-01  mbf_motion: motion commands to MotionTask (owned by S2).
 *  Senders: MissionTask (S5), LineBarcodeTask (S3), ObstacleTask (S5).
 *  Always sent with TMO_POL; MotionTask drains it every 10 ms cycle.
 * --------------------------------------------------------------------- */
/* commands; 8 B each in the buffer (4 B word + header) */
#define S2_MBF_MOTION_DEPTH 8u

IMPORT ID g_mbf_motion; /* mbf_motion in the design documents */

/* Post a command without waiting.  Returns E_OK, or E_TMOUT when the
   buffer is full (the command is dropped; the caller decides). */
IMPORT ER subsystem2_send(motion_types_cmd_t const * p_cmd);

/* Helper for senders: fills and posts one command. */
IMPORT ER subsystem2_post(uint8_t src, uint8_t seq, uint8_t cmd, int16_t arg0,
                          int16_t arg1);

/* The named API (FR5): motion_api_move_forward / _backward,
   motion_api_turn_left / _right, motion_api_stop, motion_api_drive.  Use these;
   subsystem2_post stays for SET_AUTH and CLEAR_ESTOP. */
#include "motion_api.h"

/* ---------------------------------------------------------------------
 *  IF-02  Odometry snapshot (written by MotionTask, read by everyone).
 * --------------------------------------------------------------------- */
IMPORT void subsystem2_get_odom(motion_types_odom_t * p_out); /* any task */
IMPORT void
subsystem2_put_odom(motion_types_odom_t const * p_in); /* MotionTask only */

/* ---------------------------------------------------------------------
 *  IF-03  Emergency brake (the one documented motor write outside
 *  MotionTask).  Called by SafetyTask on core 1 only.  Switches GP8-GP11
 *  from PWM to SIO outputs driven LOW (driver brake) and sets the latch.
 *  MotionTask checks the latch immediately before every PWM write and
 *  reconnects PWM only on CLEAR_ESTOP, doing check-clear-reconnect inside
 *  one DI/EI section (Tribunal 1 gap 3).  Idempotent.
 * --------------------------------------------------------------------- */
IMPORT void subsystem2_motor_estop(void);
IMPORT bool subsystem2_estop_latched(void);

#if APP_TEST
/* Time of the last subsystem2_motor_estop() call, us (app_time_us); S4-U07. */
IMPORT volatile uint32_t g_estop_test_us;
#endif

/* Private to subsystem 2 */
#include "motion_task.h"
IMPORT bool subsystem2_estop_reconnect(void); /* MotionTask only */

/* Called by usermain() in this order: every subsystem's _init() first,
   then every subsystem's _start(), so every kernel object exists before
   any task can use it. */
IMPORT ER subsystem2_init(void);  /* create this subsystem's kernel objects */
IMPORT ER subsystem2_start(void); /* create and start its tasks             */

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
