/*
 *  subsystem2.h
 *  Subsystem 2 -- Motion control (MotionTask).  Owner: Buddy 2 (Jedrek).
 *
 *  PUBLIC interface of subsystem 2: what other subsystems may use.
 *  Week 6 design review: Sections 3, 4 and 6.1.
 *
 *  STATUS: interface + object creation only (shared base).  The task,
 *  PWM, PIO encoder and PID code are still to be written by the owner.
 *  Units and ranges marked "PROPOSED" must be confirmed by the owner
 *  before other subsystems rely on them (TEAM.md section 5).
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

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

/* ---------------------------------------------------------------------
 *  IF-01  mbf_motion: motion commands to MotionTask (owned by S2).
 *  Senders: MissionTask (S5), LineBarcodeTask (S3), ObstacleTask (S5).
 *  Always sent with TMO_POL; MotionTask drains it every 10 ms cycle.
 * --------------------------------------------------------------------- */
typedef enum
{
    MOTION_SRC_MISSION = 0,     /* MissionTask: STOP, MOVE/TURN, SET_AUTH */
    MOTION_SRC_LINE    = 1,     /* LineBarcodeTask: DRIVE                 */
    MOTION_SRC_AVOID   = 2      /* ObstacleTask during a bypass           */
} motion_src_t;

typedef enum
{
    MOTION_CMD_DRIVE       = 1, /* p0 = v [mm/s], p1 = omega [mrad/s, +left]  */
    MOTION_CMD_MOVE        = 2, /* p0 = distance [mm, +fwd], p1 = v [mm/s]   */
    MOTION_CMD_TURN        = 3, /* p0 = angle [deg, +left], p1 = rate [deg/s] */
    MOTION_CMD_STOP        = 4, /* ramp down (~1 m/s^2), then brake          */
    MOTION_CMD_SET_AUTH    = 5, /* p0 = new authority (motion_src_t); MISSION only */
    MOTION_CMD_CLEAR_ESTOP = 6  /* MISSION only, after an operator reset     */
} motion_cmd_code_t;            /* units PROPOSED */

typedef struct
{
    uint8_t src;                /* motion_src_t                          */
    uint8_t seq;                /* sender's sequence number (wraps)      */
    uint8_t cmd;                /* motion_cmd_code_t                     */
    uint8_t reserved;           /* 0                                     */
    int16_t p0;                 /* see motion_cmd_code_t                 */
    int16_t p1;
} motion_cmd_t;                 /* 8 bytes */

#define S2_MBF_MOTION_DEPTH     8u  /* commands; 12 B each in the buffer */

IMPORT ID g_mbf_motion;         /* mbf_motion in the design documents */

/* Post a command without waiting.  Returns E_OK, or E_TMOUT when the
   buffer is full (the command is dropped; the caller decides). */
IMPORT ER motion_send(motion_cmd_t const * p_cmd);

/* ---------------------------------------------------------------------
 *  IF-02  Odometry snapshot (written by MotionTask, read by everyone).
 * --------------------------------------------------------------------- */
typedef enum
{
    MOTION_ST_IDLE_BRAKED    = 0,
    MOTION_ST_VELOCITY_TRACK = 1,
    MOTION_ST_MANEUVER       = 2,
    MOTION_ST_STOPPING       = 3,
    MOTION_ST_ESTOP_LATCHED  = 4
} motion_state_t;

typedef struct
{
    uint32_t t_ms;              /* time of the update (app_time_ms)      */
    int32_t  enc_l;             /* accumulated encoder counts, x4        */
    int32_t  enc_r;
    int32_t  dist_mm;           /* distance travelled, signed            */
    int16_t  v_l_mm_s;          /* measured wheel speeds                 */
    int16_t  v_r_mm_s;
    uint16_t rejected;          /* commands rejected (wrong authority)   */
    uint8_t  state;             /* motion_state_t                        */
    uint8_t  auth;              /* motion_src_t currently obeyed         */
    uint8_t  last_done_seq;     /* seq of the last completed MOVE/TURN   */
    uint8_t  reserved[3];
} motion_odom_t;

IMPORT void motion_get_odom(motion_odom_t * p_out);     /* any task      */
IMPORT void motion_put_odom(motion_odom_t const * p_in); /* MotionTask only */

/* ---------------------------------------------------------------------
 *  IF-03  Emergency brake (the one documented motor write outside
 *  MotionTask).  Called by SafetyTask on core 1 only.  Switches GP8-GP11
 *  from PWM to SIO outputs driven LOW (driver brake) and sets the latch.
 *  MotionTask checks motion_estop_latched() immediately before every PWM
 *  write and reconnects PWM only on CLEAR_ESTOP, doing check-clear-
 *  reconnect inside one DI/EI section (Tribunal 1 gap 3).  Idempotent.
 * --------------------------------------------------------------------- */
IMPORT void motor_estop(void);
IMPORT bool motion_estop_latched(void);

#endif /* SUBSYSTEM2_H */
