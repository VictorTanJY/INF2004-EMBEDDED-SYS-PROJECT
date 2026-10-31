/*
 *  motion_types.h
 *  Subsystem 2 -- motion message and snapshot formats (IF-01, IF-02).
 *  Owner: Buddy 2 (Jedrek).
 *
 *  Pure C (no kernel), so motion_core.c, mission_core.c and the host tests
 *  can use it.  subsystem2.h adds the kernel objects.
 */

#ifndef MOTION_TYPES_H
#define MOTION_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------
 *  IF-01  mbf_motion message.
 *  Authority rule (Week 6 Section 6.1): MotionTask obeys only the source
 *  that MissionTask granted with SET_AUTH.  MISSION may always send STOP,
 *  SET_AUTH and CLEAR_ESTOP; its MOVE/TURN need MISSION authority.  A
 *  command from a source without authority is rejected and counted.
 *  Queued commands are judged against the authority in force when
 *  MotionTask reads them; SET_AUTH travels through the same FIFO.
 * --------------------------------------------------------------------- */
typedef enum
{
    MOTION_SRC_MISSION = 0,     /* MissionTask: STOP, MOVE/TURN, SET_AUTH */
    MOTION_SRC_LINE    = 1,     /* LineBarcodeTask: DRIVE                 */
    MOTION_SRC_AVOID   = 2,     /* ObstacleTask during a bypass           */
    MOTION_SRC_COUNT   = 3
} motion_src_t;

typedef enum
{
    MOTION_CMD_NONE        = 0,
    MOTION_CMD_DRIVE       = 1, /* p0 = v [mm/s, +fwd], p1 = omega [mrad/s, +left] */
    MOTION_CMD_MOVE        = 2, /* p0 = distance [mm, +fwd], p1 = cruise v [mm/s, > 0] */
    MOTION_CMD_TURN        = 3, /* p0 = angle [0.1 deg, +left], p1 = rate [deg/s, > 0] */
    MOTION_CMD_STOP        = 4, /* ramp down (MOTION_STOP_DECEL), then brake       */
    MOTION_CMD_SET_AUTH    = 5, /* p0 = new authority (motion_src_t); MISSION only */
    MOTION_CMD_CLEAR_ESTOP = 6  /* MISSION only, after an operator reset           */
} motion_cmd_code_t;

typedef struct
{
    uint8_t src;                /* motion_src_t                          */
    uint8_t seq;                /* sender's sequence number (wraps)      */
    uint8_t cmd;                /* motion_cmd_code_t                     */
    uint8_t reserved;           /* 0                                     */
    int16_t p0;                 /* see motion_cmd_code_t                 */
    int16_t p1;
} motion_cmd_t;                 /* 8 bytes */

/* ---------------------------------------------------------------------
 *  IF-02  Odometry snapshot (written by MotionTask every 10 ms).
 *  Pose: x forward / y left from the power-on position, heading
 *  accumulated (not wrapped), + left.  Encoder odometry only (A5).
 * --------------------------------------------------------------------- */
typedef enum
{
    MOTION_ST_IDLE_BRAKED    = 0,
    MOTION_ST_VELOCITY_TRACK = 1,
    MOTION_ST_MANEUVER       = 2,
    MOTION_ST_STOPPING       = 3,
    MOTION_ST_ESTOP_LATCHED  = 4
} motion_state_t;

typedef enum
{
    MOTION_PH_NONE   = 0,
    MOTION_PH_ACCEL  = 1,
    MOTION_PH_CRUISE = 2,
    MOTION_PH_DECEL  = 3,
    MOTION_PH_SETTLE = 4
} motion_phase_t;

typedef struct
{
    uint32_t t_ms;              /* time of the update (app_time_ms)      */
    int32_t  enc_l;             /* accumulated encoder counts, x4        */
    int32_t  enc_r;
    int32_t  dist_mm;           /* mean of both wheels, signed           */
    int32_t  x_mm;              /* dead-reckoned pose                    */
    int32_t  y_mm;
    int32_t  heading_cdeg;      /* accumulated, + left (0.01 deg)        */
    int16_t  v_l_mm_s;          /* measured wheel speeds                 */
    int16_t  v_r_mm_s;
    int16_t  v_mm_s;            /* mean speed                            */
    int16_t  yaw_rate_cdeg_s;   /* + left                                */
    int16_t  duty_l;            /* applied duty, per mille, signed       */
    int16_t  duty_r;
    uint16_t rejected;          /* commands rejected (wrong authority)   */
    uint8_t  state;             /* motion_state_t                        */
    uint8_t  phase;             /* motion_phase_t (MANEUVER only)        */
    uint8_t  auth;              /* motion_src_t currently obeyed         */
    uint8_t  last_done_seq;     /* seq of the last completed MOVE/TURN   */
    uint8_t  last_done_src;     /* its source                            */
    uint8_t  last_fail_seq;     /* seq of the last failed MOVE/TURN      */
    uint8_t  done_count;        /* + 1 per completion (wraps)            */
    uint8_t  fail_count;        /* + 1 per failure (wraps)               */
    uint8_t  hw_ok;             /* 1 = PWM and PIO encoders initialised  */
    uint8_t  reserved;
} motion_odom_t;

#endif /* MOTION_TYPES_H */
