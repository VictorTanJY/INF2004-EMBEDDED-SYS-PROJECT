/*
 *  motion_core.h
 *  Subsystem 2 -- the MotionTask controller as a pure state machine
 *  (Week 6 Section 6.1).  Owner: Buddy 2 (Jedrek).
 *
 *  No kernel, register or SDK access: MotionTask (motion_task.c) feeds it
 *  encoder counts and commands every 10 ms and writes the duty it returns,
 *  and testing/subsystem2/host runs the same code against a motor model.
 *
 *  States: IDLE_BRAKED, VELOCITY_TRACK (DRIVE), MANEUVER (MOVE / TURN with
 *  a trapezoidal profile ACCEL -> CRUISE -> DECEL -> SETTLE), STOPPING
 *  (ramp at MOTION_STOP_DECEL_MM_S2, then brake) and ESTOP_LATCHED.
 */

#ifndef MOTION_CORE_H
#define MOTION_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "motion_types.h"

typedef enum
{
    MOTION_RES_ACCEPTED      = 0,
    MOTION_RES_REJECTED_AUTH = 1,   /* source does not hold authority       */
    MOTION_RES_REJECTED_ESTOP= 2,   /* latched: only CLEAR_ESTOP is possible */
    MOTION_RES_INVALID       = 3,   /* bad code or parameter                */
    MOTION_RES_CLEAR_ESTOP   = 4    /* valid CLEAR_ESTOP: the task must clear
                                       the latch, reconnect PWM, then call
                                       motion_core_estop_cleared()          */
} motion_result_t;

typedef enum
{
    MOTION_EV_NONE      = 0,
    MOTION_EV_DONE      = 1,        /* MOVE / TURN completed: seq, src      */
    MOTION_EV_FAILED    = 2,        /* timeout, pre-empted or E-stop: seq, src */
    MOTION_EV_STALL     = 3,        /* set SAFETY_FLT_STALL                 */
    MOTION_EV_ENC_FAULT = 4,        /* report only                          */
    MOTION_EV_ENC_OK    = 5         /* glitches stopped for 1 s             */
} motion_ev_code_t;

typedef struct
{
    uint8_t code;                   /* motion_ev_code_t */
    uint8_t seq;
    uint8_t src;
} motion_event_t;

#define MOTION_EV_QUEUE_LEN     6u

typedef struct
{
    /* state machine */
    uint8_t  state;                 /* motion_state_t   */
    uint8_t  phase;                 /* motion_phase_t   */
    uint8_t  auth;                  /* motion_src_t     */
    uint16_t rejected;

    /* odometry */
    bool     b_have_prev;
    int32_t  enc_l;                 /* sign-corrected counts */
    int32_t  enc_r;
    int64_t  dist_l_um;
    int64_t  dist_r_um;
    int64_t  x_um;
    int64_t  y_um;
    int32_t  heading_cdeg;
    int32_t  v_l;                   /* measured, mm/s (filtered) */
    int32_t  v_r;
    int32_t  yaw_rate_cdeg_s;
    uint32_t prev_ms;

    /* set-points (mm/s, mrad/s) */
    int32_t  sp_v;
    int32_t  sp_w;
    int32_t  drive_v;
    int32_t  drive_w;
    uint32_t last_drive_ms;

    /* manoeuvre */
    uint8_t  m_cmd;                 /* MOTION_CMD_MOVE or _TURN */
    uint8_t  m_seq;
    uint8_t  m_src;
    int8_t   m_dir;                 /* +1 / -1 */
    int64_t  m_target_um;           /* MOVE: distance; TURN: wheel arc  */
    int64_t  m_start_l_um;
    int64_t  m_start_r_um;
    int32_t  m_vmax;                /* mm/s (wheel speed for TURN)      */
    int32_t  m_prof_v;              /* current profile speed, mm/s      */
    uint32_t m_start_ms;
    uint32_t m_timeout_ms;
    uint32_t m_settle_ms;

    /* wheel PID */
    int32_t  integ_l_um;
    int32_t  integ_r_um;
    int16_t  duty_l;
    int16_t  duty_r;

    /* faults */
    uint32_t stall_since_ms;
    bool     b_stall_timing;
    bool     b_stall_reported;
    bool     b_enc_fault;
    uint32_t enc_glitch_ms;

    /* completions */
    uint8_t  last_done_seq;
    uint8_t  last_done_src;
    uint8_t  last_fail_seq;
    uint8_t  done_count;
    uint8_t  fail_count;

    /* events of the current cycle */
    motion_event_t ev[MOTION_EV_QUEUE_LEN];
    uint8_t        ev_count;
} motion_core_t;

void            motion_core_init(motion_core_t * p_mc, uint32_t now_ms);

/* Apply one command (call for every message drained this cycle). */
motion_result_t motion_core_command(motion_core_t * p_mc, motion_cmd_t const * p_cmd,
                                    uint32_t now_ms);

/* The task cleared the latch and reconnected PWM: leave ESTOP_LATCHED. */
void            motion_core_estop_cleared(motion_core_t * p_mc);

/*
 * One 10 ms control step.  enc_l / enc_r are the raw PIO counts (the core
 * applies MOTION_ENC_SIGN_*).  b_estop = the latch is set.  Afterwards
 * p_mc->duty_l / duty_r hold the per-mille duty to write (0 / 0 = brake),
 * already in robot terms (+ = forward); the driver applies MOTION_MOT_SIGN_*.
 */
void            motion_core_step(motion_core_t * p_mc, int32_t enc_l, int32_t enc_r,
                                 bool b_estop, uint32_t now_ms);

/* Fill the public snapshot from the core (t_ms and hw_ok are set by the task). */
void            motion_core_snapshot(motion_core_t const * p_mc, motion_odom_t * p_out);

/* Pop the next event (raised by commands or steps since the last pop);
   false when empty.  MotionTask drains them after every step. */
bool            motion_core_pop_event(motion_core_t * p_mc, motion_event_t * p_ev);

/* Expected duration of a manoeuvre (ms), used for its timeout. */
uint32_t        motion_core_expected_ms(int64_t distance_um, int32_t vmax_mm_s);

#endif /* MOTION_CORE_H */
