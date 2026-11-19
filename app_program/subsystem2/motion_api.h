/*
 *  motion_api.h
 *  Subsystem 2 -- the named motion API (Week 6 FR5; the project brief's
 *  moveForward / moveBackward / turnLeft / turnRight / stop).  Owner: Buddy 2.
 *
 *  Each function posts one command to MotionTask through mbf_motion and
 *  returns at once (non-blocking).  MotionTask runs the move closed-loop on
 *  the encoders and obeys only the source that MissionTask has given motion
 *  authority, so two subsystems cannot drive the motors at the same time.
 *  Completion is reported as MOTION_DONE / MOTION_FAILED (with seq) to
 *  MissionTask and in the odometry snapshot (last_done_seq, last_fail_seq).
 *
 *  Parameters shared by every function:
 *    src  motion_types_src_t of the caller (MOTION_SRC_MISSION, _LINE, _AVOID)
 *    seq  caller's sequence number, echoed in the completion report
 *  Return: 0 (E_OK) when queued; E_TMOUT when mbf_motion is full (the
 *  command is dropped); MOTION_API_E_PAR for an out-of-range argument.
 *
 *  Pure C header: no kernel types, so host tests can link motion_api.c
 *  against their own subsystem2_post().
 */

#ifndef MOTION_API_H
#define MOTION_API_H

#include <stdint.h>
#include "motion_types.h"

#define MOTION_API_E_PAR  (-17) /* same value as the kernel's E_PAR */
#define MOTION_API_MAX_MM 4095  /* one MOVE: up to 4.09 m (13-bit field) */
/* one TURN: up to 409.5 deg (13-bit field) */
#define MOTION_API_MAX_DDEG 4095
#define MOTION_API_MAX_RATE 511 /* mm/s or deg/s (9-bit field)          */

/* Result of every function here: 0 (E_OK), E_TMOUT or MOTION_API_E_PAR.
   NOTE: plain int on purpose: it must be the same type as the kernel's
   error code ER (typedef INT ER, INT = int), which subsystem2_post()
   returns on the robot; int32_t is long int on the Arm compiler. */
typedef int motion_api_err_t;

/* Posts one raw command (subsystem2.c on the robot). */
motion_api_err_t subsystem2_post(uint8_t src, uint8_t seq, uint8_t cmd,
                                 int16_t arg0, int16_t arg1);

/* moveForward(distance): drive distance_mm forward at speed_mm_s, then brake.
 */
motion_api_err_t motion_api_move_forward(uint8_t src, uint8_t seq,
                                         uint16_t distance_mm,
                                         uint16_t speed_mm_s);

/* moveBackward(distance): the same, reversing. */
motion_api_err_t motion_api_move_backward(uint8_t src, uint8_t seq,
                                          uint16_t distance_mm,
                                          uint16_t speed_mm_s);

/* turnLeft(angle) / turnRight(angle): spin in place by angle_ddeg tenths of
   a degree (900 = 90.0 deg) at rate_deg_s. */
motion_api_err_t motion_api_turn_left(uint8_t src, uint8_t seq,
                                      uint16_t angle_ddeg, uint16_t rate_deg_s);
motion_api_err_t motion_api_turn_right(uint8_t src, uint8_t seq,
                                       uint16_t angle_ddeg,
                                       uint16_t rate_deg_s);

/* stop(): cancel any move, ramp down (MOTION_STOP_DECEL), then brake. */
motion_api_err_t motion_api_stop(uint8_t src, uint8_t seq);

/* drive(v, omega): continuous closed-loop driving for the line follower and
   the bypass: v_mm_s forward (+) / back (-), omega_mrad_s turn rate
   (+ left).  Must be refreshed at least every 100 ms, or MotionTask stops. */
motion_api_err_t motion_api_drive(uint8_t src, uint8_t seq, int16_t v_mm_s,
                                  int16_t omega_mrad_s);

#endif /* MOTION_API_H */

/*** end of file ***/
