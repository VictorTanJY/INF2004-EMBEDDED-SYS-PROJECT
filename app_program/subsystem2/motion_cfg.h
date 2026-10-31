/*
 *  motion_cfg.h
 *  Subsystem 2 -- robot geometry and controller tuning.  Owner: Buddy 2.
 *
 *  EVERY value marked MEASURE is a placeholder until it is measured on the
 *  robot (assumptions A4, A5; NFR4, NFR5).  docs/SETUP_AND_TEST.md says how.
 *  Pure C header: included by motion_core.c and the host tests.
 */

#ifndef MOTION_CFG_H
#define MOTION_CFG_H

/* ---- Geometry ------------------------------------------------------- */
/* Encoder counts per WHEEL revolution, x4 quadrature.  MEASURE (A4):
   turn each wheel 10 revolutions by hand and read enc_l / enc_r.
   Measured 2026-11-01: about 2530 per wheel turn, both wheels.         */
#define MOTION_COUNTS_PER_REV       2536
/* Wheel circumference in micrometres (pi x diameter).  MEASURE: drive
   1 m five times against a tape measure and correct this value.        */
#define MOTION_WHEEL_CIRC_UM        209309      /* measured 2026-11-01 */
/* Effective track width (wheel contact centre to centre).  MEASURE (A5):
   10 x 360 deg turns, scale by commanded / actual angle.                */
#define MOTION_TRACK_MM             106
/* Encoder direction: +1 if the count rises when the wheel drives the robot
   forward, otherwise -1.  MEASURE (A4).                                 */
#define MOTION_ENC_SIGN_L           (-1)
#define MOTION_ENC_SIGN_R           (1)
/* Motor direction: +1 if "A = PWM, B = 0" drives the robot forward.     */
#define MOTION_MOT_SIGN_L           (1)
#define MOTION_MOT_SIGN_R           (-1)

/* ---- Limits --------------------------------------------------------- */
#define MOTION_V_MAX_MM_S           450         /* set-point clamp          */
#define MOTION_ACCEL_MM_S2          800         /* MOVE / TURN profile      */
#define MOTION_STOP_DECEL_MM_S2     1000        /* STOPPING ramp, ~1 m/s^2  */
#define MOTION_DRIVE_TIMEOUT_MS     100         /* no DRIVE -> stop (6.1)   */
#define MOTION_DUTY_MAX             1000        /* per mille                */

/* ---- Wheel-speed PID, duty in per mille.  MEASURE (NFR5): tune so the
   step response settles within 300 ms with < 10 % steady error.  The
   controller: duty = ff * sp + kp * e + ki * sum(e) * dt, with the
   integrator clamped and frozen while the output saturates.            */
#define MOTION_KFF_MILLI            2000        /* 2.0 per mille per mm/s   */
#define MOTION_KP_MILLI             1500        /* 1.5 per mille per mm/s   */
#define MOTION_KI_MILLI             8000        /* 8.0 per mille per mm     */
#define MOTION_I_MAX                400         /* per mille                */
#define MOTION_MIN_DUTY             120         /* static friction offset; MEASURE */

/* ---- Manoeuvres ----------------------------------------------------- */
#define MOTION_POS_TOL_UM           2000        /* MOVE done within 2 mm    */
#define MOTION_ANG_TOL_CDEG         100         /* TURN done within 1 deg   */
#define MOTION_SETTLE_MS            100         /* brake time before DONE   */
#define MOTION_STRAIGHT_K           40          /* mrad/s per mm of wheel difference */
#define MOTION_TIMEOUT_MARGIN_MS    500         /* timeout = 2 x expected + margin */

/* ---- Faults --------------------------------------------------------- */
#define MOTION_STALL_DUTY           600         /* |duty| above this ...    */
#define MOTION_STALL_SPEED_MM_S     10          /* ... with |v| below this  */
#define MOTION_STALL_MS             300         /* ... for this long = STALL */
/* More counts in one 10 ms cycle than the motor can produce = glitch
   (ENC_FAULT, report only): 3 x the counts at MOTION_V_MAX_MM_S.
   counts per 10 ms = v [mm/s] x CPR x 10 / circumference [um].        */
#define MOTION_ENC_GLITCH_COUNTS    ((3 * MOTION_V_MAX_MM_S * MOTION_COUNTS_PER_REV * 10) / \
                                     MOTION_WHEEL_CIRC_UM)
#define MOTION_PROFILE_V_MIN        40          /* mm/s floor near the target */

/* ---- PWM: 125 MHz / (TOP + 1) = 20 kHz (NFR1) ----------------------- */
#define MOTION_PWM_TOP              6249u

#endif /* MOTION_CFG_H */
