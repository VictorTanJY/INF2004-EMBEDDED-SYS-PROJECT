/*
 *  mission_cfg.h
 *  Subsystem 5 -- mission and obstacle tuning.  Owner: Buddy 5.
 *  Values marked MEASURE are placeholders (A11, A12, NFR6, FR11).
 *  Pure C header.
 */

#ifndef MISSION_CFG_H
#define MISSION_CFG_H

/* ---- Run parameters (defaults; MQTT "set" changes them in IDLE) ------- */
#define MISSION_CRUISE_MM_S         90      /* param 1 (MQTT set: 150..350)     */
#define MISSION_SLOW_MM             400     /* param 2: slow-down below 40 cm   */
#define MISSION_STOP_MM             200     /* param 3: stop below 20 cm (FR9)  */
#define MISSION_LOST_MM             80      /* param 4: line lost (FR2)         */
/* param 5: junction length.  FR2 says 20 mm, but a crossing line is only
   18 mm wide (Barcode & Line Specification), so both sensors can be fully
   black for less than 20 mm.  Default 12 mm; MEASURE on the junction track. */
#define MISSION_JUNCTION_MM         12

#define MISSION_SLOW_SPEED_MM_S     150     /* after SLOW_DOWN (FR9)            */
#define MISSION_HUMP_SPEED_MM_S     150     /* on a hump (Week 6 Section 5)     */

/* ---- Navigation at a junction (FR4) ---------------------------------- */
/* Line sensors to wheel axle: drive this far so the robot turns about the
   junction centre.  Measured 3 Nov: 120 mm (OBST_IR_AHEAD_MM).          */
#define MISSION_NAV_CENTRE_MM       120
#define MISSION_NAV_STRAIGHT_MM     60      /* clear the crossing before following again */
#define MISSION_NAV_SPEED_MM_S      200
#define MISSION_TURN_RATE_DEG_S     120

/* ---- Boot self-test --------------------------------------------------- */
/* Which self-tests BOOT requires: bit 0 motion (PWM + PIO), bit 1 line
   (ADC + DMA), bit 2 IMU (GY-511 answers on I2C1), bit 3 1 us timer.  Only
   the IMU needs wired hardware: until the GY-511 is connected, use 0x0B
   (docs/SETUP_AND_TEST.md, step B2), and put 0x0F back for every run.   */
#define MISSION_SELFTEST_MASK       0x0Fu

/* ---- Supervision ----------------------------------------------------- */
#define MISSION_BOOT_TIMEOUT_MS     3000    /* self-test                       */
#define MISSION_CAL_STILL_MS        1500    /* IMU level: 0.5 s hands-off + 1 s */
#define MISSION_CAL_SPIN_RATE       90      /* deg/s, 360 deg spin             */
#define MISSION_CAL_DONE_TIMEOUT_MS 2000
#define MISSION_NAV_TIMEOUT_MS      10000
#define MISSION_OBST_TIMEOUT_MS     60000   /* > BLOCKED 10 s + the 4 Nov edge scan */
#define MISSION_AVOID_TIMEOUT_MS    90000   /* 5 Nov: 40 mm/s bypass with the edge scan takes ~60 s */
#define MISSION_RECOVER_TIMEOUT_MS  30000   /* 0.5 m at 30 mm/s + the sweep */
#define MISSION_SEARCH_RATE_DEG_S   60      /* LINE_SEARCH sweep +-60 deg      */
#define MISSION_SEARCH_ANGLE_DDEG   600     /* 60.0 deg                        */

/* ---- Obstacle geometry (FR10, FR11) ---------------------------------- */
#define OBST_ROBOT_WIDTH_MM         130     /* MEASURE: outer width            */
#define OBST_ROBOT_LENGTH_MM        180     /* MEASURE: sensor to rear         */
/* Measured 3 Nov with a ruler (car geometry, fixed by the chassis):     */
#define OBST_US_AHEAD_MM            85      /* ultrasonic front to wheel axle  */
#define OBST_IR_AHEAD_MM            120     /* IR line sensors to wheel axle   */
#define OBST_MARGIN_MM              50      /* FR11 clearance                  */
#define OBST_MAX_RANGE_MM           800     /* profile only what is this close */
#define OBST_DEPTH_TOL_MM           60      /* same object: within r_min + this; a flat
                                               face 20 cm wide at 15-20 cm spans < 30 mm.
                                               MEASURE with angled boxes (A11)        */
#define OBST_HALF_BEAM_CDEG         750     /* HC-SR04 ~15 deg beam: MEASURE (A11) */
/* 3 Nov, after the instructor's guidance (25 Oct forum: estimate, not
   measure; median-filtered pings, background subtraction, consecutive-
   angle edges, conservative margins, more than one scan position, plan
   with the maximum expected width) and two runs that read a bottle as
   2 mm and as 586 mm wide:
   - the reported width keeps the beam correction (best estimate, graded),
     but the PLAN uses the uncorrected edges (the beam can only make an
     object look wider, so this side is safe);
   - a reported width outside OBST_WIDTH_MIN..MAX, or an edge at the end
     of the scan window, is "unsure": the car backs off and scans again
     once, and if still unsure plans as if the object were OBST_WIDTH_MAX
     wide around its nearest point (instructor: obstacles 10-20 cm).     */
#define OBST_WIDTH_MIN_MM           80
#define OBST_WIDTH_MAX_MM           200
#define OBST_SECOND_LOOK_MM         60      /* back-off before the second scan */
#define OBST_SCAN_PINGS             3       /* median of this many pings per angle */
/* 3 Nov guard: an echo within +-10 deg of straight ahead nearer than
   this (stop distance + 5 cm) forbids CONTINUE (see obst_profile).    */
#define OBST_GUARD_HALF_DEG         10
#define OBST_GUARD_MM               (MISSION_STOP_MM + 50)
/* After the bypass: if the straight seek finds no line, sweep +-45 deg on
   the spot, accepting any line the sensors see. */
#define OBST_SWEEP_DDEG             450
#define OBST_SWEEP_TURN_DEG_S       MISSION_CAL_SPIN_RATE
#define OBST_CLEAR_MAX_MM           500     /* nothing seen: this much free    */
#define OBST_TOO_CLOSE_MM           80      /* < 8 cm: TOO_CLOSE (FR15)        */
/* 3 Nov bypass: 45 deg turns, slow, turns at the calibration spin rate
   (where the turn slip is measured, line_slip_gain) and scaled by that
   slip; back in at 45 deg until a line sensor touches the line, then
   OBST_IR_AHEAD_MM on (axle on the line), then turn onto it.            */
#define OBST_TURN_OUT_DDEG          450     /* bypass turn-out 45.0 deg        */
#define OBST_BYPASS_SPEED_MM_S      40      /* 5 Nov: slower legs slip less; MOTION_PROFILE_V_MIN is 40 */
#define OBST_BYPASS_TURN_DEG_S      MISSION_CAL_SPIN_RATE
#define OBST_JOIN_MAX_PERMILLE      1500    /* back in: give up after 1.5 x the step-out leg */
#define OBST_JOIN_EXTRA_MM          100     /* ... + this                      */
/* 4 Nov: the car overshot the line after the fixed 45 deg turn back.
   Now it spins slowly toward the line's direction and stops where the
   line is centred between the two line sensors (measured, not assumed:
   the line may curve there); up to the turn-out angle + this.          */
#define OBST_LINESCAN_DEG_S         30
/* 4 Nov edge scan (as the reference robot measures its bypass): after the
   servo scan, the sensor looks straight ahead and the CAR turns in steps
   to each side until the echo leaves the obstacle's face; the edge comes
   from the car's heading (encoders / turn slip), so it does not depend on
   the servo's angle or direction.                                       */
#define OBST_EDGE_STEP_DDEG         50      /* chassis step, 5 deg               */
#define OBST_EDGE_MAX_DDEG          500     /* look at most 50 deg to each side  */
#define OBST_EDGE_TURN_DEG_S        30
#define OBST_EDGE_SETTLE_MS         150     /* after each step, before the pings */
/* 4 Nov: 40 let something beside the box, ~28 mm behind its face, count
   as the face (right edge read 20 cm right of the line).  25 as the
   reference robot measured it (80 too loose, 35 cut faces at 25-30 deg). */
#define OBST_EDGE_BAND_MM           25      /* echo within this behind the face plane = face */
#define OBST_EDGE_MISSES            2u      /* clear readings in a row that end a side */
/* 5 Nov side choice: the smaller step-out; within OBST_TIE_MM of each
   other they count as equal, then the side whose edge was measured (not
   filled in), then OBST_PREF_LEFT.                                       */
#define OBST_TIE_MM                 20
/* 6 Nov: free room beside the edge-scan edges, from the servo scan's
   points.  A side with less than car + 2 margins of room (a wall, a bag)
   is not used.  Points within OBST_ROOM_TOL of an edge count as the
   obstacle itself (its edge can read a few cm short, and the beam
   smears it sideways): that is how the old check took a box's own half
   for a second object.  Flat things (the mat edge) give no echo.      */
#define OBST_ROOM_TOL_MM            70
#define OBST_PREF_LEFT              1       /* 1: pass on the left when equal, 0: right */
#define OBST_LINESCAN_SPARE_DDEG    450
#define OBST_LINESCAN_CENTRE_ERR    300     /* |(L - R)/(L + R)| x 1000 at or below this = centred */
#define OBST_PASS_MAX_MM            800     /* 3 Nov: room to extend past a seen side */
#define OBST_SEEK_MAX_MM            500     /* FR11: back on the line within 0.5 m */
#define OBST_SEEK_SPEED_MM_S        30      /* 5 Nov: creep to the line */
#define OBST_SEEK_LAT_MM            60      /* accept black only within +-60 mm */
#define OBST_SEEK_HEADING_CDEG      2000    /* ... and +-20 deg of the original heading */
#define OBST_REVERSE_MM             100
#define OBST_REVERSE_TRIES          2
#define OBST_BLOCKED_RESCAN_MS      2000
#define OBST_BLOCKED_RESCANS        5
#define OBST_BLOCKED_TIMEOUT_MS     10000

/* ---- Scanning --------------------------------------------------------- */
#define OBST_SERVO_CENTRE_DEG       90
#define OBST_FINE_STEP_DEG          5       /* FR10 fine scan step               */
/* +-40 deg: a 20 cm box at 20 cm spans +-27 deg plus half the beam, so the
   Week 6 +-30 deg window clipped it (host test S5-H10).                 */
#define OBST_FINE_HALF_DEG          40
#define OBST_SCAN_MIN_DEG           20
#define OBST_SCAN_MAX_DEG           160
#define OBST_REFINE_ITERS           2       /* bisection pings per edge: 5 -> ~1 deg */
#define OBST_EDGE_ALLOW_MM          15      /* bypass: extra offset for width-estimate error */
#define OBST_SERVO_MS_PER_DEG       2       /* SG90 ~0.1 s / 60 deg, + margin  */
#define OBST_SERVO_SETTLE_MS        20
#define OBST_SERVO_MIN_US           500     /* pulse at 0 deg   MEASURE        */
#define OBST_SERVO_MAX_US           2500    /* pulse at 180 deg MEASURE        */
/* 6 Nov, measured on the car: a box placed LEFT of the line read 7-20 cm
   RIGHT in both servo scans (and a centred box read mostly right): the
   mounted servo turns the other way, so the driver mirrors the pulse
   (scan angle > 90 still means left).                                  */
#define OBST_SERVO_LEFT_IS_HIGH     0       /* 1: a longer pulse turns it left */
#define OBST_NO_ECHO_PINGS          5       /* report NO_ECHO after this many  */
#define OBST_HUMP_MASK_CDEG         500     /* |pitch| > 5 deg masks 40 / 20 cm */
#define OBST_HUMP_MASK_MM           600
#define OBST_HUMP_MASK_MS           4000
#define OBST_ECHO_MAX_US            23200   /* 4 m; longer = no target          */

#endif /* MISSION_CFG_H */
