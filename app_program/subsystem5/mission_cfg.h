/*
 *  mission_cfg.h
 *  Subsystem 5 -- mission and obstacle tuning.  Owner: Buddy 5.
 *  Values marked MEASURE are placeholders (A11, A12, NFR6, FR11).
 *  Pure C header.
 */

#ifndef MISSION_CFG_H
#define MISSION_CFG_H

/* ---- Run parameters (defaults; MQTT "set" changes them in IDLE) ------- */
#define MISSION_CRUISE_MM_S         250     /* param 1, 150..350 (NFR11)        */
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
   junction centre.  MEASURE.                                            */
#define MISSION_NAV_CENTRE_MM       55
#define MISSION_NAV_STRAIGHT_MM     60      /* clear the crossing before following again */
#define MISSION_NAV_SPEED_MM_S      200
#define MISSION_TURN_RATE_DEG_S     120

/* ---- Boot self-test --------------------------------------------------- */
/* Which self-tests BOOT requires: bit 0 motion (PWM + PIO), bit 1 line
   (ADC + DMA), bit 2 IMU (GY-511 answers on I2C1), bit 3 1 us timer.  Only
   the IMU needs wired hardware: until the GY-511 is connected, use 0x0B
   (docs/SETUP_AND_TEST.md, stage 1), and put 0x0F back for every run.   */
#define MISSION_SELFTEST_MASK       0x0Fu

/* ---- Supervision ----------------------------------------------------- */
#define MISSION_BOOT_TIMEOUT_MS     3000    /* self-test                       */
#define MISSION_CAL_STILL_MS        1000    /* IMU level offsets               */
#define MISSION_CAL_SPIN_RATE       90      /* deg/s, 360 deg spin             */
#define MISSION_CAL_DONE_TIMEOUT_MS 2000
#define MISSION_NAV_TIMEOUT_MS      10000
#define MISSION_OBST_TIMEOUT_MS     25000   /* > ObstacleTask's 10 s BLOCKED   */
#define MISSION_AVOID_TIMEOUT_MS    15000
#define MISSION_RECOVER_TIMEOUT_MS  10000
#define MISSION_SEARCH_RATE_DEG_S   60      /* LINE_SEARCH sweep +-60 deg      */
#define MISSION_SEARCH_ANGLE_DDEG   600     /* 60.0 deg                        */

/* ---- Obstacle geometry (FR10, FR11) ---------------------------------- */
#define OBST_ROBOT_WIDTH_MM         130     /* MEASURE: outer width            */
#define OBST_ROBOT_LENGTH_MM        180     /* MEASURE: sensor to rear         */
#define OBST_MARGIN_MM              50      /* FR11 clearance                  */
#define OBST_MAX_RANGE_MM           800     /* profile only what is this close */
#define OBST_DEPTH_TOL_MM           60      /* same object: within r_min + this; a flat
                                               face 20 cm wide at 15-20 cm spans < 30 mm.
                                               MEASURE with angled boxes (A11)        */
#define OBST_HALF_BEAM_CDEG         750     /* HC-SR04 ~15 deg beam: MEASURE (A11) */
#define OBST_CLEAR_MAX_MM           500     /* nothing seen: this much free    */
#define OBST_TOO_CLOSE_MM           80      /* < 8 cm: TOO_CLOSE (FR15)        */
#define OBST_TURN_OUT_DDEG          600     /* bypass turn-out 60.0 deg        */
#define OBST_BYPASS_SPEED_MM_S      250
#define OBST_BYPASS_TURN_DEG_S      180
#define OBST_PASS_MAX_MM            600
#define OBST_SEEK_MAX_MM            500     /* FR11: back on the line within 0.5 m */
#define OBST_SEEK_SPEED_MM_S        150
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
#define OBST_SERVO_LEFT_IS_HIGH     1       /* 1: larger angle looks left      */
#define OBST_NO_ECHO_PINGS          5       /* report NO_ECHO after this many  */
#define OBST_HUMP_MASK_CDEG         500     /* |pitch| > 5 deg masks 40 / 20 cm */
#define OBST_HUMP_MASK_MM           600
#define OBST_HUMP_MASK_MS           4000
#define OBST_ECHO_MAX_US            23200   /* 4 m; longer = no target          */

#endif /* MISSION_CFG_H */
