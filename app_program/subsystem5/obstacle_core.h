/*
 *  obstacle_core.h
 *  Subsystem 5 -- ObstacleTask geometry as pure functions (Week 6 Section
 *  6.4, FR9-FR12).  Owner: Buddy 5.  Host-tested.
 *
 *  Servo angles in degrees: 90 = straight ahead, larger = further left
 *  (OBST_SERVO_LEFT_IS_HIGH maps the real servo).  A point (a, r) lies at
 *  forward = r sin a and lateral = -r cos a (+ left) from the sensor.
 */

#ifndef OBSTACLE_CORE_H
#define OBSTACLE_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "mission_types.h"
#include "mission_cfg.h"

#define OBST_SCAN_MAX_POINTS    32u

typedef struct
{
    int16_t  angle_deg;
    uint16_t r_mm;              /* OBST_RANGE_NONE = no echo / out of range */
} obst_point_t;

typedef struct
{
    bool     b_valid;           /* something within OBST_MAX_RANGE_MM      */
    uint16_t closest_mm;
    int16_t  nearest_angle;     /* angle of the closest point              */
    int16_t  left_lat_mm;       /* edges, beam-corrected                   */
    int16_t  right_lat_mm;
    uint16_t w_mm;
    uint16_t clr_l_mm;          /* free lateral space beyond each edge     */
    uint16_t clr_r_mm;
    uint8_t  plan;              /* obst_plan_t                             */
    bool     b_sure;            /* width plausible and both edges inside the window */
    int16_t  plan_l_mm;         /* edges the plan used (+ left)            */
    int16_t  plan_r_mm;
    int16_t  deviation_mm;      /* lateral offset for the chosen bypass    */
} obst_profile_t;

/* Median of three ranges (OBST_RANGE_NONE counts as far away). */
uint16_t obst_median3(uint16_t a, uint16_t b, uint16_t c);

/* HC-SR04 echo width to distance: us / 58 = cm (datasheet).  Widths of
   OBST_ECHO_MAX_US (4 m) or more mean no target: OBST_RANGE_NONE. */
uint16_t obst_echo_to_mm(uint32_t width_us);

/* Profile one obstacle from the fine scan (and the coarse scan for the
   clearances), then choose CONTINUE / LEFT / RIGHT / BLOCKED. */
void     obst_profile(obst_point_t const * p_fine, uint8_t n_fine,
                      obst_point_t const * p_coarse, uint8_t n_coarse,
                      obst_profile_t * p_out);

/* As obst_profile(), for an obstacle the monitor already saw at r_det_mm
   straight ahead: if the scan saw nothing, plan for an OBST_WIDTH_MAX
   object there (never CONTINUE into a detected obstacle). */
void     obst_profile_detected(obst_point_t const * p_fine, uint8_t n_fine,
                               obst_point_t const * p_coarse, uint8_t n_coarse,
                               uint16_t r_det_mm, uint16_t back_mm, obst_profile_t * p_out);

/* ---- Edge scan (4 Nov): the car turns, the sensor looks straight ahead.
   phi = the car's real heading from where it stopped (0.01 deg, + left),
   r = range from the sensor (OBST_US_AHEAD_MM ahead of the axle).       */
/* The echo comes from the obstacle's face (not something behind it). */
bool    obst_edge_on_face(int32_t phi_cdeg, uint16_t r_mm, uint16_t face_mm);
/* First edge estimate at the last on-face heading (7 Nov): the sensor's
   own lateral position (mm, + left).  The sensor swings with the car and
   a flat face echoes while the sensor is still behind it (9.5 cm box:
   4.6 / 5.0 cm, true 4.75).  It can fall short (blue box left side), so
   it is only the step-out leg's minimum: obst_out_* measure the rest. */
int32_t obst_edge_lat(int32_t phi_cdeg);
/* The same echo placed on the corner with the beam's inner edge
   (OBST_HALF_BEAM_CDEG, as before 7 Nov): the widest believable edge on
   side +1 / -1.  Used for the past-max test and the side choice (the
   sensor positions above stay within OBST_US_AHEAD_MM, too close
   together to tell the sides apart). */
int32_t obst_edge_wide_lat(int32_t phi_cdeg, uint16_t r_mm, int32_t side);
/* That edge is further out from the line than an OBST_WIDTH_MAX obstacle's
   could be on this side (as the reference robot's "past max"): it is
   something else, the side's edge is not known. */
bool    obst_edge_past_max(int32_t lat_mm, int32_t side);
/* Reported width (7 Nov) from the last echo on each side (heading phi,
   range r), the beam's inner edge on the corner at OBST_WIDTH_BEAM_CDEG.
   Only the dashboard's width: the bypass does not use it. */
uint16_t obst_edge_width(int32_t phi_l_cdeg, uint16_t r_l_mm, int32_t phi_r_cdeg, uint16_t r_r_mm);
/* Re-plan p from the edge scan: edges, width, side (away from the
   obstacle's centre), offset.  A side whose edge was not found takes the
   servo scan's edge (p on entry) if it saw a real width, else reaches
   OBST_WIDTH_MAX from the other.  Neither found: the servo scan's plan
   edges, with the same side rule (p unchanged if that scan saw nothing). */
void    obst_edge_plan(obst_profile_t * p, bool b_has_l, int32_t lat_l, bool b_has_r, int32_t lat_r);
/* 7 Nov: after obst_edge_plan (given the far edges): the offset on the
   chosen side becomes the step-out leg's minimum, from the sensor-position
   edge (obst_edge_lat) when that side had an echo. */
void    obst_edge_plan_min(obst_profile_t * p, bool b_has_l, int32_t min_l, bool b_has_r, int32_t min_r);

/* ---- Scan sequencer: COARSE (30..150 by 30) -> FINE (nearest +-40 by 5)
   -> REFINE each edge by bisection.  The task pings obst_scan_angle(),
   feeds the range with obst_scan_feed() until obst_scan_done(). */
typedef enum
{
    OBST_SCAN_COARSE   = 0,
    OBST_SCAN_FINE     = 1,
    OBST_SCAN_REFINE_R = 2,
    OBST_SCAN_REFINE_L = 3,
    OBST_SCAN_DONE     = 4
} obst_scan_phase_t;

typedef struct
{
    uint8_t      phase;
    obst_point_t coarse[5];
    uint8_t      n_coarse;
    obst_point_t fine[OBST_SCAN_MAX_POINTS];
    uint8_t      n_fine;
    int16_t      angle;             /* next angle to ping            */
    int16_t      fine_to;
    int16_t      hit;               /* refine: inner (hit) angle      */
    int16_t      miss;              /* refine: outer (miss) angle     */
    uint16_t     r_min;
    uint8_t      iter;
} obst_scan_t;

void    obst_scan_start(obst_scan_t * p_s);
int16_t obst_scan_angle(obst_scan_t const * p_s);
void    obst_scan_feed(obst_scan_t * p_s, uint16_t r_mm);
bool    obst_scan_done(obst_scan_t const * p_s);
void    obst_scan_profile(obst_scan_t const * p_s, obst_profile_t * p_out);
void    obst_scan_profile_detected(obst_scan_t const * p_s, uint16_t r_det_mm, uint16_t back_mm,
                                   obst_profile_t * p_out);

/* Bypass path: up to OBST_BYPASS_MAX_STEPS steps.  TURN values are real
   degrees; the task scales them by the measured turn slip. */
typedef enum
{
    OBST_STEP_MOVE = 1,         /* value = mm (+ forward)                  */
    OBST_STEP_TURN = 2,         /* value = 0.1 deg (+ left)                */
    OBST_STEP_PASS = 3,         /* drive on, servo watching the obstacle   */
    OBST_STEP_JOIN = 4,         /* drive on until a line sensor sees the
                                   line; value = give up after this, mm    */
    OBST_STEP_LINESCAN = 5,     /* spin slowly until the line is centred
                                   under the sensors; value = most 0.1 deg (+ left) */
    OBST_STEP_OUT = 6           /* 7 Nov: step out at the turned heading,
                                   the servo looking along the line at the
                                   face, until clear of it (obst_out_*);
                                   value = planned leg, mm               */
} obst_step_kind_t;

typedef struct
{
    uint8_t kind;
    int16_t value;
} obst_step_t;

#define OBST_BYPASS_MAX_STEPS   10u

uint8_t  obst_bypass_plan(obst_profile_t const * p_prof, uint8_t side, obst_step_t * p_steps);

/* ---- Step-out leg (7 Nov).  Lateral values are mm from the line toward
   the bypass side; theta = the car's real heading from the line (0.01
   deg, toward the side, > 0); face_mm = the face's distance ahead of the
   axle where the car stopped (before any back-up). */
typedef struct
{
    int32_t min_lat;            /* the plan's offset: never step out less  */
    int32_t seen_lat;           /* sensor lateral at the furthest face echo */
    bool    b_seen;
    uint8_t pings;
} obst_out_t;

void    obst_out_start(obst_out_t * p_o, int32_t min_lat);
/* Range at which the face would echo, the sensor looking along the line. */
int32_t obst_out_expect_mm(int32_t face_mm, int32_t axle_fwd_mm, int32_t theta_cdeg);
/* One ping: the axle is axle_lat out and axle_fwd on from the stop. */
void    obst_out_ping(obst_out_t * p_o, int32_t axle_lat, int32_t axle_fwd, int32_t theta_cdeg,
                      uint16_t r_mm, int32_t face_mm);
/* Axle lateral to reach: clear of the furthest face echo by the car's
   half width + margin + allowance, never less than the plan, never more
   than past an OBST_WIDTH_MAX obstacle's furthest possible edge. */
int32_t obst_out_target(obst_out_t const * p_o);
bool    obst_out_done(obst_out_t const * p_o, int32_t axle_lat);
/* The car's front corner is within OBST_MARGIN_MM of the face plane: the
   leg must end here, whatever it has seen. */
bool    obst_out_blocked(int32_t face_mm, int32_t axle_fwd_mm, int32_t theta_cdeg);
/* ---- Width probe (7 Nov): sensor lateral positions (mm, + left) of the
   face echoes while the car side-steps past both edges. */
typedef struct
{
    bool    b_seen;
    int32_t lo;                 /* rightmost sensor lateral with a face echo */
    int32_t hi;                 /* leftmost                                  */
    uint8_t off;                /* clear pings in a row                      */
} obst_probe_t;

void    obst_probe_start(obst_probe_t * p_p);
/* Sensor lateral (+ left) when the axle is axle_lat out and the car turned
   theta (0.01 deg, + left) with the servo looking back along the line. */
int32_t obst_probe_sensor_lat(int32_t axle_lat, int32_t theta_cdeg);
/* One ping; true if it was the face (within OBST_DEPTH_TOL of expect). */
bool    obst_probe_ping(obst_probe_t * p_p, int32_t sensor_lat, uint16_t r_mm, int32_t expect_mm);
/* Edges with the echo run-on (OBST_PROBE_EDGE_OVER_MM) taken off; false if
   the face was never seen.  Never narrower than nothing. */
bool    obst_probe_edges(obst_probe_t const * p_p, int32_t * p_left, int32_t * p_right);

/* Servo angle (deg, > 90 left) that looks along the line from a car
   turned theta toward side +1 (left) / -1 (right). */
int32_t obst_out_servo_deg(int32_t theta_cdeg, int32_t side);

/* Cross-track offset (+ left) of (x, y) from the line through (x0, y0)
   with heading h0, all from the odometry snapshot. */
int32_t  obst_cross_track_mm(int32_t x, int32_t y, int32_t x0, int32_t y0, int32_t h0_cdeg);

/* SEEK_LINE acceptance (Week 6 6.4): within +-60 mm of the original path
   and +-20 deg of the original heading. */
bool     obst_line_accept(int32_t x, int32_t y, int32_t h_cdeg,
                          int32_t x0, int32_t y0, int32_t h0_cdeg);

/* Hump mask for the 40 / 20 cm thresholds (never for TOO_CLOSE). */
typedef struct
{
    bool     b_active;
    uint32_t start_ms;
    int32_t  start_dist_mm;
} obst_mask_t;

bool     obst_hump_mask(obst_mask_t * p_m, bool b_pitch_valid, int32_t pitch_cdeg,
                        int32_t dist_mm, uint32_t now_ms);

#endif /* OBSTACLE_CORE_H */
