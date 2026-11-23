/** @file obstacle_core.h
 *
 *  @brief Subsystem 5 -- ObstacleTask geometry as pure functions (Week 6
 * Section 6.4, FR9-FR12).  Owner: Buddy 5.  Host-tested.
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

#define OBST_COARSE_PINGS 5u /* coarse sweep: one ping per 45 deg */

#define OBST_SCAN_MAX_POINTS 32u

typedef struct
{
    int16_t  angle_deg;
    uint16_t r_mm; /**< OBST_RANGE_NONE = no echo / out of range */
} obstacle_core_point_t;

typedef struct
{
    bool     b_valid; /**< something within OBST_MAX_RANGE_MM      */
    uint16_t closest_mm;
    int16_t  nearest_angle; /**< angle of the closest point              */
    int16_t  left_lat_mm;   /**< edges, beam-corrected                   */
    int16_t  right_lat_mm;
    uint16_t w_mm;
    uint16_t clr_l_mm; /**< free lateral space beyond each edge     */
    uint16_t clr_r_mm;
    uint8_t  plan; /**< mission_types_obst_plan_t                             */
    bool     b_sure;    /**< width plausible and both edges inside the window */
    int16_t  plan_l_mm; /**< edges the plan used (+ left)            */
    int16_t  plan_r_mm;
    int16_t  deviation_mm; /**< lateral offset for the chosen bypass    */
} obstacle_core_profile_t;

/** Median of three ranges (OBST_RANGE_NONE counts as far away). */
uint16_t obstacle_core_median3(uint16_t val_a, uint16_t val_b, uint16_t val_c);

/** HC-SR04 echo width to distance: us / 58 = cm (datasheet).  Widths of
   OBST_ECHO_MAX_US (4 m) or more mean no target: OBST_RANGE_NONE. */
uint16_t obstacle_core_echo_to_mm(uint32_t width_us);

/** Profile one obstacle from the fine scan (and the coarse scan for the
   clearances), then choose CONTINUE / LEFT / RIGHT / BLOCKED. */
void obstacle_core_profile(obstacle_core_point_t const * p_fine, uint8_t n_fine,
                           obstacle_core_point_t const * p_coarse,
                           uint8_t n_coarse, obstacle_core_profile_t * p_out);

/** As obstacle_core_profile(), for an obstacle the monitor already saw at
   r_det_mm straight ahead: if the scan saw nothing, plan for an OBST_WIDTH_MAX
   object there (never CONTINUE into a detected obstacle). */
void obstacle_core_profile_detected(obstacle_core_point_t const * p_fine,
                                    uint8_t                       n_fine,
                                    obstacle_core_point_t const * p_coarse,
                                    uint8_t n_coarse, uint16_t r_det_mm,
                                    uint16_t                  back_mm,
                                    obstacle_core_profile_t * p_out);

/* ---- Edge scan (4 Nov): the car turns, the sensor looks straight ahead.
   phi = the car's real heading from where it stopped (0.01 deg, + left),
   r = range from the sensor (OBST_US_AHEAD_MM ahead of the axle).       */
/** The echo comes from the obstacle's face (not something behind it). */
bool obstacle_core_edge_on_face(int32_t phi_cdeg, uint16_t r_mm,
                                uint16_t face_mm);
/** First edge estimate at the last on-face heading (7 Nov): the sensor's
   own lateral position (mm, + left).  The sensor swings with the car and
   a flat face echoes while the sensor is still behind it (9.5 cm box:
   4.6 / 5.0 cm, true 4.75).  It can fall short (blue box left side), so
   it is only the step-out leg's minimum: obst_out_* measure the rest. */
int32_t obstacle_core_edge_lat(int32_t phi_cdeg);
/** The same echo placed on the corner with the beam's inner edge
   (OBST_HALF_BEAM_CDEG, as before 7 Nov): the widest believable edge on
   side +1 / -1.  Used for the past-max test and the side choice (the
   sensor positions above stay within OBST_US_AHEAD_MM, too close
   together to tell the sides apart). */
int32_t obstacle_core_edge_wide_lat(int32_t phi_cdeg, uint16_t r_mm,
                                    int32_t side);
/** That edge is further out from the line than an OBST_WIDTH_MAX obstacle's
   could be on this side: it is
   something else, the side's edge is not known. */
bool obstacle_core_edge_past_max(int32_t lat_mm, int32_t side);
/** Reported width (7 Nov) from the last echo on each side (heading phi,
   range r), the beam's inner edge on the corner at OBST_WIDTH_BEAM_CDEG.
   Only the dashboard's width: the bypass does not use it. */
uint16_t obstacle_core_edge_width(int32_t phi_l_cdeg, uint16_t r_l_mm,
                                  int32_t phi_r_cdeg, uint16_t r_r_mm);
/** Re-plan p from the edge scan: edges, width, side (away from the
   obstacle's centre), offset.  A side whose edge was not found takes the
   servo scan's edge (p on entry) if it saw a real width, else reaches
   OBST_WIDTH_MAX from the other.  Neither found: the servo scan's plan
   edges, with the same side rule (p unchanged if that scan saw nothing). */
void obstacle_core_edge_plan(obstacle_core_profile_t * p_prof, bool b_has_l,
                             int32_t lat_l, bool b_has_r, int32_t lat_r);
/** 7 Nov: after obstacle_core_edge_plan (given the far edges): the offset on
   the chosen side becomes the step-out leg's minimum, from the sensor-position
   edge (obstacle_core_edge_lat) when that side had an echo. */
void obstacle_core_edge_plan_min(obstacle_core_profile_t * p_prof, bool b_has_l,
                                 int32_t min_l, bool b_has_r, int32_t min_r);

/* ---- Scan sequencer: COARSE (30..150 by 30) -> FINE (nearest +-40 by 5)
   -> REFINE each edge by bisection.  The task pings obstacle_core_scan_angle(),
   feeds the range with obstacle_core_scan_feed() until
   obstacle_core_scan_done(). */
typedef uint8_t obstacle_core_scan_phase_t; /* one of the constants below */
#define OBST_SCAN_COARSE   0u
#define OBST_SCAN_FINE     1u
#define OBST_SCAN_REFINE_R 2u
#define OBST_SCAN_REFINE_L 3u
#define OBST_SCAN_DONE     4u

typedef struct
{
    uint8_t               phase;
    obstacle_core_point_t coarse[OBST_COARSE_PINGS];
    uint8_t               n_coarse;
    obstacle_core_point_t fine[OBST_SCAN_MAX_POINTS];
    uint8_t               n_fine;
    int16_t               angle; /**< next angle to ping            */
    int16_t               fine_to;
    int16_t               hit;  /**< refine: inner (hit) angle      */
    int16_t               miss; /**< refine: outer (miss) angle     */
    uint16_t              r_min;
    uint8_t               iter;
} obstacle_core_scan_t;

/** Start a servo scan: coarse sweep first. */
void obstacle_core_scan_start(obstacle_core_scan_t * p_s);
/** The servo angle for the next ping, deg. */
int16_t obstacle_core_scan_angle(obstacle_core_scan_t const * p_s);
/** Feed the median range at the current angle; moves the scan on. */
void obstacle_core_scan_feed(obstacle_core_scan_t * p_s, uint16_t r_mm);
/** True once the scan is complete. */
bool obstacle_core_scan_done(obstacle_core_scan_t const * p_s);
/** Profile of the completed scan (obstacle_core_profile()). */
void obstacle_core_scan_profile(obstacle_core_scan_t const * p_s,
                                obstacle_core_profile_t *    p_out);
/** Profile of the completed scan for an obstacle the monitor saw at r_det_mm,
    backed off back_mm since (obstacle_core_profile_detected()). */
void obstacle_core_scan_detected(obstacle_core_scan_t const * p_s,
                                 uint16_t r_det_mm, uint16_t back_mm,
                                 obstacle_core_profile_t * p_out);

/** Bypass path: up to OBST_BYPASS_MAX_STEPS steps.  TURN values are real
   degrees; the task scales them by the measured turn slip. */
typedef uint8_t obstacle_core_step_kind_t; /* one of the constants below */
#define OBST_STEP_MOVE 1u                  /* value = mm (+ forward) */
#define OBST_STEP_TURN 2u                  /* value = 0.1 deg (+ left) */
#define OBST_STEP_PASS 3u /* drive on, servo watching the obstacle */
/* drive on until a line sensor sees the line; value = give up after this, mm */
#define OBST_STEP_JOIN 4u
/* spin slowly until the line is centred under the sensors; value = most 0.1
   deg (+ left) */
#define OBST_STEP_LINESCAN 5u
/* 7 Nov: step out at the turned heading, the servo looking along the line at
   the face, until clear of it (obst_out_*); value = planned leg, mm */
#define OBST_STEP_OUT 6u

typedef struct
{
    uint8_t kind;
    int16_t value;
} obstacle_core_step_t;

#define OBST_BYPASS_MAX_STEPS 10u

/** 23 Nov: how far (mm, 0 = not at all) the car must back up before the
    turn-out so that its inner side passes the obstacle's corner, and its
    front corners the face in the turn, by OBST_MARGIN_MM. */
int32_t obstacle_core_bypass_back_mm(obstacle_core_profile_t const * p_prof);

/** The steps of the bypass on side (OBST_PLAN_LEFT / RIGHT) for this profile
    into p_steps.  Returns how many. */
uint8_t obstacle_core_bypass_plan(obstacle_core_profile_t const * p_prof,
                                  uint8_t side, obstacle_core_step_t * p_steps);

/* ---- Step-out leg (7 Nov).  Lateral values are mm from the line toward
   the bypass side; theta = the car's real heading from the line (0.01
   deg, toward the side, > 0); face_mm = the face's distance ahead of the
   axle where the car stopped (before any back-up). */
typedef struct
{
    int32_t min_lat;  /**< the plan's offset: never step out less  */
    int32_t seen_lat; /**< sensor lateral at the furthest face echo */
    bool    b_seen;
    uint8_t pings;
} obstacle_core_out_t;

/** Start a step-out leg; min_lat = the plan's offset. */
void obstacle_core_out_start(obstacle_core_out_t * p_o, int32_t min_lat);
/** Range at which the face would echo, the sensor looking along the line. */
int32_t obstacle_core_out_expect_mm(int32_t face_mm, int32_t axle_fwd_mm,
                                    int32_t theta_cdeg);
/** One ping: the axle is axle_lat out and axle_fwd on from the stop. */
void obstacle_core_out_ping(obstacle_core_out_t * p_o, int32_t axle_lat,
                            int32_t axle_fwd, int32_t theta_cdeg, uint16_t r_mm,
                            int32_t face_mm);
/** Axle lateral to reach: clear of the furthest face echo by the car's
   half width + margin + allowance, never less than the plan, never more
   than past an OBST_WIDTH_MAX obstacle's furthest possible edge. */
int32_t obstacle_core_out_target(obstacle_core_out_t const * p_o);
/** True when the step-out leg has gone far enough sideways (axle_lat, mm). */
bool obstacle_core_out_done(obstacle_core_out_t const * p_o, int32_t axle_lat);
/** Shortest distance (mm, 0 = touching) from the car's outline, axle at
   (axle_fwd, axle_lat) on heading theta, to the obstacle: from its face
   (face_mm ahead of the stop point) on, from its edge (edge_lat) toward
   the line. */
int32_t obstacle_core_out_clear_mm(int32_t face_mm, int32_t edge_lat,
                                   int32_t axle_fwd_mm, int32_t axle_lat,
                                   int32_t theta_cdeg);
/** The car's front corner is within OBST_MARGIN_MM of the face plane
   (the width probe's limit; the step-out leg uses
   obstacle_core_out_blocked() since 23 Nov). */
bool obstacle_core_face_blocked(int32_t face_mm, int32_t axle_fwd_mm,
                                int32_t theta_cdeg);
/** 23 Nov: the car's outline is within OBST_OUT_BLOCK_MM of the obstacle
   (obstacle_core_out_clear_mm()): the leg must end here, whatever it has
   seen. */
bool obstacle_core_out_blocked(int32_t face_mm, int32_t edge_lat,
                               int32_t axle_fwd_mm, int32_t axle_lat,
                               int32_t theta_cdeg);
/* ---- Width probe (7 Nov): sensor lateral positions (mm, + left) of the
   face echoes while the car side-steps past both edges. */
typedef struct
{
    bool    b_seen;
    int32_t low;  /**< rightmost sensor lateral with a face echo */
    int32_t high; /**< leftmost                                  */
    uint8_t off;  /**< clear pings in a row                      */
} obstacle_core_probe_t;

/** 9 Nov: the face range to approach: r if it is within
   OBST_PROBE_FACE_TOL_MM of where the scan put the face, else that. */
uint16_t obstacle_core_probe_face(uint16_t r_mm, uint16_t expect_mm);
/** Start a width probe: no face seen yet. */
void obstacle_core_probe_start(obstacle_core_probe_t * p_p);
/** Sensor lateral (+ left) when the axle is axle_lat out and the car turned
   theta (0.01 deg, + left) with the servo looking back along the line. */
int32_t obstacle_core_probe_sensor_lat(int32_t axle_lat, int32_t theta_cdeg);
/** One ping; true if it was the face (within OBST_DEPTH_TOL of expect). */
bool obstacle_core_probe_ping(obstacle_core_probe_t * p_p, int32_t sensor_lat,
                              uint16_t r_mm, int32_t expect_mm);
/** Edges with the echo run-on (OBST_PROBE_EDGE_OVER_MM) taken off; false if
   the face was never seen.  Never narrower than nothing. */
bool obstacle_core_probe_edges(obstacle_core_probe_t const * p_p,
                               int32_t * p_left, int32_t * p_right);

/** Servo angle (deg, > 90 left) that looks along the line from a car
   turned theta toward side +1 (left) / -1 (right). */
int32_t obstacle_core_out_servo_deg(int32_t theta_cdeg, int32_t side);

/** Cross-track offset (+ left) of (x, y) from the line through (x0, y0)
   with heading h0, all from the odometry snapshot. */
int32_t obstacle_core_cross_track_mm(int32_t x_mm, int32_t y_mm, int32_t x0_mm,
                                     int32_t y0_mm, int32_t h0_cdeg);

/** SEEK_LINE acceptance (Week 6 6.4): within +-60 mm of the original path
   and +-20 deg of the original heading. */
bool obstacle_core_line_accept(int32_t x_mm, int32_t y_mm, int32_t h_cdeg,
                               int32_t x0_mm, int32_t y0_mm, int32_t h0_cdeg);

/** Hump mask for the 40 / 20 cm thresholds (never for TOO_CLOSE). */
typedef struct
{
    bool     b_active;
    uint32_t start_ms;
    int32_t  start_dist_mm;
} obstacle_core_mask_t;

/** True while the obstacle thresholds are masked because the robot is pitched
    on a hump. */
bool obstacle_core_hump_mask(obstacle_core_mask_t * p_m, bool b_pitch_valid,
                             int32_t pitch_cdeg, int32_t dist_mm,
                             uint32_t now_ms);

/* Change Requirement 1: the obstacle snapshot as stored between tasks. */
#define OBST_SNAP_BITS  166u /* sum of the layout widths      */
#define OBST_SNAP_WORDS 6u   /* 24 B instead of 28 B          */
/** Pack the obstacle snapshot into OBST_SNAP_WORDS words. */
void obstacle_core_snap_pack(mission_types_obst_snap_t const * p_snap,
                             uint32_t *                        p_words);
/** Unpack OBST_SNAP_WORDS words into the obstacle snapshot. */
void obstacle_core_snap_unpack(uint32_t const *            p_words,
                               mission_types_obst_snap_t * p_snap);
/** Bits used by the packed obstacle snapshot. */
uint32_t obstacle_core_snap_layout_bits(void);

#endif /* OBSTACLE_CORE_H */

/*** end of file ***/
