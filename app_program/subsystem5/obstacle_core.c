/** @file obstacle_core.c
 *
 *  @brief Subsystem 5 -- obstacle geometry (pure).  See obstacle_core.h.
 */

#include "obstacle_core.h"
#include <stddef.h>
#include "../app_pack.h"
#include "mission_cfg.h"
#include "../app_types.h"
#include "../app_math.h"

/* other objects this close matter for clearance */
#define OBST_NEAR_EXTRA_MM 300u

/* NOTE: OBST_DEPTH_TOL_MM and OBST_WIDTH_MAX_MM are positive int constants
   (mission_cfg.h) also used in signed lateral arithmetic; these unsigned
   copies, for comparisons with ranges, keep their values. */
#define DEPTH_TOL_MM ((uint32_t)OBST_DEPTH_TOL_MM)
#define WIDTH_MAX_MM ((uint32_t)OBST_WIDTH_MAX_MM)

/* ---- step-out leg (7 Nov) -------------------------------------------- */
#define OUT_EXTRA_MM \
    (OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM)

/* servo shaft ahead of the axle */
#define OUT_SHAFT_MM (OBST_US_AHEAD_MM - OBST_SERVO_ARM_MM)

/* Cast (uint16_t)(sizeof ...): the layout table has fewer than 64 entries. */
#define OBST_FIELDS \
    ((uint16_t)(sizeof(g_obst_layout) / sizeof(g_obst_layout[0])))

/* ---------------------------------------------------------------------
 *  Change Requirement 1: data representation of IF-10 (obstacle part).
 * --------------------------------------------------------------------- */
static app_pack_field_t const g_obst_layout[] = {
    {offsetof(mission_types_obst_snap_t, t_ms),
     sizeof(((mission_types_obst_snap_t){0}).t_ms), 32u, PK_W},
    {offsetof(mission_types_obst_snap_t, pings),
     sizeof(((mission_types_obst_snap_t){0}).pings), 32u,
     PK_W}, /* counter since boot */
    {offsetof(mission_types_obst_snap_t, d_mm),
     sizeof(((mission_types_obst_snap_t){0}).d_mm), 12u,
     PK_UN}, /* 0..4094 mm, NONE (0xFFFF) */
    {offsetof(mission_types_obst_snap_t, w_mm),
     sizeof(((mission_types_obst_snap_t){0}).w_mm), 11u, PK_U}, /* 0..2047 mm */
    {offsetof(mission_types_obst_snap_t, closest_mm),
     sizeof(((mission_types_obst_snap_t){0}).closest_mm), 12u, PK_UN},
    {offsetof(mission_types_obst_snap_t, clr_l_mm),
     sizeof(((mission_types_obst_snap_t){0}).clr_l_mm), 12u, PK_UN},
    {offsetof(mission_types_obst_snap_t, clr_r_mm),
     sizeof(((mission_types_obst_snap_t){0}).clr_r_mm), 12u, PK_UN},
    {offsetof(mission_types_obst_snap_t, left_lat_mm),
     sizeof(((mission_types_obst_snap_t){0}).left_lat_mm), 12u,
     PK_S}, /* +-2047 mm */
    {offsetof(mission_types_obst_snap_t, right_lat_mm),
     sizeof(((mission_types_obst_snap_t){0}).right_lat_mm), 12u, PK_S},
    {offsetof(mission_types_obst_snap_t, no_echo),
     sizeof(((mission_types_obst_snap_t){0}).no_echo), 8u,
     PK_U}, /* consecutive, saturates at 255   */
    {offsetof(mission_types_obst_snap_t, valid),
     sizeof(((mission_types_obst_snap_t){0}).valid), 1u, PK_U},
    {offsetof(mission_types_obst_snap_t, plan),
     sizeof(((mission_types_obst_snap_t){0}).plan), 3u, PK_U}, /* 5 plans */
    {offsetof(mission_types_obst_snap_t, state),
     sizeof(((mission_types_obst_snap_t){0}).state), 4u, PK_U}, /* 13 states */
    {offsetof(mission_types_obst_snap_t, retries),
     sizeof(((mission_types_obst_snap_t){0}).retries), 3u, PK_U}};

/** The obstacle = the run of points around the nearest one at its range.
   Consecutive-angle rule (hysteresis): a single point that misses between
   two that hit is a dropout, not an edge; an edge needs 2 misses. */
static int32_t lateral_mm(int32_t angle_cdeg, uint16_t r_mm);
static void    decide(obstacle_core_point_t const * p_fine, uint8_t n_fine,
                      obstacle_core_point_t const * p_coarse, uint8_t n_coarse,
                      uint8_t low, uint8_t high, uint16_t r_min, int32_t pl_l,
                      int32_t pl_r, obstacle_core_profile_t * p_out);
static int32_t lateral2_mm(int32_t servo_cdeg, int32_t beam_cdeg,
                           uint16_t r_mm);
static bool    usable(obstacle_core_point_t const * p_point);
static bool same_object(obstacle_core_point_t const * p_point, uint16_t r_min);
static bool in_group(obstacle_core_point_t const * p_point, uint16_t r_min,
                     int32_t lat0);
static void grow(obstacle_core_point_t const * p_pts, uint8_t count,
                 uint8_t nearest, uint16_t r_min, uint8_t * p_lo,
                 uint8_t * p_hi);
static void profile_core(obstacle_core_point_t const * p_fine, uint8_t n_fine,
                         obstacle_core_point_t const * p_coarse,
                         uint8_t n_coarse, obstacle_core_profile_t * p_out);
static void force_ahead(obstacle_core_point_t const * p_fine, uint8_t n_fine,
                        obstacle_core_point_t const * p_coarse,
                        uint8_t n_coarse, uint16_t range,
                        obstacle_core_profile_t * p_out);
static bool is_hit(obstacle_core_point_t const * p_point, uint16_t r_min);
static void coarse_done(obstacle_core_scan_t * p_s);
static void refine_step(obstacle_core_scan_t * p_s, uint16_t r_mm);
static void edge_plan_apply(obstacle_core_profile_t * p_prof, bool b_has_l,
                            int32_t lat_l, bool b_has_r, int32_t lat_r);
static void fine_insert(obstacle_core_scan_t * p_s, int16_t angle,
                        uint16_t r_mm);
static void start_refine(obstacle_core_scan_t * p_s, bool b_right);
static int32_t  width_lat(int32_t phi_cdeg, uint16_t r_mm, int32_t side);
static void     profile_clear(obstacle_core_profile_t * p_out);
static uint16_t nearest_echo(obstacle_core_point_t const * p_fine,
                             uint8_t n_fine, uint8_t * p_nearest);
static void profile_measure(obstacle_core_point_t const * p_fine,
                            uint8_t                       n_fine,
                            obstacle_core_point_t const * p_coarse,
                            uint8_t n_coarse, uint8_t nearest, uint16_t r_min,
                            obstacle_core_profile_t * p_out);

uint16_t obstacle_core_median3 (uint16_t val_a, uint16_t val_b, uint16_t val_c)
{
    uint16_t low    = val_a;
    uint16_t high   = val_b;
    uint16_t median = val_c;

    if (low > high)
    {
        low  = val_b;
        high = val_a;
    }

    if (val_c <= low)
    {
        median = low;
    }
    else if (val_c >= high)
    {
        median = high;
    }
    else
    {
        /* val_c is the median */
    }

    return median;
}

uint16_t obstacle_core_echo_to_mm (uint32_t width_us)
{
    uint16_t r_mm = OBST_RANGE_NONE;

    if ((width_us != 0u) && (width_us < OBST_ECHO_MAX_US))
    {
        /* Cast (uint16_t)((width_us * 10u) / 58u): width_us < OBST_ECHO_MAX_US
           (tested above), so the range is below 4000 mm. */
        r_mm = (uint16_t)((width_us * 10u) / 58u); /* us / 58 = cm */
    }

    return r_mm;
}

void obstacle_core_profile (obstacle_core_point_t const * p_fine,
                            uint8_t                       n_fine,
                            obstacle_core_point_t const * p_coarse,
                            uint8_t n_coarse, obstacle_core_profile_t * p_out)
{
    uint16_t r_ahead = OBST_RANGE_NONE;
    uint8_t  idx;

    profile_core(p_fine, n_fine, p_coarse, n_coarse, p_out);

    /* Guard (3 Nov: twice the profile said CONTINUE, or found nothing at
       all, with the box 15-18 cm straight ahead, and the car drove into
       it): if any echo within OBST_GUARD_HALF_DEG of straight ahead is
       closer than OBST_GUARD_MM, raw (no background, no range filter),
       the plan is never CONTINUE: plan as for an OBST_WIDTH_MAX object
       centred straight ahead at that range. */
    /* Cast (uint8_t)(n_fine + n_coarse): at most OBST_SCAN_MAX_POINTS +
       OBST_COARSE_PINGS (37) points. */
    for (idx = 0u; idx < (uint8_t)(n_fine + n_coarse); idx++)
    {
        obstacle_core_point_t const * p_point =
            (idx < n_fine) ? &p_fine[idx] : &p_coarse[idx - n_fine];

        /* Cast (int32_t)p_point->angle_deg: every int16_t value fits. */
        if ((app_types_abs_i32((int32_t)p_point->angle_deg
                               - OBST_SERVO_CENTRE_DEG)
             <= OBST_GUARD_HALF_DEG)
            && (p_point->r_mm < r_ahead))
        {
            r_ahead = p_point->r_mm;
        }
    }

    if ((r_ahead < OBST_GUARD_MM) && (OBST_PLAN_CONTINUE == p_out->plan))
    {
        force_ahead(p_fine, n_fine, p_coarse, n_coarse, r_ahead, p_out);
    }
}

void obstacle_core_profile_detected (obstacle_core_point_t const * p_fine,
                                     uint8_t                       n_fine,
                                     obstacle_core_point_t const * p_coarse,
                                     uint8_t n_coarse, uint16_t r_det_mm,
                                     uint16_t                  back_mm,
                                     obstacle_core_profile_t * p_out)
{
    obstacle_core_profile(p_fine, n_fine, p_coarse, n_coarse, p_out);

    /* 3 Nov: a round water bottle was seen while driving (16 cm, median
       of 3) but not in the scan (curved glass returns few echoes), the
       profile was empty, the plan CONTINUE, and the car drove into it.
       Nothing seen in the scan after a detection = trust the detection. */
    /* 4 Nov: the test is on the range when it was seen, not after the
       back-off (19.2 cm + 6 cm passed the 25 cm limit and the car drove on) */
    if (!p_out->b_valid && (r_det_mm < OBST_GUARD_MM))
    {
        /* Cast (uint16_t)(r_det_mm + back_mm): a detection range (< 4000 mm)
           plus a back-off clamped to 1000 mm. */
        force_ahead(p_fine, n_fine, p_coarse, n_coarse,
                    (uint16_t)(r_det_mm + back_mm), p_out);
    }
}

void obstacle_core_scan_start (obstacle_core_scan_t * p_s)
{
    p_s->phase    = OBST_SCAN_COARSE;
    p_s->n_coarse = 0u;
    p_s->n_fine   = 0u;
    p_s->angle    = 30;
    p_s->iter     = 0u;
    p_s->r_min    = OBST_RANGE_NONE;
}

int16_t obstacle_core_scan_angle (obstacle_core_scan_t const * p_s)
{
    return p_s->angle;
}

bool obstacle_core_scan_done (obstacle_core_scan_t const * p_s)
{
    return (OBST_SCAN_DONE == p_s->phase);
}

void obstacle_core_scan_feed (obstacle_core_scan_t * p_s, uint16_t r_mm)
{
    switch (p_s->phase)
    {
        case OBST_SCAN_COARSE:
            p_s->coarse[p_s->n_coarse].angle_deg = p_s->angle;
            p_s->coarse[p_s->n_coarse].r_mm      = r_mm;
            p_s->n_coarse++;

            if (p_s->n_coarse < 5u)
            {
                /* Cast (int16_t)(p_s->angle + 30): the coarse sweep stays
                   within 30..150 deg. */
                p_s->angle = (int16_t)(p_s->angle + 30);
            }
            else
            {
                coarse_done(p_s);
            }
        break;

        case OBST_SCAN_FINE:
            fine_insert(p_s, p_s->angle, r_mm);

            if ((p_s->angle + OBST_FINE_STEP_DEG) <= p_s->fine_to)
            {
                /* Cast (int16_t)(p_s->angle + ...): the fine sweep stays within
                   the servo's 0..180 deg. */
                p_s->angle = (int16_t)(p_s->angle + OBST_FINE_STEP_DEG);
            }
            else
            {
                start_refine(p_s, true);
            }
        break;

        case OBST_SCAN_REFINE_R:
        case OBST_SCAN_REFINE_L:
            refine_step(p_s, r_mm);
        break;

        default:
        break;
    }
}

void obstacle_core_scan_profile (obstacle_core_scan_t const * p_s,
                                 obstacle_core_profile_t *    p_out)
{
    obstacle_core_profile(p_s->fine, p_s->n_fine, p_s->coarse, p_s->n_coarse,
                          p_out);
}

void obstacle_core_scan_detected (obstacle_core_scan_t const * p_s,
                                  uint16_t r_det_mm, uint16_t back_mm,
                                  obstacle_core_profile_t * p_out)
{
    obstacle_core_profile_detected(p_s->fine, p_s->n_fine, p_s->coarse,
                                   p_s->n_coarse, r_det_mm, back_mm, p_out);
}

/* ---- edge scan (4 Nov) ------------------------------------------------ */
bool obstacle_core_edge_on_face (int32_t phi_cdeg, uint16_t r_mm,
                                 uint16_t face_mm)
{
    /* forward distance of the echo from the axle vs the face plane */
    /* Casts (int32_t)((((int64_t)OBST_US_AHEAD_ ...))): |sin|, |cos| <= 1.0 in
       Q15, so the result is no larger than the int32_t factor;
       (int64_t)OBST_US_AHEAD_MM: a constant that fits; (int64_t)r_mm: every
       uint16_t value fits. */
    int32_t const fwd = (int32_t)((((int64_t)OBST_US_AHEAD_MM + (int64_t)r_mm)
                                   * app_math_cos_q15(phi_cdeg))
                                  / 32768);

    /* Casts (int32_t)OBST_US_AHEAD_MM: a constant that fits; (int32_t)face_mm:
       every uint16_t value fits. */
    return (r_mm != OBST_RANGE_NONE) && (r_mm <= OBST_MAX_RANGE_MM)
           && (fwd <= ((int32_t)OBST_US_AHEAD_MM + (int32_t)face_mm
                       + OBST_EDGE_BAND_MM));
}

int32_t obstacle_core_edge_lat (int32_t phi_cdeg)
{
    /* Casts (int32_t)(((int64_t)OBST_US_AHEAD_M ...)): the value is
       -5570560..5570559; (int64_t)OBST_US_AHEAD_MM: a constant that fits. */
    return (int32_t)(((int64_t)OBST_US_AHEAD_MM * app_math_sin_q15(phi_cdeg))
                     / 32768);
}

int32_t obstacle_core_edge_wide_lat (int32_t phi_cdeg, uint16_t r_mm,
                                     int32_t side)
{
    int32_t const psi = phi_cdeg - (side * OBST_HALF_BEAM_CDEG);

    /* Casts (int32_t)((((int64_t)OBST_US_AHEAD_ ...))): |sin|, |cos| <= 1.0 in
       Q15, so the result is no larger than the int32_t factor;
       (int64_t)OBST_US_AHEAD_MM: a constant that fits; (int64_t)r_mm: every
       uint16_t value fits. */
    return (int32_t)((((int64_t)OBST_US_AHEAD_MM * app_math_sin_q15(phi_cdeg))
                      + ((int64_t)r_mm * app_math_sin_q15(psi)))
                     / 32768);
}

uint16_t obstacle_core_edge_width (int32_t phi_l_cdeg, uint16_t r_l_mm,
                                   int32_t phi_r_cdeg, uint16_t r_r_mm)
{
    /* Cast (uint16_t)app_types_clamp_i32(...): the value is 0..65535. */
    return (uint16_t)app_types_clamp_i32(
        width_lat(phi_l_cdeg, r_l_mm, 1) - width_lat(phi_r_cdeg, r_r_mm, -1), 0,
        0xFFFF);
}

bool obstacle_core_edge_past_max (int32_t lat_mm, int32_t side)
{
    return ((side * lat_mm) > OBST_WIDTH_MAX_MM);
}

void obstacle_core_edge_plan (obstacle_core_profile_t * p_prof, bool b_has_l,
                              int32_t lat_l, bool b_has_r, int32_t lat_r)
{
    /* No echo on either side and no servo scan: nothing to plan from. */
    if (b_has_l || b_has_r || p_prof->b_valid)
    {
        edge_plan_apply(p_prof, b_has_l, lat_l, b_has_r, lat_r);
    }
}

void obstacle_core_edge_plan_min (obstacle_core_profile_t * p_prof,
                                  bool b_has_l, int32_t min_l, bool b_has_r,
                                  int32_t min_r)
{
    int32_t const extra =
        OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM;

    if ((OBST_PLAN_LEFT == p_prof->plan) && b_has_l)
    {
        /* Cast (int16_t)app_types_max_i32(min_l + ...): edge offsets (+-800 mm)
           plus a margin of a few hundred mm. */
        p_prof->deviation_mm = (int16_t)app_types_max_i32(min_l + extra, extra);
    }
    else if ((OBST_PLAN_RIGHT == p_prof->plan) && b_has_r)
    {
        /* Cast (int16_t)-app_types_max_i32(-min_r ...): edge offsets (+-800 mm)
           plus a margin of a few hundred mm. */
        p_prof->deviation_mm =
            (int16_t)-app_types_max_i32(-min_r + extra, extra);
    }
    else
    {
        /* unseen side or no bypass: the plan's offset stands */
    }
}

/* ---- bypass ----------------------------------------------------------- */
uint8_t obstacle_core_bypass_plan (obstacle_core_profile_t const * p_prof,
                                   uint8_t side, obstacle_core_step_t * p_steps)
{
    /* Casts (int32_t)(((int64_t)dev * 32768) / ...): dev is below 1 m and sin_q
       is at least sin(45 deg); (int64_t)dev, (int64_t)diag: every int32_t value
       fits; (int32_t)(((int64_t)diag * cos_q) ...): cos_q <= 1.0 in Q15, so the
       result is no larger than diag; (int32_t)p_prof->closest_mm: every
       uint16_t value fits. */
    int32_t const sign  = (OBST_PLAN_LEFT == side) ? 1 : -1;
    int32_t const dev   = app_types_abs_i32(p_prof->deviation_mm);
    int32_t const sin_q = app_math_sin_q15(OBST_TURN_OUT_DDEG * 10);
    int32_t const cos_q = app_math_cos_q15(OBST_TURN_OUT_DDEG * 10);
    int32_t const diag  = (int32_t)(((int64_t)dev * 32768) / sin_q);
    int32_t const fwd   = (int32_t)(((int64_t)diag * cos_q) / 32768);
    int32_t const room  = (int32_t)p_prof->closest_mm - OBST_MARGIN_MM;
    /* 7 Nov: the step-out leg may go OBST_OUT_SPARE further out */
    /* Casts (int32_t)(((int64_t)(dev + ...))): dev is below 1 m and sin_q is at
       least sin(45 deg); (int64_t)(dev + OBST_OUT_SPARE_MM): every int value
       fits. */
    int32_t const fwd_r =
        (int32_t)(((int64_t)(dev + OBST_OUT_SPARE_MM) * cos_q) / sin_q);
    int32_t pass;
    int32_t back = 0;
    int32_t depth;
    uint8_t n_steps = 0u;

    /* Not enough room to get sideways before reaching the face: back up. */
    if (fwd_r > room)
    {
        /* Cast (int16_t)-back: back is a back-off of at most 1000 mm. */
        back                   = (fwd_r - room) + 20;
        p_steps[n_steps].kind  = OBST_STEP_MOVE;
        p_steps[n_steps].value = (int16_t)-back;
        n_steps++;
    }

    /* Casts (int16_t)(sign * ...), (int16_t)(-sign * ...): +-450 (0.1 deg);
       (int16_t)diag: the step-out leg is a few hundred mm (dev < 1 m). */
    p_steps[n_steps].kind  = OBST_STEP_TURN;
    p_steps[n_steps].value = (int16_t)(sign * OBST_TURN_OUT_DDEG);
    n_steps++;
    p_steps[n_steps].kind  = OBST_STEP_OUT;
    p_steps[n_steps].value = (int16_t)diag;
    n_steps++;
    p_steps[n_steps].kind  = OBST_STEP_TURN;
    p_steps[n_steps].value = (int16_t)(-sign * OBST_TURN_OUT_DDEG);
    n_steps++;

    /* PASS: the task ends it when the side ping has seen the obstacle and
       lost it again; 3 Nov: this value is now the MINIMUM (face to the
       far side of a 20 cm deep object, plus the car, plus the margin):
       the side ping lost a flat side early and the car turned in beside
       the box.  OBST_ROBOT_LENGTH_MM is measured from the sensor, so the
       sensor's OBST_US_AHEAD_MM in front of the turning axle cancels out
       (face at US_AHEAD + closest from the axle, rear at LENGTH - US_AHEAD
       behind it). */
    /* depth (4 Nov): taken as the measured width (square), at most
       OBST_WIDTH_MAX; OBST_WIDTH_MAX when the width is not known */
    /* Casts (int32_t)p_prof->closest_mm: every uint16_t value fits;
       (int16_t)app_types_min_i32(pass ...): limited to OBST_PASS_MAX_MM (800);
       pass is a positive length. */
    depth = ((p_prof->w_mm > 0u) && (p_prof->w_mm <= WIDTH_MAX_MM))
                ? p_prof->w_mm
                : OBST_WIDTH_MAX_MM;
    pass  = app_types_max_i32(((int32_t)p_prof->closest_mm + back) - fwd, 0)
            + depth + OBST_ROBOT_LENGTH_MM + OBST_MARGIN_MM;
    p_steps[n_steps].kind  = OBST_STEP_PASS;
    p_steps[n_steps].value = (int16_t)app_types_min_i32(pass, OBST_PASS_MAX_MM);
    n_steps++;

    /* 3 Nov: back in at the same angle until a line sensor touches the
       line (not a fixed distance: the turns slip), then OBST_IR_AHEAD_MM
       on so the axle is on the line, then (4 Nov) spin until the line is
       centred under the sensors, wherever it runs. */
    /* Casts (int16_t)(-sign * ...): +-450 (0.1 deg); (int16_t)(((diag * ...))):
       1.5 x a step-out leg of a few hundred mm, + 100 mm;
       (int16_t)(sign * ...): +-900 (0.1 deg). */
    p_steps[n_steps].kind  = OBST_STEP_TURN;
    p_steps[n_steps].value = (int16_t)(-sign * OBST_TURN_OUT_DDEG);
    n_steps++;
    p_steps[n_steps].kind  = OBST_STEP_JOIN;
    p_steps[n_steps].value = (int16_t)(((diag * OBST_JOIN_MAX_PERMILLE) / 1000)
                                       + OBST_JOIN_EXTRA_MM);
    n_steps++;
    p_steps[n_steps].kind  = OBST_STEP_MOVE;
    p_steps[n_steps].value = OBST_IR_AHEAD_MM;
    n_steps++;
    p_steps[n_steps].kind = OBST_STEP_LINESCAN;
    p_steps[n_steps].value =
        (int16_t)(sign * (OBST_TURN_OUT_DDEG + OBST_LINESCAN_SPARE_DDEG));
    n_steps++;
    return n_steps;
}

void obstacle_core_out_start (obstacle_core_out_t * p_o, int32_t min_lat)
{
    p_o->min_lat  = min_lat;
    p_o->seen_lat = 0;
    p_o->b_seen   = false;
    p_o->pings    = 0u;
}

int32_t obstacle_core_out_expect_mm (int32_t face_mm, int32_t axle_fwd_mm,
                                     int32_t theta_cdeg)
{
    /* the sensor: the shaft OUT_SHAFT ahead of the axle along the car,
       then the arm along the line */
    /* Casts (int32_t)(((int64_t)OUT_SHAFT_MM * ...)): the value is
       -3670016..3670015; (int64_t)OUT_SHAFT_MM: a constant that fits. */
    int32_t const sensor_fwd =
        axle_fwd_mm
        + (int32_t)(((int64_t)OUT_SHAFT_MM * app_math_cos_q15(theta_cdeg))
                    / 32768)
        + OBST_SERVO_ARM_MM;

    return face_mm - sensor_fwd;
}

void obstacle_core_out_ping (obstacle_core_out_t * p_o, int32_t axle_lat,
                             int32_t axle_fwd, int32_t theta_cdeg,
                             uint16_t r_mm, int32_t face_mm)
{
    int32_t const expect =
        obstacle_core_out_expect_mm(face_mm, axle_fwd, theta_cdeg);

    /* Casts (uint8_t)app_types_min_i32(...): the value is 1..255;
       (int32_t)p_o->pings: every uint8_t value fits. */
    p_o->pings = (uint8_t)app_types_min_i32((int32_t)p_o->pings + 1, 255);

    /* Cast (int32_t)r_mm: every uint16_t value fits. */
    if ((r_mm != OBST_RANGE_NONE)
        && ((int32_t)r_mm <= (expect + OBST_DEPTH_TOL_MM)))
    {
        /* Casts (int32_t)(((int64_t)OUT_SHAFT_MM * ...)): the value is
           -3670016..3670015; (int64_t)OUT_SHAFT_MM: a constant that fits. */
        int32_t const lat =
            axle_lat
            + (int32_t)(((int64_t)OUT_SHAFT_MM * app_math_sin_q15(theta_cdeg))
                        / 32768);

        if (!p_o->b_seen || (lat > p_o->seen_lat))
        {
            p_o->seen_lat = lat;
        }

        p_o->b_seen = true;
    }
}

int32_t obstacle_core_out_target (obstacle_core_out_t const * p_o)
{
    int32_t target = p_o->min_lat;

    if (p_o->b_seen)
    {
        target = app_types_max_i32(target, p_o->seen_lat + OUT_EXTRA_MM);
    }

    return app_types_min_i32(target, OBST_WIDTH_MAX_MM + OUT_EXTRA_MM);
}

bool obstacle_core_out_done (obstacle_core_out_t const * p_o, int32_t axle_lat)
{
    return ((p_o->pings >= OBST_OUT_MIN_PINGS)
            && (axle_lat >= obstacle_core_out_target(p_o)))
           || (axle_lat >= (OBST_WIDTH_MAX_MM + OUT_EXTRA_MM));
}

bool obstacle_core_out_blocked (int32_t face_mm, int32_t axle_fwd_mm,
                                int32_t theta_cdeg)
{
    /* the front corner on the line's side sits half a width further on */
    /* Casts (int32_t)(((int64_t) ...)): the value is -4259840..4259839;
       (int64_t)(OBST_ROBOT_WIDTH_MM / 2): a constant that fits. */
    int32_t const corner = (int32_t)(((int64_t)(OBST_ROBOT_WIDTH_MM / 2)
                                      * app_math_sin_q15(theta_cdeg))
                                     / 32768);

    return obstacle_core_out_expect_mm(face_mm, axle_fwd_mm, theta_cdeg)
           <= (OBST_MARGIN_MM + corner);
}

uint16_t obstacle_core_probe_face (uint16_t r_mm, uint16_t expect_mm)
{
    /* Casts (int32_t)r_mm, (int32_t)expect_mm: every uint16_t value fits. */
    return ((OBST_RANGE_NONE == r_mm)
            || (app_types_abs_i32((int32_t)r_mm - (int32_t)expect_mm)
                > OBST_PROBE_FACE_TOL_MM))
               ? expect_mm
               : r_mm;
}

void obstacle_core_probe_start (obstacle_core_probe_t * p_p)
{
    p_p->b_seen = false;
    p_p->low    = 0;
    p_p->high   = 0;
    p_p->off    = 0u;
}

int32_t obstacle_core_probe_sensor_lat (int32_t axle_lat, int32_t theta_cdeg)
{
    /* Casts (int32_t)(((int64_t)OUT_SHAFT_MM * ...)): the value is
       -3670016..3670015; (int64_t)OUT_SHAFT_MM: a constant that fits. */
    return axle_lat
           + (int32_t)(((int64_t)OUT_SHAFT_MM * app_math_sin_q15(theta_cdeg))
                       / 32768);
}

bool obstacle_core_probe_ping (obstacle_core_probe_t * p_p, int32_t sensor_lat,
                               uint16_t r_mm, int32_t expect_mm)
{
    /* Cast (int32_t)r_mm: every uint16_t value fits. */
    bool const b_face = (r_mm != OBST_RANGE_NONE)
                        && ((int32_t)r_mm <= (expect_mm + OBST_DEPTH_TOL_MM));

    if (b_face)
    {
        if (!p_p->b_seen)
        {
            p_p->low  = sensor_lat;
            p_p->high = sensor_lat;
        }

        p_p->low    = app_types_min_i32(p_p->low, sensor_lat);
        p_p->high   = app_types_max_i32(p_p->high, sensor_lat);
        p_p->b_seen = true;
        p_p->off    = 0u;
    }
    else
    {
        /* Casts (uint8_t)app_types_min_i32(...): the value is 1..255;
           (int32_t)p_p->off: every uint8_t value fits. */
        p_p->off = (uint8_t)app_types_min_i32((int32_t)p_p->off + 1, 255);
    }

    return b_face;
}

bool obstacle_core_probe_edges (obstacle_core_probe_t const * p_p,
                                int32_t * p_left, int32_t * p_right)
{
    int32_t left;
    int32_t right;

    if (p_p->b_seen)
    {
        left  = p_p->high - OBST_PROBE_EDGE_OVER_MM;
        right = p_p->low + OBST_PROBE_EDGE_OVER_MM;

        if (left < right)
        {
            left  = (p_p->high + p_p->low)
                    / 2; /* narrower than the run-on: a post */
            right = left;
        }

        *p_left  = left;
        *p_right = right;
    }

    return p_p->b_seen;
}

int32_t obstacle_core_out_servo_deg (int32_t theta_cdeg, int32_t side)
{
    return app_types_clamp_i32(OBST_SERVO_CENTRE_DEG
                                   - ((side * theta_cdeg) / 100),
                               OBST_SCAN_MIN_DEG, OBST_SCAN_MAX_DEG);
}

int32_t obstacle_core_cross_track_mm (int32_t x_mm, int32_t y_mm, int32_t x0_mm,
                                      int32_t y0_mm, int32_t h0_cdeg)
{
    /* Casts (int64_t)x_mm, (int64_t)y_mm: every int32_t value fits. */
    int64_t const delta_x = (int64_t)x_mm - x0_mm;
    int64_t const delta_y = (int64_t)y_mm - y0_mm;

    int64_t const cross = (-delta_x * app_math_sin_q15(h0_cdeg))
                          + (delta_y * app_math_cos_q15(h0_cdeg));

    /* Cast (int32_t)((cross + ((cross >= 0) ? ...))): positions are odometry in
       mm (|x|, |y| < 2^20), so the cross product / 32768 fits. */
    return (int32_t)((cross + ((cross >= 0) ? 16384 : -16384))
                     / 32768); /* rounded */
}

bool obstacle_core_line_accept (int32_t x_mm, int32_t y_mm, int32_t h_cdeg,
                                int32_t x0_mm, int32_t y0_mm, int32_t h0_cdeg)
{
    int32_t const cross_mm =
        obstacle_core_cross_track_mm(x_mm, y_mm, x0_mm, y0_mm, h0_cdeg);
    int32_t const dh_cdeg = app_math_wrap_cdeg(h_cdeg - h0_cdeg);

    return (app_types_abs_i32(cross_mm) <= OBST_SEEK_LAT_MM)
           && (app_types_abs_i32(dh_cdeg) <= OBST_SEEK_HEADING_CDEG);
}

bool obstacle_core_hump_mask (obstacle_core_mask_t * p_m, bool b_pitch_valid,
                              int32_t pitch_cdeg, int32_t dist_mm,
                              uint32_t now_ms)
{
    bool const b_pitched =
        b_pitch_valid && (app_types_abs_i32(pitch_cdeg) > OBST_HUMP_MASK_CDEG);

    if (!b_pitched)
    {
        p_m->b_active = false;
        p_m->start_ms = 0u; /* re-arm for the next hump */
    }
    else
    {
        if (!p_m->b_active && (0u == p_m->start_ms))
        {
            p_m->b_active      = true;
            p_m->start_ms      = (0u == now_ms) ? 1u : now_ms;
            p_m->start_dist_mm = dist_mm;
        }

        if (p_m->b_active
            && ((app_types_elapsed_ms(now_ms, p_m->start_ms)
                 >= OBST_HUMP_MASK_MS)
                || (app_types_abs_i32(dist_mm - p_m->start_dist_mm)
                    >= OBST_HUMP_MASK_MM)))
        {
            p_m->b_active = false; /* lapsed: stays off until level again */
        }
    }

    return p_m->b_active;
}

void obstacle_core_snap_pack (mission_types_obst_snap_t const * p_snap,
                              uint32_t *                        p_words)
{
    app_pack_pack_rec(g_obst_layout, OBST_FIELDS, p_snap, p_words,
                      OBST_SNAP_WORDS);
}

void obstacle_core_snap_unpack (uint32_t const *            p_words,
                                mission_types_obst_snap_t * p_snap)
{
    app_pack_unpack_rec(g_obst_layout, OBST_FIELDS, p_words, p_snap);
}

uint32_t obstacle_core_snap_layout_bits (void)
{
    return app_pack_layout_bits(g_obst_layout, OBST_FIELDS);
}

/** Lateral (+ left) of a servo-scan echo from the servo shaft (7 Nov): the
   sensor sits OBST_SERVO_ARM_MM out along the horn at servo angle
   servo_cdeg, and the echo is r_mm from it along beam_cdeg (the servo angle,
   or an edge corrected by half a beam). */
static int32_t lateral2_mm (int32_t servo_cdeg, int32_t beam_cdeg,
                            uint16_t r_mm)
{
    /* Casts (int64_t)r_mm: every uint16_t value fits;
       (int64_t)OBST_SERVO_ARM_MM: a constant that fits. */
    int64_t const proj =
        ((int64_t)r_mm * app_math_cos_q15(beam_cdeg))
        + ((int64_t)OBST_SERVO_ARM_MM * app_math_cos_q15(servo_cdeg));

    /* Cast (int32_t)(proj / 32768): a projection of a distance in mm with a Q15
       factor (<= 1.0). */
    return -(int32_t)(proj / 32768);
}

static int32_t lateral_mm (int32_t angle_cdeg, uint16_t r_mm)
{
    return lateral2_mm(angle_cdeg, angle_cdeg, r_mm);
}

/** An echo that can be part of an obstacle: in range. */
static bool usable (obstacle_core_point_t const * p_point)
{
    return (p_point->r_mm != OBST_RANGE_NONE)
           && (p_point->r_mm <= OBST_MAX_RANGE_MM);
}

/** True if a usable point is within DEPTH_TOL_MM behind the nearest echo
    r_min. */
static bool same_object (obstacle_core_point_t const * p_point, uint16_t r_min)
{
    /* Cast (uint32_t)r_min: every uint16_t value fits. */
    return usable(p_point)
           && (p_point->r_mm <= ((uint32_t)r_min + DEPTH_TOL_MM));
}

/** Part of the obstacle grown from the nearest echo: at its range and
   (9 Nov) within OBST_GROUP_MAX_MM sideways of it. */
static bool in_group (obstacle_core_point_t const * p_point, uint16_t r_min,
                      int32_t lat0)
{
    /* Cast (int32_t)p_point->angle_deg: every int16_t value fits. */
    return same_object(p_point, r_min)
           && (app_types_abs_i32(
                   lateral_mm((int32_t)p_point->angle_deg * 100, p_point->r_mm)
                   - lat0)
               <= OBST_GROUP_MAX_MM);
}

/** Grow the obstacle from its nearest echo outwards to the first points that
    are not part of it; *p_lo / *p_hi = its first and last point. */
static void grow (obstacle_core_point_t const * p_pts, uint8_t count,
                  uint8_t nearest, uint16_t r_min, uint8_t * p_lo,
                  uint8_t * p_hi)
{
    /* Cast (int32_t)p_pts[nearest].angle_deg: every int16_t value fits. */
    int32_t const lat0 = lateral_mm((int32_t)p_pts[nearest].angle_deg * 100,
                                    p_pts[nearest].r_mm);
    uint8_t       low  = nearest;
    uint8_t       high = nearest;

    for (;;)
    {
        if ((low > 0u) && in_group(&p_pts[low - 1u], r_min, lat0))
        {
            low--;
        }
        else if ((low > 1u) && in_group(&p_pts[low - 2u], r_min, lat0))
        {
            /* Cast (uint8_t)(low - 2u): low > 1 (tested above), so an index
               0..35. */
            low = (uint8_t)(low - 2u);
        }
        else
        {
            break;
        }
    }

    for (;;)
    {
        if (((high + 1u) < count) && in_group(&p_pts[high + 1u], r_min, lat0))
        {
            high++;
        }
        else if (((high + 2u) < count)
                 && in_group(&p_pts[high + 2u], r_min, lat0))
        {
            /* Cast (uint8_t)(high + 2u): high + 2 < count <= 37 (tested
               above). */
            high = (uint8_t)(high + 2u);
        }
        else
        {
            break;
        }
    }

    *p_lo = low;
    *p_hi = high;
}

/** Profile of a scan: nearest echo, edges, width and the plan. */
static void profile_core (obstacle_core_point_t const * p_fine, uint8_t n_fine,
                          obstacle_core_point_t const * p_coarse,
                          uint8_t n_coarse, obstacle_core_profile_t * p_out)
{
    uint8_t        nearest = 0u;
    uint16_t const r_min   = nearest_echo(p_fine, n_fine, &nearest);

    profile_clear(p_out);

    if (r_min != OBST_RANGE_NONE)
    {
        profile_measure(p_fine, n_fine, p_coarse, n_coarse, nearest, r_min,
                        p_out);
    }

    /* else nothing in front: CONTINUE */
}

/** The profile before anything is measured: nothing seen, CONTINUE. */
static void profile_clear (obstacle_core_profile_t * p_out)
{
    p_out->b_valid       = false;
    p_out->plan          = OBST_PLAN_CONTINUE;
    p_out->closest_mm    = OBST_RANGE_NONE;
    p_out->w_mm          = 0u;
    p_out->left_lat_mm   = 0;
    p_out->right_lat_mm  = 0;
    p_out->clr_l_mm      = OBST_CLEAR_MAX_MM;
    p_out->clr_r_mm      = OBST_CLEAR_MAX_MM;
    p_out->deviation_mm  = 0;
    p_out->nearest_angle = OBST_SERVO_CENTRE_DEG;
    p_out->b_sure        = false;
    p_out->plan_l_mm     = 0;
    p_out->plan_r_mm     = 0;
}

/** Index of the nearest usable echo (*p_nearest) and its range, or
   OBST_RANGE_NONE when there is none. */
static uint16_t nearest_echo (obstacle_core_point_t const * p_fine,
                              uint8_t n_fine, uint8_t * p_nearest)
{
    uint16_t r_min   = OBST_RANGE_NONE;
    uint8_t  nearest = 0u;
    uint8_t  idx;

    /* 9 Nov: the nearest echo in the car's path first; a wall beside the
       track can be nearer than the box on the line (02:59 run). */
    for (idx = 0u; idx < n_fine; idx++)
    {
        /* Cast (int32_t)p_fine[idx].angle_deg: every int16_t value fits. */
        if (usable(&p_fine[idx]) && (p_fine[idx].r_mm < r_min)
            && (app_types_abs_i32(lateral_mm(
                    (int32_t)p_fine[idx].angle_deg * 100, p_fine[idx].r_mm))
                <= ((OBST_ROBOT_WIDTH_MM / 2) + OBST_MARGIN_MM)))
        {
            r_min   = p_fine[idx].r_mm;
            nearest = idx;
        }
    }

    if (OBST_RANGE_NONE == r_min)
    {
        /* nothing in the path: the nearest echo anywhere (may be CONTINUE) */
        for (idx = 0u; idx < n_fine; idx++)
        {
            if (usable(&p_fine[idx]) && (p_fine[idx].r_mm < r_min))
            {
                r_min   = p_fine[idx].r_mm;
                nearest = idx;
            }
        }
    }

    *p_nearest = nearest;
    return r_min;
}

/** Grow the obstacle from its nearest echo, measure its edges and plan. */
static void profile_measure (obstacle_core_point_t const * p_fine,
                             uint8_t                       n_fine,
                             obstacle_core_point_t const * p_coarse,
                             uint8_t n_coarse, uint8_t nearest, uint16_t r_min,
                             obstacle_core_profile_t * p_out)
{
    uint8_t low;
    uint8_t high;
    int32_t a_left;
    int32_t a_right;
    int32_t lat_l;
    int32_t lat_r;
    int32_t pl_l;
    int32_t pl_r;

    grow(p_fine, n_fine, nearest, r_min, &low, &high);

    /* The fine scan runs from low to high angle (right to left).  The beam
       sees an edge up to half a beam early: move each edge inward. */
    /* Casts (int32_t)p_fine[low].angle_deg, (int32_t)p_fine[high].angle_deg:
       every int16_t value fits. */
    a_right = ((int32_t)p_fine[low].angle_deg * 100) + OBST_HALF_BEAM_CDEG;
    a_left  = ((int32_t)p_fine[high].angle_deg * 100) - OBST_HALF_BEAM_CDEG;

    if (a_left < a_right)
    {
        int32_t const mid = (a_left + a_right) / 2; /* narrower than the beam */

        a_left  = mid;
        a_right = mid;
    }

    /* Casts (int32_t)p_fine[high].angle_deg, (int32_t)p_fine[low].angle_deg:
       every int16_t value fits. */
    lat_l = lateral2_mm((int32_t)p_fine[high].angle_deg * 100, a_left,
                        p_fine[high].r_mm);
    lat_r = lateral2_mm((int32_t)p_fine[low].angle_deg * 100, a_right,
                        p_fine[low].r_mm);

    /* Edges for the PLAN (3 Nov): uncorrected, so never narrower than seen.
       Too narrow to be plausible: at least OBST_WIDTH_MAX wide around the
       middle of what was seen.  An edge that ran out of the scan window: the
       object may reach OBST_WIDTH_MAX from the edge that was seen. */
    {
        /* Casts (int32_t)p_fine[high].angle_deg,
           (int32_t)p_fine[low].angle_deg: every int16_t value fits. */
        int32_t const raw_l = lateral_mm((int32_t)p_fine[high].angle_deg * 100,
                                         p_fine[high].r_mm);
        int32_t const raw_r =
            lateral_mm((int32_t)p_fine[low].angle_deg * 100, p_fine[low].r_mm);
        int32_t const centre  = (raw_l + raw_r) / 2;
        int32_t const w_est   = lat_l - lat_r;
        bool const    b_r_end = (low > 0u); /* a miss right of the right edge */
        bool const    b_l_end =
            ((high + 1u) < n_fine); /* a miss left of the left edge   */

        p_out->b_sure = (w_est >= OBST_WIDTH_MIN_MM)
                        && (w_est <= OBST_WIDTH_MAX_MM) && b_r_end && b_l_end;
        pl_l          = raw_l;
        pl_r          = raw_r;

        if (w_est < OBST_WIDTH_MIN_MM)
        {
            pl_l = app_types_max_i32(pl_l, centre + (OBST_WIDTH_MAX_MM / 2));
            pl_r = app_types_min_i32(pl_r, centre - (OBST_WIDTH_MAX_MM / 2));
        }

        if (!b_l_end)
        {
            pl_l = app_types_max_i32(pl_l, raw_r + OBST_WIDTH_MAX_MM);
        }

        if (!b_r_end)
        {
            pl_r = app_types_min_i32(pl_r, raw_l - OBST_WIDTH_MAX_MM);
        }
    }

    /* Casts (int16_t)lat_l, (int16_t)lat_r: lateral edges of echoes within 800
       mm are within +-800 mm; (uint16_t)app_types_max_i32(lat_l - ...): edges
       within +-800 mm, so the width is 0..1600 mm. */
    p_out->b_valid       = true;
    p_out->closest_mm    = r_min;
    p_out->nearest_angle = p_fine[nearest].angle_deg;
    p_out->left_lat_mm   = (int16_t)lat_l;
    p_out->right_lat_mm  = (int16_t)lat_r;
    p_out->w_mm          = (uint16_t)app_types_max_i32(lat_l - lat_r, 0);
    decide(p_fine, n_fine, p_coarse, n_coarse, low, high, r_min, pl_l, pl_r,
           p_out);
}

/** Clearances beside the planned edges and the PLAN (FR11).  Points lo..hi
   of the fine scan are the obstacle itself (lo > hi: none). */
static void decide (obstacle_core_point_t const * p_fine, uint8_t n_fine,
                    obstacle_core_point_t const * p_coarse, uint8_t n_coarse,
                    uint8_t low, uint8_t high, uint16_t r_min, int32_t pl_l,
                    int32_t pl_r, obstacle_core_profile_t * p_out)
{
    int32_t const corridor = (OBST_ROBOT_WIDTH_MM / 2) + OBST_MARGIN_MM;
    int32_t const need     = OBST_ROBOT_WIDTH_MM + (2 * OBST_MARGIN_MM);
    int32_t       clr_l    = OBST_CLEAR_MAX_MM;
    int32_t       clr_r    = OBST_CLEAR_MAX_MM;
    uint8_t       idx;

    /* Clearance: nearby points outside the obstacle, from both scans. */
    /* Cast (uint8_t)(n_fine + n_coarse): at most OBST_SCAN_MAX_POINTS +
       OBST_COARSE_PINGS (37) points. */
    for (idx = 0u; idx < (uint8_t)(n_fine + n_coarse); idx++)
    {
        obstacle_core_point_t const * p_point =
            (idx < n_fine) ? &p_fine[idx] : &p_coarse[idx - n_fine];
        int32_t lat;

        /* Skipped: the obstacle's own fine pings, unusable or far pings,
           and a coarse ping at the obstacle's own range (the obstacle seen
           through the side of the beam, not a second object). */
        /* Cast (uint32_t)r_min: every uint16_t value fits. */
        if (((idx < n_fine) && (idx >= low) && (idx <= high))
            || !usable(p_point)
            || (p_point->r_mm
                > (((uint32_t)r_min + DEPTH_TOL_MM) + OBST_NEAR_EXTRA_MM))
            || ((idx >= n_fine)
                && (p_point->r_mm <= ((uint32_t)r_min + DEPTH_TOL_MM))))
        {
            /* not a clearance point */
        }
        else
        {
            /* Cast (int32_t)p_point->angle_deg: every int16_t value fits. */
            lat = lateral_mm((int32_t)p_point->angle_deg * 100, p_point->r_mm);

            if (lat > pl_l)
            {
                clr_l = app_types_min_i32(clr_l, lat - pl_l);
            }
            else if (lat < pl_r)
            {
                clr_r = app_types_min_i32(clr_r, pl_r - lat);
            }
            else
            {
                /* inside the obstacle's lateral span (behind it) */
            }
        }
    }

    /* Casts (uint16_t)clr_l, (uint16_t)clr_r: clearances are limited to
       0..OBST_CLEAR_MAX_MM (500); (int16_t)pl_l, (int16_t)pl_r: planning edges
       are within +-800 mm plus OBST_WIDTH_MAX_MM. */
    p_out->clr_l_mm     = (uint16_t)clr_l;
    p_out->clr_r_mm     = (uint16_t)clr_r;
    p_out->plan_l_mm    = (int16_t)pl_l;
    p_out->plan_r_mm    = (int16_t)pl_r;
    p_out->deviation_mm = 0;

    if ((pl_r > corridor) || (pl_l < -corridor))
    {
        p_out->plan = OBST_PLAN_CONTINUE; /* beside the path */
    }
    else
    {
        int32_t const dev_l = pl_l + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2)
                              + OBST_EDGE_ALLOW_MM;
        int32_t const dev_r = -pl_r + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2)
                              + OBST_EDGE_ALLOW_MM;
        bool const    b_left  = (clr_l >= need);
        bool const    b_right = (clr_r >= need);

        if (b_left && (!b_right || (dev_l <= dev_r)))
        {
            /* Cast (int16_t)dev_l: an edge (+-1000 mm) plus half the car and
               the margin. */
            p_out->plan         = OBST_PLAN_LEFT;
            p_out->deviation_mm = (int16_t)dev_l;
        }
        else if (b_right)
        {
            /* Cast (int16_t)-dev_r: an edge (+-1000 mm) plus half the car and
               the margin. */
            p_out->plan         = OBST_PLAN_RIGHT;
            p_out->deviation_mm = (int16_t)-dev_r;
        }
        else
        {
            p_out->plan = OBST_PLAN_BLOCKED;
        }
    }
}

/** Plan as for an OBST_WIDTH_MAX object centred straight ahead at r. */
static void force_ahead (obstacle_core_point_t const * p_fine, uint8_t n_fine,
                         obstacle_core_point_t const * p_coarse,
                         uint8_t n_coarse, uint16_t range,
                         obstacle_core_profile_t * p_out)
{
    p_out->b_valid       = true;
    p_out->b_sure        = false;
    p_out->closest_mm    = range;
    p_out->nearest_angle = OBST_SERVO_CENTRE_DEG;

    if (0u == p_out->w_mm)
    {
        p_out->left_lat_mm  = 0;
        p_out->right_lat_mm = 0;
    }

    decide(p_fine, n_fine, p_coarse, n_coarse, 1u, 0u, range,
           OBST_WIDTH_MAX_MM / 2, -(OBST_WIDTH_MAX_MM / 2), p_out);
}

/* ---- scan sequencer ---------------------------------------------------- */
/** Refining an edge: true if the point belongs to the obstacle. */
static bool is_hit (obstacle_core_point_t const * p_point, uint16_t r_min)
{
    return same_object(p_point, r_min);
}

/** obstacle_core_scan_feed(): the coarse sweep is complete; aim the fine
    sweep at the nearest usable ping, or finish if there is none. */
static void coarse_done (obstacle_core_scan_t * p_s)
{
    uint8_t  idx;
    uint16_t best   = OBST_RANGE_NONE;
    int16_t  centre = OBST_SERVO_CENTRE_DEG;

    for (idx = 0u; idx < OBST_COARSE_PINGS; idx++)
    {
        if (usable(&p_s->coarse[idx]) && (p_s->coarse[idx].r_mm < best))
        {
            best   = p_s->coarse[idx].r_mm;
            centre = p_s->coarse[idx].angle_deg;
        }
    }

    if (OBST_RANGE_NONE == best)
    {
        p_s->phase = OBST_SCAN_DONE; /* nothing near: CONTINUE */
    }
    else
    {
        /* Casts (int16_t)app_types_max_i32(centre ...): the value
           is 20..32727; (int16_t)app_types_min_i32(centre ...): the
           value is -32728..160. */
        p_s->phase   = OBST_SCAN_FINE;
        p_s->angle   = (int16_t)app_types_max_i32(centre - OBST_FINE_HALF_DEG,
                                                  OBST_SCAN_MIN_DEG);
        p_s->fine_to = (int16_t)app_types_min_i32(centre + OBST_FINE_HALF_DEG,
                                                  OBST_SCAN_MAX_DEG);
    }
}

/** obstacle_core_scan_feed() while refining an edge: halve the hit-miss
    interval with the ping r_mm. */
static void refine_step (obstacle_core_scan_t * p_s, uint16_t r_mm)
{
    obstacle_core_point_t point;

    point.angle_deg = p_s->angle;
    point.r_mm      = r_mm;
    fine_insert(p_s, p_s->angle, r_mm);

    if (is_hit(&point, p_s->r_min))
    {
        p_s->hit = p_s->angle;
    }
    else
    {
        p_s->miss = p_s->angle;
    }

    /* Cast (int16_t)((p_s->hit + p_s->miss) / ...): the value is
       -32768..32767. */
    p_s->iter++;
    p_s->angle = (int16_t)((p_s->hit + p_s->miss) / 2);

    if ((p_s->iter >= OBST_REFINE_ITERS) || (p_s->angle == p_s->hit)
        || (p_s->angle == p_s->miss))
    {
        if (OBST_SCAN_REFINE_R == p_s->phase)
        {
            start_refine(p_s, false);
        }
        else
        {
            p_s->phase = OBST_SCAN_DONE;
        }
    }
}

/** obstacle_core_edge_plan() with something to plan from: the edges, the
    planned side and the deviation. */
static void edge_plan_apply (obstacle_core_profile_t * p_prof, bool b_has_l,
                             int32_t lat_l, bool b_has_r, int32_t lat_r)
{
    int32_t const extra =
        OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM;
    bool b_left;

    bool const b_servo =
        p_prof->b_valid
        && (p_prof->w_mm > 0u); /* the servo scan saw a real face */

    if (!b_has_l && !b_has_r)
    {
        /* 6 Nov: no echo on either side: plan from the servo scan's edges
           with the same side rule below (keeping the servo scan's own
           plan chose sides by other rules: left 35 cm when right was
           the short way) */
        lat_l = p_prof->plan_l_mm;
        lat_r = p_prof->plan_r_mm;
    }

    /* 4 Nov: a side with no echo at all is not proof that the obstacle runs
       on that way (a centred box got 0 echoes on the left and the car went
       right).  Take the servo scan's edge for it when that scan saw a real
       face, otherwise as far as the largest obstacle from the seen edge. */
    if (!b_has_l)
    {
        lat_l = b_servo ? app_types_max_i32(p_prof->plan_l_mm, lat_r)
                        : (lat_r + OBST_WIDTH_MAX_MM);
    }

    if (!b_has_r)
    {
        lat_r = b_servo ? app_types_min_i32(p_prof->plan_r_mm, lat_l)
                        : (lat_l - OBST_WIDTH_MAX_MM);
    }

    if (lat_l < lat_r)
    {
        int32_t const mid = (lat_l + lat_r) / 2;

        lat_l = mid;
        lat_r = mid;
    }

    /* Casts (int16_t)lat_l, (int16_t)lat_r: lateral edges of echoes within 800
       mm are within +-800 mm; (uint16_t)(lat_l - lat_r): lat_l >= lat_r here,
       and both are within +-800 mm. */
    p_prof->b_valid      = true;
    p_prof->b_sure       = b_has_l && b_has_r;
    p_prof->left_lat_mm  = (int16_t)lat_l;
    p_prof->right_lat_mm = (int16_t)lat_r;
    p_prof->plan_l_mm    = (int16_t)lat_l;
    p_prof->plan_r_mm    = (int16_t)lat_r;
    p_prof->w_mm         = (uint16_t)(lat_l - lat_r);

    /* 5 Nov: the smaller step-out; within OBST_TIE_MM they are equal, then
       the side whose edge was measured rather than filled in, then
       OBST_PREF_LEFT.  (4 Nov: from the edges only, not the servo scan's
       clearances: that scan split a centred box in two and its left half
       "blocked" the left, so the car went right, off the mat.) */
    {
        int32_t const sh_l = lat_l + extra;
        int32_t const sh_r = -lat_r + extra;

        if ((sh_l + OBST_TIE_MM) < sh_r)
        {
            b_left = true;
        }
        else if ((sh_r + OBST_TIE_MM) < sh_l)
        {
            b_left = false;
        }
        else if (b_has_l != b_has_r)
        {
            b_left = b_has_l;
        }
        else
        {
            b_left = (OBST_PREF_LEFT != 0);
        }
    }
    /* Cast (int16_t)(b_left ? (lat_l + extra) ...): edge offsets (+-800 mm)
       plus a margin of a few hundred mm. */
    p_prof->plan = b_left ? OBST_PLAN_LEFT : OBST_PLAN_RIGHT;
    p_prof->deviation_mm =
        (int16_t)(b_left ? (lat_l + extra) : -(-lat_r + extra));
}

/** Insert keeping fine[] sorted by angle (refine points land between). */
static void fine_insert (obstacle_core_scan_t * p_s, int16_t angle,
                         uint16_t r_mm)
{
    uint8_t idx;

    if (p_s->n_fine < OBST_SCAN_MAX_POINTS) /* else the window is full */
    {
        idx = p_s->n_fine;

        while ((idx > 0u) && (p_s->fine[idx - 1u].angle_deg > angle))
        {
            p_s->fine[idx] = p_s->fine[idx - 1u];
            idx--;
        }

        p_s->fine[idx].angle_deg = angle;
        p_s->fine[idx].r_mm      = r_mm;
        p_s->n_fine++;
    }
}

/** Edges of the obstacle in the fine scan; false if none or no outer miss. */
static void start_refine (obstacle_core_scan_t * p_s, bool b_right)
{
    uint8_t  idx;
    uint8_t  nearest = 0u;
    uint8_t  low;
    uint8_t  high;
    uint16_t r_min    = OBST_RANGE_NONE;
    bool     b_refine = false;

    for (idx = 0u; idx < p_s->n_fine; idx++)
    {
        if (usable(&p_s->fine[idx]) && (p_s->fine[idx].r_mm < r_min))
        {
            r_min   = p_s->fine[idx].r_mm;
            nearest = idx;
        }
    }

    p_s->r_min = r_min;
    p_s->iter  = 0u;

    if (OBST_RANGE_NONE == r_min)
    {
        p_s->phase = OBST_SCAN_DONE;
    }
    else
    {
        grow(p_s->fine, p_s->n_fine, nearest, r_min, &low, &high);

        if (b_right && (low > 0u))
        {
            p_s->phase = OBST_SCAN_REFINE_R;
            p_s->hit   = p_s->fine[low].angle_deg;
            p_s->miss  = p_s->fine[low - 1u].angle_deg;
            b_refine   = true;
        }
        else if (!b_right && ((high + 1u) < p_s->n_fine))
        {
            p_s->phase = OBST_SCAN_REFINE_L;
            p_s->hit   = p_s->fine[high].angle_deg;
            p_s->miss  = p_s->fine[high + 1u].angle_deg;
            b_refine   = true;
        }
        else if (b_right)
        {
            /* right edge at the window limit: refine the left edge */
            start_refine(p_s, false);
        }
        else
        {
            /* edge at the window limit: nothing to refine on this side */
            p_s->phase = OBST_SCAN_DONE;
        }
    }

    if (b_refine)
    {
        /* Cast (int16_t)((p_s->hit + p_s->miss) / ...): the value is
           -32768..32767. */
        p_s->angle = (int16_t)((p_s->hit + p_s->miss) / 2);

        if ((p_s->angle == p_s->hit) || (p_s->angle == p_s->miss))
        {
            p_s->iter = OBST_REFINE_ITERS; /* already 1 deg apart */
        }
    }
}

/** Lateral position (mm, + left) of the echo edge seen at phi_cdeg and r_mm,
    with the calibrated beam angle on side side. */
static int32_t width_lat (int32_t phi_cdeg, uint16_t r_mm, int32_t side)
{
    int32_t const psi = phi_cdeg - (side * OBST_WIDTH_BEAM_CDEG);

    /* Casts (int32_t)((((int64_t)OBST_US_AHEAD_ ...))): |sin|, |cos| <= 1.0 in
       Q15, so the result is no larger than the int32_t factor;
       (int64_t)OBST_US_AHEAD_MM: a constant that fits; (int64_t)r_mm: every
       uint16_t value fits. */
    return (int32_t)((((int64_t)OBST_US_AHEAD_MM * app_math_sin_q15(phi_cdeg))
                      + ((int64_t)r_mm * app_math_sin_q15(psi)))
                     / 32768);
}

/*** end of file ***/
