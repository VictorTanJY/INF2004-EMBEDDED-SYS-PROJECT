/*
 *  obstacle_core.c
 *  Subsystem 5 -- obstacle geometry (pure).  See obstacle_core.h.
 */

#include "obstacle_core.h"
#include "mission_cfg.h"
#include "../app_types.h"
#include "../app_math.h"

#define OBST_NEAR_EXTRA_MM      300     /* other objects this close matter for clearance */

uint16_t obst_median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t lo = a;
    uint16_t hi = b;

    if (lo > hi)
    {
        lo = b;
        hi = a;
    }
    if (c <= lo)
    {
        return lo;
    }
    if (c >= hi)
    {
        return hi;
    }
    return c;
}

uint16_t obst_echo_to_mm(uint32_t width_us)
{
    if ((width_us == 0u) || (width_us >= OBST_ECHO_MAX_US))
    {
        return OBST_RANGE_NONE;
    }
    return (uint16_t)((width_us * 10u) / 58u);      /* us / 58 = cm */
}

/* Lateral (+ left) of a servo-scan echo from the servo shaft (7 Nov): the
   sensor sits OBST_SERVO_ARM_MM out along the horn at servo angle
   servo_cdeg, and the echo is r_mm from it along beam_cdeg (the servo angle,
   or an edge corrected by half a beam). */
static int32_t lateral2_mm(int32_t servo_cdeg, int32_t beam_cdeg, uint16_t r_mm)
{
    int64_t const q = ((int64_t)r_mm * app_cos_q15(beam_cdeg)) +
                      ((int64_t)OBST_SERVO_ARM_MM * app_cos_q15(servo_cdeg));

    return -(int32_t)(q / 32768);
}

static int32_t lateral_mm(int32_t angle_cdeg, uint16_t r_mm)
{
    return lateral2_mm(angle_cdeg, angle_cdeg, r_mm);
}

/* An echo that can be part of an obstacle: in range. */
static bool usable(obst_point_t const * p)
{
    return (p->r_mm != OBST_RANGE_NONE) && (p->r_mm <= OBST_MAX_RANGE_MM);
}

static bool same_object(obst_point_t const * p, uint16_t r_min)
{
    return usable(p) && (p->r_mm <= ((uint32_t)r_min + OBST_DEPTH_TOL_MM));
}

/* The obstacle = the run of points around the nearest one at its range.
   Consecutive-angle rule (hysteresis): a single point that misses between
   two that hit is a dropout, not an edge; an edge needs 2 misses. */
static int32_t lateral_mm(int32_t angle_cdeg, uint16_t r_mm);

/* Part of the obstacle grown from the nearest echo: at its range and
   (9 Nov) within OBST_GROUP_MAX_MM sideways of it. */
static bool in_group(obst_point_t const * p, uint16_t r_min, int32_t lat0)
{
    return same_object(p, r_min) &&
           (app_abs_i32(lateral_mm((int32_t)p->angle_deg * 100, p->r_mm) - lat0) <= OBST_GROUP_MAX_MM);
}

static void grow(obst_point_t const * p_pts, uint8_t n, uint8_t nearest, uint16_t r_min,
                 uint8_t * p_lo, uint8_t * p_hi)
{
    int32_t const lat0 = lateral_mm((int32_t)p_pts[nearest].angle_deg * 100, p_pts[nearest].r_mm);
    uint8_t       lo   = nearest;
    uint8_t       hi   = nearest;

    for (;;)
    {
        if ((lo > 0u) && in_group(&p_pts[lo - 1u], r_min, lat0))
        {
            lo--;
        }
        else if ((lo > 1u) && in_group(&p_pts[lo - 2u], r_min, lat0))
        {
            lo = (uint8_t)(lo - 2u);
        }
        else
        {
            break;
        }
    }
    for (;;)
    {
        if (((hi + 1u) < n) && in_group(&p_pts[hi + 1u], r_min, lat0))
        {
            hi++;
        }
        else if (((hi + 2u) < n) && in_group(&p_pts[hi + 2u], r_min, lat0))
        {
            hi = (uint8_t)(hi + 2u);
        }
        else
        {
            break;
        }
    }
    *p_lo = lo;
    *p_hi = hi;
}

static void decide(obst_point_t const * p_fine, uint8_t n_fine,
                   obst_point_t const * p_coarse, uint8_t n_coarse,
                   uint8_t lo, uint8_t hi, uint16_t r_min, int32_t pl_l, int32_t pl_r,
                   obst_profile_t * p_out);

static void profile_core(obst_point_t const * p_fine, uint8_t n_fine,
                         obst_point_t const * p_coarse, uint8_t n_coarse,
                         obst_profile_t * p_out)
{
    uint8_t       i;
    uint8_t       nearest = 0u;
    uint8_t       lo;
    uint8_t       hi;
    uint16_t      r_min = OBST_RANGE_NONE;
    int32_t       a_left;
    int32_t       a_right;
    int32_t       lat_l;
    int32_t       lat_r;
    int32_t       pl_l;
    int32_t       pl_r;

    p_out->b_valid      = false;
    p_out->plan         = (uint8_t)OBST_PLAN_CONTINUE;
    p_out->closest_mm   = OBST_RANGE_NONE;
    p_out->w_mm         = 0u;
    p_out->left_lat_mm  = 0;
    p_out->right_lat_mm = 0;
    p_out->clr_l_mm     = OBST_CLEAR_MAX_MM;
    p_out->clr_r_mm     = OBST_CLEAR_MAX_MM;
    p_out->deviation_mm = 0;
    p_out->nearest_angle = OBST_SERVO_CENTRE_DEG;
    p_out->b_sure       = false;
    p_out->plan_l_mm    = 0;
    p_out->plan_r_mm    = 0;

    /* 9 Nov: the nearest echo in the car's path first; a wall beside the
       track can be nearer than the box on the line (02:59 run). */
    for (i = 0u; i < n_fine; i++)
    {
        if (usable(&p_fine[i]) && (p_fine[i].r_mm < r_min) &&
            (app_abs_i32(lateral_mm((int32_t)p_fine[i].angle_deg * 100, p_fine[i].r_mm)) <=
             ((OBST_ROBOT_WIDTH_MM / 2) + OBST_MARGIN_MM)))
        {
            r_min   = p_fine[i].r_mm;
            nearest = i;
        }
    }
    if (r_min == OBST_RANGE_NONE)
    {
        /* nothing in the path: the nearest echo anywhere (may be CONTINUE) */
        for (i = 0u; i < n_fine; i++)
        {
            if (usable(&p_fine[i]) && (p_fine[i].r_mm < r_min))
            {
                r_min   = p_fine[i].r_mm;
                nearest = i;
            }
        }
    }
    if (r_min == OBST_RANGE_NONE)
    {
        return;                         /* nothing in front: CONTINUE */
    }

    grow(p_fine, n_fine, nearest, r_min, &lo, &hi);

    /* The fine scan runs from low to high angle (right to left).  The beam
       sees an edge up to half a beam early: move each edge inward. */
    a_right = ((int32_t)p_fine[lo].angle_deg * 100) + OBST_HALF_BEAM_CDEG;
    a_left  = ((int32_t)p_fine[hi].angle_deg * 100) - OBST_HALF_BEAM_CDEG;
    if (a_left < a_right)
    {
        int32_t const mid = (a_left + a_right) / 2;     /* narrower than the beam */

        a_left  = mid;
        a_right = mid;
    }
    lat_l = lateral2_mm((int32_t)p_fine[hi].angle_deg * 100, a_left, p_fine[hi].r_mm);
    lat_r = lateral2_mm((int32_t)p_fine[lo].angle_deg * 100, a_right, p_fine[lo].r_mm);

    /* Edges for the PLAN (3 Nov): uncorrected, so never narrower than seen.
       Too narrow to be plausible: at least OBST_WIDTH_MAX wide around the
       middle of what was seen.  An edge that ran out of the scan window: the object
       may reach OBST_WIDTH_MAX from the edge that was seen. */
    {
        int32_t const raw_l   = lateral_mm((int32_t)p_fine[hi].angle_deg * 100, p_fine[hi].r_mm);
        int32_t const raw_r   = lateral_mm((int32_t)p_fine[lo].angle_deg * 100, p_fine[lo].r_mm);
        int32_t const centre  = (raw_l + raw_r) / 2;
        int32_t const w_est   = lat_l - lat_r;
        bool const    b_r_end = (lo > 0u);              /* a miss right of the right edge */
        bool const    b_l_end = ((hi + 1u) < n_fine);   /* a miss left of the left edge   */

        p_out->b_sure = (w_est >= OBST_WIDTH_MIN_MM) && (w_est <= OBST_WIDTH_MAX_MM) && b_r_end && b_l_end;
        pl_l = raw_l;
        pl_r = raw_r;
        if (w_est < OBST_WIDTH_MIN_MM)
        {
            pl_l = app_max_i32(pl_l, centre + (OBST_WIDTH_MAX_MM / 2));
            pl_r = app_min_i32(pl_r, centre - (OBST_WIDTH_MAX_MM / 2));
        }
        if (!b_l_end)
        {
            pl_l = app_max_i32(pl_l, raw_r + OBST_WIDTH_MAX_MM);
        }
        if (!b_r_end)
        {
            pl_r = app_min_i32(pl_r, raw_l - OBST_WIDTH_MAX_MM);
        }
    }

    p_out->b_valid       = true;
    p_out->closest_mm    = r_min;
    p_out->nearest_angle = p_fine[nearest].angle_deg;
    p_out->left_lat_mm   = (int16_t)lat_l;
    p_out->right_lat_mm  = (int16_t)lat_r;
    p_out->w_mm          = (uint16_t)app_max_i32(lat_l - lat_r, 0);
    decide(p_fine, n_fine, p_coarse, n_coarse, lo, hi, r_min, pl_l, pl_r, p_out);
}

/* Clearances beside the planned edges and the PLAN (FR11).  Points lo..hi
   of the fine scan are the obstacle itself (lo > hi: none). */
static void decide(obst_point_t const * p_fine, uint8_t n_fine,
                   obst_point_t const * p_coarse, uint8_t n_coarse,
                   uint8_t lo, uint8_t hi, uint16_t r_min, int32_t pl_l, int32_t pl_r,
                   obst_profile_t * p_out)
{
    int32_t const corridor = (OBST_ROBOT_WIDTH_MM / 2) + OBST_MARGIN_MM;
    int32_t const need     = OBST_ROBOT_WIDTH_MM + (2 * OBST_MARGIN_MM);
    int32_t       clr_l    = OBST_CLEAR_MAX_MM;
    int32_t       clr_r    = OBST_CLEAR_MAX_MM;
    uint8_t       i;

    /* Clearance: nearby points outside the obstacle, from both scans. */
    for (i = 0u; i < (uint8_t)(n_fine + n_coarse); i++)
    {
        obst_point_t const * p = (i < n_fine) ? &p_fine[i] : &p_coarse[i - n_fine];
        int32_t              lat;

        if (((i < n_fine) && (i >= lo) && (i <= hi)) || !usable(p) ||
            (p->r_mm > ((uint32_t)r_min + OBST_DEPTH_TOL_MM + OBST_NEAR_EXTRA_MM)))
        {
            continue;
        }
        if ((i >= n_fine) && (p->r_mm <= ((uint32_t)r_min + OBST_DEPTH_TOL_MM)))
        {
            /* A coarse ping at the obstacle's own range is the obstacle seen
               through the side of the beam, not a second object. */
            continue;
        }
        lat = lateral_mm((int32_t)p->angle_deg * 100, p->r_mm);
        if (lat > pl_l)
        {
            clr_l = app_min_i32(clr_l, lat - pl_l);
        }
        else if (lat < pl_r)
        {
            clr_r = app_min_i32(clr_r, pl_r - lat);
        }
        else
        {
            /* inside the obstacle's lateral span (behind it) */
        }
    }

    p_out->clr_l_mm      = (uint16_t)clr_l;
    p_out->clr_r_mm      = (uint16_t)clr_r;
    p_out->plan_l_mm     = (int16_t)pl_l;
    p_out->plan_r_mm     = (int16_t)pl_r;
    p_out->deviation_mm  = 0;

    if ((pl_r > corridor) || (pl_l < -corridor))
    {
        p_out->plan = (uint8_t)OBST_PLAN_CONTINUE;      /* beside the path */
    }
    else
    {
        int32_t const dev_l  = pl_l + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM;
        int32_t const dev_r  = -pl_r + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM;
        bool const    b_left = (clr_l >= need);
        bool const    b_right = (clr_r >= need);

        if (b_left && (!b_right || (dev_l <= dev_r)))
        {
            p_out->plan         = (uint8_t)OBST_PLAN_LEFT;
            p_out->deviation_mm = (int16_t)dev_l;
        }
        else if (b_right)
        {
            p_out->plan         = (uint8_t)OBST_PLAN_RIGHT;
            p_out->deviation_mm = (int16_t)-dev_r;
        }
        else
        {
            p_out->plan = (uint8_t)OBST_PLAN_BLOCKED;
        }
    }
}

/* Plan as for an OBST_WIDTH_MAX object centred straight ahead at r. */
static void force_ahead(obst_point_t const * p_fine, uint8_t n_fine,
                        obst_point_t const * p_coarse, uint8_t n_coarse,
                        uint16_t r, obst_profile_t * p_out)
{
    p_out->b_valid       = true;
    p_out->b_sure        = false;
    p_out->closest_mm    = r;
    p_out->nearest_angle = OBST_SERVO_CENTRE_DEG;
    if (p_out->w_mm == 0u)
    {
        p_out->left_lat_mm  = 0;
        p_out->right_lat_mm = 0;
    }
    decide(p_fine, n_fine, p_coarse, n_coarse, 1u, 0u, r,
           OBST_WIDTH_MAX_MM / 2, -(OBST_WIDTH_MAX_MM / 2), p_out);
}

void obst_profile(obst_point_t const * p_fine, uint8_t n_fine,
                  obst_point_t const * p_coarse, uint8_t n_coarse,
                  obst_profile_t * p_out)
{
    uint16_t r_ahead = OBST_RANGE_NONE;
    uint8_t  i;

    profile_core(p_fine, n_fine, p_coarse, n_coarse, p_out);

    /* Guard (3 Nov: twice the profile said CONTINUE, or found nothing at
       all, with the box 15-18 cm straight ahead, and the car drove into
       it): if any echo within OBST_GUARD_HALF_DEG of straight ahead is
       closer than OBST_GUARD_MM, raw (no background, no range filter),
       the plan is never CONTINUE: plan as for an OBST_WIDTH_MAX object
       centred straight ahead at that range. */
    for (i = 0u; i < (uint8_t)(n_fine + n_coarse); i++)
    {
        obst_point_t const * p = (i < n_fine) ? &p_fine[i] : &p_coarse[i - n_fine];

        if ((app_abs_i32((int32_t)p->angle_deg - OBST_SERVO_CENTRE_DEG) <= OBST_GUARD_HALF_DEG) &&
            (p->r_mm < r_ahead))
        {
            r_ahead = p->r_mm;
        }
    }
    if ((r_ahead < OBST_GUARD_MM) && (p_out->plan == (uint8_t)OBST_PLAN_CONTINUE))
    {
        force_ahead(p_fine, n_fine, p_coarse, n_coarse, r_ahead, p_out);
    }
}

void obst_profile_detected(obst_point_t const * p_fine, uint8_t n_fine,
                           obst_point_t const * p_coarse, uint8_t n_coarse,
                           uint16_t r_det_mm, uint16_t back_mm, obst_profile_t * p_out)
{
    obst_profile(p_fine, n_fine, p_coarse, n_coarse, p_out);

    /* 3 Nov: a round water bottle was seen while driving (16 cm, median
       of 3) but not in the scan (curved glass returns few echoes), the
       profile was empty, the plan CONTINUE, and the car drove into it.
       Nothing seen in the scan after a detection = trust the detection. */
    /* 4 Nov: the test is on the range when it was seen, not after the
       back-off (19.2 cm + 6 cm passed the 25 cm limit and the car drove on) */
    if (!p_out->b_valid && (r_det_mm < OBST_GUARD_MM))
    {
        force_ahead(p_fine, n_fine, p_coarse, n_coarse, (uint16_t)(r_det_mm + back_mm), p_out);
    }
}

/* ---- scan sequencer ---------------------------------------------------- */
static bool is_hit(obst_point_t const * p, uint16_t r_min)
{
    return same_object(p, r_min);
}

/* Insert keeping fine[] sorted by angle (refine points land between). */
static void fine_insert(obst_scan_t * p_s, int16_t angle, uint16_t r_mm)
{
    uint8_t i;

    if (p_s->n_fine >= OBST_SCAN_MAX_POINTS)
    {
        return;
    }
    i = p_s->n_fine;
    while ((i > 0u) && (p_s->fine[i - 1u].angle_deg > angle))
    {
        p_s->fine[i] = p_s->fine[i - 1u];
        i--;
    }
    p_s->fine[i].angle_deg = angle;
    p_s->fine[i].r_mm      = r_mm;
    p_s->n_fine++;
}

/* Edges of the obstacle in the fine scan; false if none or no outer miss. */
static void start_refine(obst_scan_t * p_s, bool b_right)
{
    uint8_t  i;
    uint8_t  nearest = 0u;
    uint8_t  lo;
    uint8_t  hi;
    uint16_t r_min = OBST_RANGE_NONE;

    for (i = 0u; i < p_s->n_fine; i++)
    {
        if (usable(&p_s->fine[i]) && (p_s->fine[i].r_mm < r_min))
        {
            r_min   = p_s->fine[i].r_mm;
            nearest = i;
        }
    }
    p_s->r_min = r_min;
    p_s->iter  = 0u;
    if (r_min == OBST_RANGE_NONE)
    {
        p_s->phase = (uint8_t)OBST_SCAN_DONE;
        return;
    }
    grow(p_s->fine, p_s->n_fine, nearest, r_min, &lo, &hi);
    if (b_right && (lo > 0u))
    {
        p_s->phase = (uint8_t)OBST_SCAN_REFINE_R;
        p_s->hit   = p_s->fine[lo].angle_deg;
        p_s->miss  = p_s->fine[lo - 1u].angle_deg;
    }
    else if (!b_right && ((hi + 1u) < p_s->n_fine))
    {
        p_s->phase = (uint8_t)OBST_SCAN_REFINE_L;
        p_s->hit   = p_s->fine[hi].angle_deg;
        p_s->miss  = p_s->fine[hi + 1u].angle_deg;
    }
    else
    {
        /* edge at the window limit: nothing to refine on this side */
        if (b_right)
        {
            start_refine(p_s, false);
            return;
        }
        p_s->phase = (uint8_t)OBST_SCAN_DONE;
        return;
    }
    p_s->angle = (int16_t)((p_s->hit + p_s->miss) / 2);
    if ((p_s->angle == p_s->hit) || (p_s->angle == p_s->miss))
    {
        p_s->iter = OBST_REFINE_ITERS;      /* already 1 deg apart */
    }
}

void obst_scan_start(obst_scan_t * p_s)
{
    p_s->phase    = (uint8_t)OBST_SCAN_COARSE;
    p_s->n_coarse = 0u;
    p_s->n_fine   = 0u;
    p_s->angle    = 30;
    p_s->iter     = 0u;
    p_s->r_min    = OBST_RANGE_NONE;
}

int16_t obst_scan_angle(obst_scan_t const * p_s)
{
    return p_s->angle;
}

bool obst_scan_done(obst_scan_t const * p_s)
{
    return (p_s->phase == (uint8_t)OBST_SCAN_DONE);
}

void obst_scan_feed(obst_scan_t * p_s, uint16_t r_mm)
{
    switch (p_s->phase)
    {
        case OBST_SCAN_COARSE:
            p_s->coarse[p_s->n_coarse].angle_deg = p_s->angle;
            p_s->coarse[p_s->n_coarse].r_mm      = r_mm;
            p_s->n_coarse++;
            if (p_s->n_coarse < 5u)
            {
                p_s->angle = (int16_t)(p_s->angle + 30);
            }
            else
            {
                uint8_t  i;
                uint16_t best   = OBST_RANGE_NONE;
                int16_t  centre = OBST_SERVO_CENTRE_DEG;

                for (i = 0u; i < 5u; i++)
                {
                    if (usable(&p_s->coarse[i]) && (p_s->coarse[i].r_mm < best))
                    {
                        best   = p_s->coarse[i].r_mm;
                        centre = p_s->coarse[i].angle_deg;
                    }
                }
                if (best == OBST_RANGE_NONE)
                {
                    p_s->phase = (uint8_t)OBST_SCAN_DONE;   /* nothing near: CONTINUE */
                }
                else
                {
                    p_s->phase   = (uint8_t)OBST_SCAN_FINE;
                    p_s->angle   = (int16_t)app_max_i32(centre - OBST_FINE_HALF_DEG, OBST_SCAN_MIN_DEG);
                    p_s->fine_to = (int16_t)app_min_i32(centre + OBST_FINE_HALF_DEG, OBST_SCAN_MAX_DEG);
                }
            }
            break;

        case OBST_SCAN_FINE:
            fine_insert(p_s, p_s->angle, r_mm);
            if ((p_s->angle + OBST_FINE_STEP_DEG) <= p_s->fine_to)
            {
                p_s->angle = (int16_t)(p_s->angle + OBST_FINE_STEP_DEG);
            }
            else
            {
                start_refine(p_s, true);
            }
            break;

        case OBST_SCAN_REFINE_R:
        case OBST_SCAN_REFINE_L:
        {
            obst_point_t pt;

            pt.angle_deg = p_s->angle;
            pt.r_mm      = r_mm;
            fine_insert(p_s, p_s->angle, r_mm);
            if (is_hit(&pt, p_s->r_min))
            {
                p_s->hit = p_s->angle;
            }
            else
            {
                p_s->miss = p_s->angle;
            }
            p_s->iter++;
            p_s->angle = (int16_t)((p_s->hit + p_s->miss) / 2);
            if ((p_s->iter >= OBST_REFINE_ITERS) || (p_s->angle == p_s->hit) || (p_s->angle == p_s->miss))
            {
                if (p_s->phase == (uint8_t)OBST_SCAN_REFINE_R)
                {
                    start_refine(p_s, false);
                }
                else
                {
                    p_s->phase = (uint8_t)OBST_SCAN_DONE;
                }
            }
            break;
        }

        default:
            break;
    }
}

void obst_scan_profile(obst_scan_t const * p_s, obst_profile_t * p_out)
{
    obst_profile(p_s->fine, p_s->n_fine, p_s->coarse, p_s->n_coarse, p_out);
}

void obst_scan_profile_detected(obst_scan_t const * p_s, uint16_t r_det_mm, uint16_t back_mm,
                                obst_profile_t * p_out)
{
    obst_profile_detected(p_s->fine, p_s->n_fine, p_s->coarse, p_s->n_coarse, r_det_mm, back_mm, p_out);
}

/* ---- edge scan (4 Nov) ------------------------------------------------ */
bool obst_edge_on_face(int32_t phi_cdeg, uint16_t r_mm, uint16_t face_mm)
{
    /* forward distance of the echo from the axle vs the face plane */
    int32_t const fwd = (int32_t)((((int64_t)OBST_US_AHEAD_MM + r_mm) * app_cos_q15(phi_cdeg)) / 32768);

    return (r_mm != OBST_RANGE_NONE) && (r_mm <= OBST_MAX_RANGE_MM) &&
           (fwd <= ((int32_t)OBST_US_AHEAD_MM + (int32_t)face_mm + OBST_EDGE_BAND_MM));
}

int32_t obst_edge_lat(int32_t phi_cdeg)
{
    return (int32_t)(((int64_t)OBST_US_AHEAD_MM * app_sin_q15(phi_cdeg)) / 32768);
}

int32_t obst_edge_wide_lat(int32_t phi_cdeg, uint16_t r_mm, int32_t side)
{
    int32_t const psi = phi_cdeg - (side * OBST_HALF_BEAM_CDEG);

    return (int32_t)((((int64_t)OBST_US_AHEAD_MM * app_sin_q15(phi_cdeg)) +
                      ((int64_t)r_mm * app_sin_q15(psi))) / 32768);
}

static int32_t width_lat(int32_t phi_cdeg, uint16_t r_mm, int32_t side)
{
    int32_t const psi = phi_cdeg - (side * OBST_WIDTH_BEAM_CDEG);

    return (int32_t)((((int64_t)OBST_US_AHEAD_MM * app_sin_q15(phi_cdeg)) +
                      ((int64_t)r_mm * app_sin_q15(psi))) / 32768);
}

uint16_t obst_edge_width(int32_t phi_l_cdeg, uint16_t r_l_mm, int32_t phi_r_cdeg, uint16_t r_r_mm)
{
    return (uint16_t)app_clamp_i32(width_lat(phi_l_cdeg, r_l_mm, 1) - width_lat(phi_r_cdeg, r_r_mm, -1),
                                   0, 0xFFFF);
}

bool obst_edge_past_max(int32_t lat_mm, int32_t side)
{
    return ((side * lat_mm) > OBST_WIDTH_MAX_MM);
}

void obst_edge_plan(obst_profile_t * p, bool b_has_l, int32_t lat_l, bool b_has_r, int32_t lat_r)
{
    int32_t const extra = OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM;
    bool          b_left;

    bool const b_servo = p->b_valid && (p->w_mm > 0u);   /* the servo scan saw a real face */

    if (!b_has_l && !b_has_r)
    {
        /* 6 Nov: no echo on either side: plan from the servo scan's edges
           with the same side rule below (keeping the servo scan's own
           plan chose sides by other rules: left 35 cm when right was
           the short way) */
        if (!p->b_valid)
        {
            return;
        }
        lat_l = p->plan_l_mm;
        lat_r = p->plan_r_mm;
    }
    /* 4 Nov: a side with no echo at all is not proof that the obstacle runs
       on that way (a centred box got 0 echoes on the left and the car went
       right).  Take the servo scan's edge for it when that scan saw a real
       face, otherwise as far as the largest obstacle from the seen edge. */
    if (!b_has_l)
    {
        lat_l = b_servo ? app_max_i32(p->plan_l_mm, lat_r) : (lat_r + OBST_WIDTH_MAX_MM);
    }
    if (!b_has_r)
    {
        lat_r = b_servo ? app_min_i32(p->plan_r_mm, lat_l) : (lat_l - OBST_WIDTH_MAX_MM);
    }
    if (lat_l < lat_r)
    {
        int32_t const mid = (lat_l + lat_r) / 2;

        lat_l = mid;
        lat_r = mid;
    }
    p->b_valid      = true;
    p->b_sure       = b_has_l && b_has_r;
    p->left_lat_mm  = (int16_t)lat_l;
    p->right_lat_mm = (int16_t)lat_r;
    p->plan_l_mm    = (int16_t)lat_l;
    p->plan_r_mm    = (int16_t)lat_r;
    p->w_mm         = (uint16_t)(lat_l - lat_r);

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
    p->plan         = b_left ? (uint8_t)OBST_PLAN_LEFT : (uint8_t)OBST_PLAN_RIGHT;
    p->deviation_mm = (int16_t)(b_left ? (lat_l + extra) : -(-lat_r + extra));
}

void obst_edge_plan_min(obst_profile_t * p, bool b_has_l, int32_t min_l, bool b_has_r, int32_t min_r)
{
    int32_t const extra = OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM;

    if ((p->plan == (uint8_t)OBST_PLAN_LEFT) && b_has_l)
    {
        p->deviation_mm = (int16_t)app_max_i32(min_l + extra, extra);
    }
    else if ((p->plan == (uint8_t)OBST_PLAN_RIGHT) && b_has_r)
    {
        p->deviation_mm = (int16_t)-app_max_i32(-min_r + extra, extra);
    }
    else
    {
        /* unseen side or no bypass: the plan's offset stands */
    }
}

/* ---- bypass ----------------------------------------------------------- */
uint8_t obst_bypass_plan(obst_profile_t const * p_prof, uint8_t side, obst_step_t * p_steps)
{
    int32_t const s     = (side == (uint8_t)OBST_PLAN_LEFT) ? 1 : -1;
    int32_t const dev   = app_abs_i32(p_prof->deviation_mm);
    int32_t const sin_q = app_sin_q15(OBST_TURN_OUT_DDEG * 10);
    int32_t const cos_q = app_cos_q15(OBST_TURN_OUT_DDEG * 10);
    int32_t const d1    = (int32_t)(((int64_t)dev * 32768) / sin_q);
    int32_t const fwd   = (int32_t)(((int64_t)d1 * cos_q) / 32768);
    int32_t const room  = (int32_t)p_prof->closest_mm - OBST_MARGIN_MM;
    /* 7 Nov: the step-out leg may go OBST_OUT_SPARE further out */
    int32_t const fwd_r = (int32_t)(((int64_t)(dev + OBST_OUT_SPARE_MM) * cos_q) / sin_q);
    int32_t       pass;
    int32_t       back = 0;
    int32_t       depth;
    uint8_t       n = 0u;

    /* Not enough room to get sideways before reaching the face: back up. */
    if (fwd_r > room)
    {
        back             = fwd_r - room + 20;
        p_steps[n].kind  = (uint8_t)OBST_STEP_MOVE;
        p_steps[n].value = (int16_t)-back;
        n++;
    }
    p_steps[n].kind = (uint8_t)OBST_STEP_TURN; p_steps[n].value = (int16_t)(s * OBST_TURN_OUT_DDEG);  n++;
    p_steps[n].kind = (uint8_t)OBST_STEP_OUT;  p_steps[n].value = (int16_t)d1;                        n++;
    p_steps[n].kind = (uint8_t)OBST_STEP_TURN; p_steps[n].value = (int16_t)(-s * OBST_TURN_OUT_DDEG); n++;

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
    depth = ((p_prof->w_mm > 0u) && (p_prof->w_mm <= OBST_WIDTH_MAX_MM)) ? (int32_t)p_prof->w_mm
                                                                       : OBST_WIDTH_MAX_MM;
    pass = app_max_i32((int32_t)p_prof->closest_mm + back - fwd, 0) + depth + OBST_ROBOT_LENGTH_MM +
           OBST_MARGIN_MM;
    p_steps[n].kind = (uint8_t)OBST_STEP_PASS; p_steps[n].value = (int16_t)app_min_i32(pass, OBST_PASS_MAX_MM); n++;

    /* 3 Nov: back in at the same angle until a line sensor touches the
       line (not a fixed distance: the turns slip), then OBST_IR_AHEAD_MM
       on so the axle is on the line, then (4 Nov) spin until the line is
       centred under the sensors, wherever it runs. */
    p_steps[n].kind = (uint8_t)OBST_STEP_TURN; p_steps[n].value = (int16_t)(-s * OBST_TURN_OUT_DDEG); n++;
    p_steps[n].kind  = (uint8_t)OBST_STEP_JOIN;
    p_steps[n].value = (int16_t)(((d1 * OBST_JOIN_MAX_PERMILLE) / 1000) + OBST_JOIN_EXTRA_MM);         n++;
    p_steps[n].kind = (uint8_t)OBST_STEP_MOVE; p_steps[n].value = (int16_t)OBST_IR_AHEAD_MM;          n++;
    p_steps[n].kind  = (uint8_t)OBST_STEP_LINESCAN;
    p_steps[n].value = (int16_t)(s * (OBST_TURN_OUT_DDEG + OBST_LINESCAN_SPARE_DDEG));               n++;
    return n;
}

/* ---- step-out leg (7 Nov) -------------------------------------------- */
#define OUT_EXTRA_MM    (OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM)
#define OUT_SHAFT_MM    (OBST_US_AHEAD_MM - OBST_SERVO_ARM_MM)  /* servo shaft ahead of the axle */

void obst_out_start(obst_out_t * p_o, int32_t min_lat)
{
    p_o->min_lat  = min_lat;
    p_o->seen_lat = 0;
    p_o->b_seen   = false;
    p_o->pings    = 0u;
}

int32_t obst_out_expect_mm(int32_t face_mm, int32_t axle_fwd_mm, int32_t theta_cdeg)
{
    /* the sensor: the shaft OUT_SHAFT ahead of the axle along the car,
       then the arm along the line */
    int32_t const sensor_fwd = axle_fwd_mm + (int32_t)(((int64_t)OUT_SHAFT_MM * app_cos_q15(theta_cdeg)) / 32768) +
                               OBST_SERVO_ARM_MM;

    return face_mm - sensor_fwd;
}

void obst_out_ping(obst_out_t * p_o, int32_t axle_lat, int32_t axle_fwd, int32_t theta_cdeg,
                   uint16_t r_mm, int32_t face_mm)
{
    int32_t const expect = obst_out_expect_mm(face_mm, axle_fwd, theta_cdeg);

    p_o->pings = (uint8_t)app_min_i32((int32_t)p_o->pings + 1, 255);
    if ((r_mm != OBST_RANGE_NONE) && ((int32_t)r_mm <= (expect + OBST_DEPTH_TOL_MM)))
    {
        int32_t const lat = axle_lat + (int32_t)(((int64_t)OUT_SHAFT_MM * app_sin_q15(theta_cdeg)) / 32768);

        if (!p_o->b_seen || (lat > p_o->seen_lat))
        {
            p_o->seen_lat = lat;
        }
        p_o->b_seen = true;
    }
}

int32_t obst_out_target(obst_out_t const * p_o)
{
    int32_t t = p_o->min_lat;

    if (p_o->b_seen)
    {
        t = app_max_i32(t, p_o->seen_lat + OUT_EXTRA_MM);
    }
    return app_min_i32(t, OBST_WIDTH_MAX_MM + OUT_EXTRA_MM);
}

bool obst_out_done(obst_out_t const * p_o, int32_t axle_lat)
{
    return ((p_o->pings >= OBST_OUT_MIN_PINGS) && (axle_lat >= obst_out_target(p_o))) ||
           (axle_lat >= (OBST_WIDTH_MAX_MM + OUT_EXTRA_MM));
}

bool obst_out_blocked(int32_t face_mm, int32_t axle_fwd_mm, int32_t theta_cdeg)
{
    /* the front corner on the line's side sits half a width further on */
    int32_t const corner = (int32_t)(((int64_t)(OBST_ROBOT_WIDTH_MM / 2) * app_sin_q15(theta_cdeg)) / 32768);

    return obst_out_expect_mm(face_mm, axle_fwd_mm, theta_cdeg) <= (OBST_MARGIN_MM + corner);
}

uint16_t obst_probe_face(uint16_t r_mm, uint16_t expect_mm)
{
    if ((r_mm == OBST_RANGE_NONE) ||
        (app_abs_i32((int32_t)r_mm - (int32_t)expect_mm) > OBST_PROBE_FACE_TOL_MM))
    {
        return expect_mm;
    }
    return r_mm;
}

void obst_probe_start(obst_probe_t * p_p)
{
    p_p->b_seen = false;
    p_p->lo     = 0;
    p_p->hi     = 0;
    p_p->off    = 0u;
}

int32_t obst_probe_sensor_lat(int32_t axle_lat, int32_t theta_cdeg)
{
    return axle_lat + (int32_t)(((int64_t)OUT_SHAFT_MM * app_sin_q15(theta_cdeg)) / 32768);
}

bool obst_probe_ping(obst_probe_t * p_p, int32_t sensor_lat, uint16_t r_mm, int32_t expect_mm)
{
    bool const b_face = (r_mm != OBST_RANGE_NONE) && ((int32_t)r_mm <= (expect_mm + OBST_DEPTH_TOL_MM));

    if (b_face)
    {
        if (!p_p->b_seen)
        {
            p_p->lo = sensor_lat;
            p_p->hi = sensor_lat;
        }
        p_p->lo     = app_min_i32(p_p->lo, sensor_lat);
        p_p->hi     = app_max_i32(p_p->hi, sensor_lat);
        p_p->b_seen = true;
        p_p->off    = 0u;
    }
    else
    {
        p_p->off = (uint8_t)app_min_i32((int32_t)p_p->off + 1, 255);
    }
    return b_face;
}

bool obst_probe_edges(obst_probe_t const * p_p, int32_t * p_left, int32_t * p_right)
{
    int32_t l;
    int32_t r;

    if (!p_p->b_seen)
    {
        return false;
    }
    l = p_p->hi - OBST_PROBE_EDGE_OVER_MM;
    r = p_p->lo + OBST_PROBE_EDGE_OVER_MM;
    if (l < r)
    {
        l = (p_p->hi + p_p->lo) / 2;    /* narrower than the run-on: a post */
        r = l;
    }
    *p_left  = l;
    *p_right = r;
    return true;
}

int32_t obst_out_servo_deg(int32_t theta_cdeg, int32_t side)
{
    return app_clamp_i32(OBST_SERVO_CENTRE_DEG - ((side * theta_cdeg) / 100), OBST_SCAN_MIN_DEG, OBST_SCAN_MAX_DEG);
}

int32_t obst_cross_track_mm(int32_t x, int32_t y, int32_t x0, int32_t y0, int32_t h0_cdeg)
{
    int64_t const dx = (int64_t)x - x0;
    int64_t const dy = (int64_t)y - y0;

    int64_t const v  = (-dx * app_sin_q15(h0_cdeg)) + (dy * app_cos_q15(h0_cdeg));

    return (int32_t)((v + ((v >= 0) ? 16384 : -16384)) / 32768);     /* rounded */
}

bool obst_line_accept(int32_t x, int32_t y, int32_t h_cdeg,
                      int32_t x0, int32_t y0, int32_t h0_cdeg)
{
    int32_t const ct = obst_cross_track_mm(x, y, x0, y0, h0_cdeg);
    int32_t const dh = app_wrap_cdeg(h_cdeg - h0_cdeg);

    return (app_abs_i32(ct) <= OBST_SEEK_LAT_MM) && (app_abs_i32(dh) <= OBST_SEEK_HEADING_CDEG);
}

bool obst_hump_mask(obst_mask_t * p_m, bool b_pitch_valid, int32_t pitch_cdeg,
                    int32_t dist_mm, uint32_t now_ms)
{
    bool const b_pitched = b_pitch_valid && (app_abs_i32(pitch_cdeg) > OBST_HUMP_MASK_CDEG);

    if (!b_pitched)
    {
        p_m->b_active = false;
        p_m->start_ms = 0u;             /* re-arm for the next hump */
        return false;
    }
    if (!p_m->b_active && (p_m->start_ms == 0u))
    {
        p_m->b_active      = true;
        p_m->start_ms      = (now_ms == 0u) ? 1u : now_ms;
        p_m->start_dist_mm = dist_mm;
    }
    if (p_m->b_active &&
        ((app_elapsed_ms(now_ms, p_m->start_ms) >= OBST_HUMP_MASK_MS) ||
         (app_abs_i32(dist_mm - p_m->start_dist_mm) >= OBST_HUMP_MASK_MM)))
    {
        p_m->b_active = false;          /* lapsed: stays off until level again */
    }
    return p_m->b_active;
}
