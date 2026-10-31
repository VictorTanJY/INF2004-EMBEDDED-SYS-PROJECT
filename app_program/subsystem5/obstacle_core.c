/*
 *  obstacle_core.c
 *  Subsystem 5 -- obstacle geometry (pure).  See obstacle_core.h.
 */

#include "obstacle_core.h"
#include "mission_cfg.h"
#include "../app_types.h"
#include "../app_math.h"

#define OBST_NEAR_EXTRA_MM      300     /* other objects this close matter for clearance */
#define OBST_ASSUMED_DEPTH_MM   200     /* PASS fallback when the side ping never sees it */

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

static int32_t lateral_mm(int32_t angle_cdeg, uint16_t r_mm)
{
    return -(int32_t)(((int64_t)r_mm * app_cos_q15(angle_cdeg)) / 32768);
}

static bool usable(obst_point_t const * p)
{
    return (p->r_mm != OBST_RANGE_NONE) && (p->r_mm <= OBST_MAX_RANGE_MM);
}

void obst_profile(obst_point_t const * p_fine, uint8_t n_fine,
                  obst_point_t const * p_coarse, uint8_t n_coarse,
                  obst_profile_t * p_out)
{
    int32_t const corridor = (OBST_ROBOT_WIDTH_MM / 2) + OBST_MARGIN_MM;
    int32_t const need     = OBST_ROBOT_WIDTH_MM + (2 * OBST_MARGIN_MM);
    uint8_t       i;
    uint8_t       nearest = 0u;
    uint8_t       lo;
    uint8_t       hi;
    uint16_t      r_min = OBST_RANGE_NONE;
    int32_t       a_left;
    int32_t       a_right;
    int32_t       lat_l;
    int32_t       lat_r;
    int32_t       clr_l = OBST_CLEAR_MAX_MM;
    int32_t       clr_r = OBST_CLEAR_MAX_MM;

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

    for (i = 0u; i < n_fine; i++)
    {
        if (usable(&p_fine[i]) && (p_fine[i].r_mm < r_min))
        {
            r_min   = p_fine[i].r_mm;
            nearest = i;
        }
    }
    if (r_min == OBST_RANGE_NONE)
    {
        return;                         /* nothing in front: CONTINUE */
    }

    /* The obstacle = the contiguous run of points around the nearest one
       that are not much further away than it (same object). */
    lo = nearest;
    hi = nearest;
    while ((lo > 0u) && usable(&p_fine[lo - 1u]) &&
           (p_fine[lo - 1u].r_mm <= (uint32_t)r_min + OBST_DEPTH_TOL_MM))
    {
        lo--;
    }
    while (((hi + 1u) < n_fine) && usable(&p_fine[hi + 1u]) &&
           (p_fine[hi + 1u].r_mm <= (uint32_t)r_min + OBST_DEPTH_TOL_MM))
    {
        hi++;
    }

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
    lat_l = lateral_mm(a_left, p_fine[hi].r_mm);
    lat_r = lateral_mm(a_right, p_fine[lo].r_mm);

    /* Clearance: nearby points outside the obstacle, from both scans. */
    for (i = 0u; i < (uint8_t)(n_fine + n_coarse); i++)
    {
        obst_point_t const * p = (i < n_fine) ? &p_fine[i] : &p_coarse[i - n_fine];
        int32_t              lat;

        if (((i >= lo) && (i <= hi)) || !usable(p) ||
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
        if (lat > lat_l)
        {
            clr_l = app_min_i32(clr_l, lat - lat_l);
        }
        else if (lat < lat_r)
        {
            clr_r = app_min_i32(clr_r, lat_r - lat);
        }
        else
        {
            /* inside the obstacle's lateral span (behind it) */
        }
    }

    p_out->b_valid       = true;
    p_out->closest_mm    = r_min;
    p_out->nearest_angle = p_fine[nearest].angle_deg;
    p_out->left_lat_mm   = (int16_t)lat_l;
    p_out->right_lat_mm  = (int16_t)lat_r;
    p_out->w_mm          = (uint16_t)app_max_i32(lat_l - lat_r, 0);
    p_out->clr_l_mm      = (uint16_t)clr_l;
    p_out->clr_r_mm      = (uint16_t)clr_r;

    /* PLAN (FR11). */
    if ((lat_r > corridor) || (lat_l < -corridor))
    {
        p_out->plan = (uint8_t)OBST_PLAN_CONTINUE;      /* beside the path */
    }
    else
    {
        int32_t const dev_l  = lat_l + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM;
        int32_t const dev_r  = -lat_r + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM;
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

/* ---- scan sequencer ---------------------------------------------------- */
static bool is_hit(obst_point_t const * p, uint16_t r_min)
{
    return usable(p) && (p->r_mm <= ((uint32_t)r_min + OBST_DEPTH_TOL_MM));
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
    lo = nearest;
    hi = nearest;
    while ((lo > 0u) && is_hit(&p_s->fine[lo - 1u], r_min))
    {
        lo--;
    }
    while (((hi + 1u) < p_s->n_fine) && is_hit(&p_s->fine[hi + 1u], r_min))
    {
        hi++;
    }
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
    int32_t       pass;
    uint8_t       n = 0u;

    /* Not enough room to get sideways before reaching the face: back up. */
    if (fwd > room)
    {
        p_steps[n].kind  = (uint8_t)OBST_STEP_MOVE;
        p_steps[n].value = (int16_t)-(fwd - room + 20);
        n++;
    }
    p_steps[n].kind = (uint8_t)OBST_STEP_TURN; p_steps[n].value = (int16_t)(s * OBST_TURN_OUT_DDEG);  n++;
    p_steps[n].kind = (uint8_t)OBST_STEP_MOVE; p_steps[n].value = (int16_t)d1;                        n++;
    p_steps[n].kind = (uint8_t)OBST_STEP_TURN; p_steps[n].value = (int16_t)(-s * OBST_TURN_OUT_DDEG); n++;

    /* PASS: the task ends it when the side ping has seen the obstacle and
       lost it again; this value is the fallback distance. */
    pass = app_max_i32((int32_t)p_prof->closest_mm - fwd, 0) + OBST_ASSUMED_DEPTH_MM + OBST_ROBOT_LENGTH_MM;
    p_steps[n].kind = (uint8_t)OBST_STEP_PASS; p_steps[n].value = (int16_t)app_min_i32(pass, OBST_PASS_MAX_MM); n++;

    p_steps[n].kind = (uint8_t)OBST_STEP_TURN; p_steps[n].value = (int16_t)(-s * OBST_TURN_OUT_DDEG); n++;
    p_steps[n].kind = (uint8_t)OBST_STEP_MOVE; p_steps[n].value = (int16_t)d1;                        n++;
    p_steps[n].kind = (uint8_t)OBST_STEP_TURN; p_steps[n].value = (int16_t)(s * OBST_TURN_OUT_DDEG);  n++;
    return n;
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
