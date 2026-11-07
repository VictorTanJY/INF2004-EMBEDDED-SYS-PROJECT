/*
 *  obstacle_task.c
 *  Subsystem 5 -- ObstacleTask (priority 9, core 1), Week 6 Section 6.4.
 *  Owner: Buddy 5 (Ryan).
 *
 *  Blocks only on flg_obst (TWF_ORW | TWF_BITCLR, 100 ms timeout so it
 *  heartbeats even when idle): RANGE every 60 ms from range_cyc, ECHO from
 *  echo_isr, REQ after each mbf_scan request.  One ping per 60 ms slot.
 *
 *  MONITOR   servo 90 deg, median of 3: < slow_mm SLOW_DOWN, < stop_mm
 *            OBSTACLE (both masked on a hump), < 8 cm TOO_CLOSE (never masked).
 *  PROFILE   coarse 30..150 by 30, fine +-40 by 5 around the nearest echo,
 *            two bisection pings per edge -> width, edges, clearances, plan.
 *            4 Nov, for a bypass: EDGE_SCAN, sensor straight ahead, the car
 *            turns 5 deg at a time to each side until the echo leaves the
 *            face; the edges from the car's heading re-plan the bypass
 *            -> PROFILE_DONE(plan, width).  No clearance: reverse 100 mm and
 *            rescan (twice), then BLOCKED: rescan every 2 s, 5 times; after
 *            10 s BLOCKED_TIMEOUT.
 *  BYPASS    turn out 45, offset, turn parallel, PASS (servo looking at the
 *            obstacle until it has been seen and passed), turn in 45, JOIN
 *            (on until a line sensor touches the line), OBST_IR_AHEAD_MM on,
 *            LINESCAN (spin slowly until the line is centred under the
 *            sensors) -> BYPASS_DONE.  3 Nov: bypass and sweep
 *            turns are scaled by the turn slip measured at calibration.  Commands use AVOID authority;
 *            completion is read from last_done_seq in the odometry snapshot.
 *  RECOVER   creep forward (<= 0.5 m) until a line sensor sees black within
 *            +-60 mm of the original path and +-20 deg of its heading, align
 *            -> RECOVERY_DONE; else sweep +-45 deg on the spot and take any
 *            line -> RECOVERY_DONE, or RECOVERY_FAILED.
 *  3 Nov (instructor's guidance): every scan angle is the median of
 *  OBST_SCAN_PINGS pings after the servo settles; an unsure profile (see
 *  mission_cfg.h) backs off OBST_SECOND_LOOK_MM and scans once more.
 */

#include "subsystem5.h"
#include "mission_cfg.h"
#include "obstacle_core.h"
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../app_types.h"
#include "../app_math.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define OBST_WAIT_TIMEOUT_MS    100
#define OBST_SIDE_SERVO_DEG     75      /* PASS: look 75 deg to the obstacle's side */
#define OBST_HOLD_K             2       /* heading hold: mrad/s per cdeg of error */
#define OBST_HOLD_MAX           1500
#define OBST_ALIGN_TOL_CDEG     300
#define OBST_CONTINUE_IGNORE_MM 300     /* after CONTINUE: no new OBSTACLE for this far */
#define OBST_TOO_CLOSE_RESET_MM 120
/* Event-history diagnostics (3 Nov) */
#define OBST_LOG_PLAN           39u     /* bypass plan: offset mm (+ left) << 16 | pass mm */
#define OBST_LOG_PASS           40u     /* pass leg: driven mm | side ping saw it << 16 */
#define OBST_LOG_JOIN           44u     /* back in: line touched after mm, 0 = not found */
#define OBST_LOG_LINESCAN       46u     /* turned onto the line after 0.1 deg (< 0: one sensor only), 0 = not found */
#define OBST_LOG_EDGES          47u     /* scan edges, mm (+ left): left << 16 | right (int16 each) */
#define OBST_LOG_EDGES_CAR      48u     /* the same from the edge scan (car turns) */
#define OBST_LOG_EDGE_SIDES     49u     /* edge scan: hits L << 24 | last deg L << 16 | hits R << 8 | last deg R */
#define OBST_LOG_EDGE_FACE      50u     /* edge scan: face mm straight ahead << 16 | first left ping mm */
#define OBST_LOG_EDGE_RAW_L     71u     /* edge scan, last left echo: heading 0.01 deg << 16 | range mm */
#define OBST_LOG_EDGE_RAW_R     72u     /* the same on the right (heading < 0)       */
#define OBST_LOG_PROBE          74u     /* width probe: left edge mm << 16 | right edge (int16 each, + left); 0x7FFF7FFF none */
#define OBST_LOG_OUT            60u     /* step-out: face edge seen at mm (0xFFFF none) << 16 | stepped out mm */

/* line scan phases */
#define LS_PH_SWEEP             0u
#define LS_PH_ALIGN             1u

/* edge scan phases */
#define EDGE_PH_TURN            0u
#define EDGE_PH_SETTLE          1u
#define EDGE_PH_PING            2u
#define EDGE_PH_RETURN          3u
/* width probe (7 Nov) */
#define PB_FACE                 0u      /* servo straight, median ping of the face */
#define PB_MOVE                 1u      /* to OBST_PROBE_FACE_MM from it            */
#define PB_TURN                 2u      /* turn OBST_PROBE_DDEG left                */
#define PB_SERVO                3u      /* servo back along the line                */
#define PB_LEFT                 4u      /* forward: past the left edge              */
#define PB_RIGHT                5u      /* back: past the right edge                */
#define PB_BACK                 6u      /* forward to where it turned               */
#define PB_UNTURN               7u      /* turn back to the line's heading          */
#define OBST_GAIN_UNIT          1000

static obst_scan_t    g_scan;
static obst_profile_t g_prof;
static obst_step_t    g_steps[OBST_BYPASS_MAX_STEPS];

typedef struct
{
    uint8_t  state;             /* obst_state_t */
    /* servo */
    int32_t  servo_deg;
    uint32_t settle_until_ms;
    /* ping */
    bool     b_ping_out;
    uint16_t win[3];
    uint8_t  win_n;
    uint16_t d_mm;
    uint16_t no_echo;
    bool     b_no_echo_set;
    /* monitor */
    bool     b_slow_sent;
    bool     b_obst_sent;
    bool     b_too_close;
    int32_t  ignore_until_mm;
    obst_mask_t mask;
    /* pose at the obstacle */
    bool     b_have_pose;
    int32_t  x0;
    int32_t  y0;
    int32_t  h0;
    /* planning / blocked */
    uint8_t  retries;
    uint8_t  rescans;
    bool     b_blocked;         /* in the BLOCKED wait (rescans keep it set) */
    uint32_t blocked_start_ms;
    uint32_t next_rescan_ms;
    /* motion steps */
    uint8_t  n_steps;
    uint8_t  step_i;
    uint8_t  side;              /* OBST_PLAN_LEFT / RIGHT */
    uint8_t  seq;
    bool     b_wait_motion;
    uint8_t  wait_fail_count;
    /* pass / seek */
    int32_t  phase_start_mm;
    int32_t  last_seen_mm;
    bool     b_seen;
    int16_t  pass_limit_mm;
    uint8_t  drive_seq;
    uint32_t pings;
    /* 3 Nov */
    uint16_t sw[OBST_SCAN_PINGS];   /* pings at the current scan angle */
    uint8_t  sw_n;
    bool     b_second_look;     /* this obstacle was already scanned twice */
    uint8_t  sweep_i;           /* SEEK_SWEEP step */
    int32_t  join_h;            /* JOIN: heading held while driving back in */
    bool     b_joined;          /* the line was found: accept it in SEEK    */
    uint8_t  scan_pass;         /* LINESCAN: 0 not started, 1 first sweep, 2 second */
    uint8_t  scan_ph;           /* LINESCAN: LS_PH_* */
    uint8_t  ls_seen[2];        /* per sensor: 0 not yet, 1 on, 2 on and off */
    bool     b_ls_prev[2];      /* per sensor: on the line at the last slot  */
    int32_t  ls_on[2];          /* heading (from scan_h0) it went on, 0.01 deg */
    int32_t  ls_off[2];         /* ... and off */
    int32_t  scan_target;       /* the line's heading: midway between them   */
    uint32_t scan_until_ms;     /* settle before the turn to it              */
    bool     b_scan_turned;     /* the turn to it has been sent              */
    uint8_t  ls_part_q;         /* 7 Nov: best one-sensor crossing: 0 none,
                                   1 went on only, 2 on and off             */
    int32_t  ls_part_h;         /* ... its heading (from scan_h0)             */
    bool     b_scan_part;       /* turning to a one-sensor crossing           */
    /* edge scan (4 Nov) */
    int8_t   edge_side;         /* +1 left, -1 right */
    uint8_t  edge_ph;           /* EDGE_PH_* */
    uint8_t  edge_miss;
    int32_t  edge_h0;           /* encoder heading where the car stopped */
    uint32_t edge_until_ms;
    bool     b_edge_has[2];     /* [0] left, [1] right */
    int32_t  edge_lat[2];       /* wide edge (side choice)           */
    int32_t  edge_min[2];       /* sensor-position edge (7 Nov): step-out minimum */
    int32_t  edge_phi[2];       /* 7 Nov: heading of the last echo, 0.01 deg  */
    uint16_t edge_r[2];         /* ... and its range                          */
    int32_t  edge_lo;           /* refine: last echo heading                  */
    int32_t  edge_hi;           /* refine: first miss heading                 */
    uint8_t  edge_ref;          /* refine iteration, 0 = coarse steps         */
    int16_t  edge_step;         /* next turn, 0.1 deg (+ left)                */
    uint8_t  edge_hits[2];      /* on-face pings per side (log)  */
    uint8_t  edge_deg[2];       /* last heading reached, deg (log) */
    uint16_t edge_face;         /* face range straight ahead, measured first */
    uint16_t edge_first_l;      /* first ping on the left (log)             */
    bool     b_edge_face;       /* edge_face measured */
    int32_t  scan_h0;           /* LINESCAN: heading at the start           */
    uint16_t det_mm;            /* range when the monitor saw the obstacle  */
    int32_t  back_mm;           /* backed off since then (second look, reverse) */
    /* width probe (7 Nov) */
    uint8_t  pb_ph;             /* PB_*                                       */
    bool     b_pb_moved;        /* the approach move has been made            */
    bool     b_pb_seen;         /* face seen on this pass                     */
    int32_t  pb_h1;             /* heading held while side-stepping           */
    int32_t  pb_face_mm;        /* face ahead of the axle before the turn     */
    int32_t  pb_lat;            /* axle sideways (+ left) at the last slot    */
    obst_probe_t probe;
    /* step-out leg (7 Nov) */
    obst_out_t out;
    int32_t  out_face_mm;       /* face ahead of the axle at the turn-out    */
    int32_t  out_h;             /* heading held while stepping out            */
} obst_ctx_t;

static obst_ctx_t g_o;

/* ------------------------------------------------------------------ */
static void servo_to(int32_t deg, uint32_t now_ms)
{
    int32_t const delta = app_abs_i32(deg - g_o.servo_deg);

    if (delta != 0)
    {
        drv_servo_set_deg(deg);
        g_o.servo_deg       = deg;
        g_o.settle_until_ms = now_ms + (uint32_t)((delta * OBST_SERVO_MS_PER_DEG) + OBST_SERVO_SETTLE_MS);
    }
}

static bool servo_settled(uint32_t now_ms)
{
    return ((int32_t)(now_ms - g_o.settle_until_ms) >= 0);
}

static void ping(void)
{
    drv_us_trigger();
    g_o.b_ping_out = true;
    g_o.pings++;
}

static void post(uint8_t type, uint16_t a, int32_t data)
{
    (void)mission_post(type, (uint8_t)APP_SRC_OBST_MISSION, a, data);
}

static void log_ev(uint16_t code, int32_t data)
{
    (void)comms_telem_post((uint8_t)COMMS_KIND_LOG, (uint8_t)APP_SRC_OBST_MISSION, code, data);
}

/* A real turn (0.1 deg) as the encoders must count it: x the turn slip
   gain measured in the calibration spin (line snapshot, 0 = not measured). */
static int32_t turn_gain(void)
{
    line_snap_t line;

    line_get_snap(&line);
    return (line.turn_gain_pm != 0u) ? (int32_t)line.turn_gain_pm : OBST_GAIN_UNIT;
}

/* Step-out leg: the car's real heading from the line toward side s,
   0.01 deg (encoder heading corrected by the measured turn slip). */
static int32_t out_theta(motion_odom_t const * p_odom, int32_t s)
{
    int32_t const raw = s * app_wrap_cdeg(p_odom->heading_cdeg - g_o.h0);

    return app_clamp_i32((raw * OBST_GAIN_UNIT) / turn_gain(), 0, 9000);
}

static int16_t slip_ddeg(int16_t ddeg)
{
    int32_t const gain = turn_gain();

    return (int16_t)app_clamp_i32(((int32_t)ddeg * gain) / OBST_GAIN_UNIT, -3600, 3600);
}

static void goto_state(uint8_t state)
{
    g_o.state         = state;
    g_o.b_wait_motion = false;
}

static void drive(int32_t v, int32_t target_h, motion_odom_t const * p_odom)
{
    int32_t const err = target_h - p_odom->heading_cdeg;
    int32_t const w   = app_clamp_i32(err * OBST_HOLD_K, -OBST_HOLD_MAX, OBST_HOLD_MAX);

    g_o.drive_seq++;
    (void)motion_drive((uint8_t)MOTION_SRC_AVOID, g_o.drive_seq, (int16_t)v, (int16_t)w);
}

static void stop_motion(void)
{
    (void)motion_stop((uint8_t)MOTION_SRC_AVOID, 0u);
}

static void send_step(uint8_t kind, int16_t value, motion_odom_t const * p_odom)
{
    uint16_t const amount = (uint16_t)app_abs_i32(value);

    g_o.seq++;
    g_o.wait_fail_count = p_odom->fail_count;
    g_o.b_wait_motion   = true;
    if (kind == (uint8_t)OBST_STEP_TURN)
    {
        if (value >= 0)
        {
            (void)motion_turn_left((uint8_t)MOTION_SRC_AVOID, g_o.seq, amount, OBST_BYPASS_TURN_DEG_S);
        }
        else
        {
            (void)motion_turn_right((uint8_t)MOTION_SRC_AVOID, g_o.seq, amount, OBST_BYPASS_TURN_DEG_S);
        }
    }
    else
    {
        if (value >= 0)
        {
            (void)motion_move_forward((uint8_t)MOTION_SRC_AVOID, g_o.seq, amount, OBST_BYPASS_SPEED_MM_S);
        }
        else
        {
            (void)motion_move_backward((uint8_t)MOTION_SRC_AVOID, g_o.seq, amount, OBST_BYPASS_SPEED_MM_S);
        }
    }
}

/* Median of the echoes that came back at one scan angle (OBST_RANGE_NONE
   only if none did).  3 Nov: a round bottle answered 1 ping in 3, and a
   median that counted misses as "far" threw that echo away. */
static uint16_t scan_median(void)
{
    uint16_t s[OBST_SCAN_PINGS];
    uint8_t  n = 0u;
    uint8_t  i;
    uint8_t  j;

    for (i = 0u; i < g_o.sw_n; i++)
    {
        if (g_o.sw[i] != OBST_RANGE_NONE)
        {
            s[n] = g_o.sw[i];
            for (j = n; (j > 0u) && (s[j - 1u] > s[j]); j--)
            {
                uint16_t const t = s[j - 1u];

                s[j - 1u] = s[j];
                s[j]      = t;
            }
            n++;
        }
    }
    return (n == 0u) ? OBST_RANGE_NONE : s[(n - 1u) / 2u];
}

/* One ping result at a scan angle: true once OBST_SCAN_PINGS are in
   (*p_r = their median). */
static bool scan_sample(uint16_t r, uint16_t * p_r)
{
    g_o.sw[g_o.sw_n] = r;
    g_o.sw_n++;
    if (g_o.sw_n < OBST_SCAN_PINGS)
    {
        return false;
    }
    *p_r     = scan_median();
    g_o.sw_n = 0u;
    return true;
}

static void send_turn(int16_t ddeg, uint16_t rate_deg_s, motion_odom_t const * p_odom)
{
    uint16_t const amount = (uint16_t)app_abs_i32(ddeg);

    g_o.seq++;
    g_o.wait_fail_count = p_odom->fail_count;
    g_o.b_wait_motion   = true;
    if (ddeg >= 0)
    {
        (void)motion_turn_left((uint8_t)MOTION_SRC_AVOID, g_o.seq, amount, rate_deg_s);
    }
    else
    {
        (void)motion_turn_right((uint8_t)MOTION_SRC_AVOID, g_o.seq, amount, rate_deg_s);
    }
}

/* 1 = done, -1 = failed, 0 = still running */
static int motion_result(motion_odom_t const * p_odom)
{
    if ((p_odom->last_done_seq == g_o.seq) && (p_odom->last_done_src == (uint8_t)MOTION_SRC_AVOID))
    {
        g_o.b_wait_motion = false;
        return 1;
    }
    if ((p_odom->fail_count != g_o.wait_fail_count) && (p_odom->last_fail_seq == g_o.seq))
    {
        g_o.b_wait_motion = false;
        return -1;
    }
    return 0;
}

static void store_pose(motion_odom_t const * p_odom)
{
    g_o.x0          = p_odom->x_mm;
    g_o.y0          = p_odom->y_mm;
    g_o.h0          = p_odom->heading_cdeg;
    g_o.b_have_pose = true;
}

static void start_scan(uint32_t now_ms)
{
    g_o.sw_n = 0u;
    obst_scan_start(&g_scan);
    goto_state((uint8_t)OBST_ST_COARSE_SCAN);
    servo_to(obst_scan_angle(&g_scan), now_ms);
}

static void publish_profile(void)
{
    obst_snap_t s;

    obst_get_snap(&s);
    s.valid        = g_prof.b_valid ? 1u : 0u;
    s.plan         = g_prof.plan;
    s.w_mm         = g_prof.w_mm;
    s.closest_mm   = g_prof.closest_mm;
    s.clr_l_mm     = g_prof.clr_l_mm;
    s.clr_r_mm     = g_prof.clr_r_mm;
    s.left_lat_mm  = g_prof.left_lat_mm;
    s.right_lat_mm = g_prof.right_lat_mm;
    obst_put_snap(&s);
}

/* The scan finished: decide (PLAN). */
static void plan(uint32_t now_ms, motion_odom_t const * p_odom)
{
    obst_profile_t const * const p_pr = &g_prof;

    obst_scan_profile_detected(&g_scan, g_o.det_mm, (uint16_t)app_clamp_i32(g_o.back_mm, 0, 1000), &g_prof);
    publish_profile();
    if (p_pr->b_valid)
    {
        log_ev(OBST_LOG_EDGES, (int32_t)(((uint32_t)(uint16_t)p_pr->left_lat_mm << 16) |
                                         (uint16_t)p_pr->right_lat_mm));
    }
    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);

    if (g_prof.b_valid && !g_prof.b_sure && !g_o.b_second_look)
    {
        /* unsure estimate: look again from a little further back once;
           the second profile plans with OBST_WIDTH_MAX if still unsure */
        g_o.b_second_look = true;
        goto_state((uint8_t)OBST_ST_REVERSE);
        g_o.back_mm += OBST_SECOND_LOOK_MM;
        send_step((uint8_t)OBST_STEP_MOVE, -OBST_SECOND_LOOK_MM, p_odom);
    }
    else if ((g_prof.plan == (uint8_t)OBST_PLAN_LEFT) || (g_prof.plan == (uint8_t)OBST_PLAN_RIGHT))
    {
        g_o.b_blocked     = false;
#if OBST_WIDTH_PROBE
        /* 7 Nov: measure the edges by side-stepping past them */
        goto_state((uint8_t)OBST_ST_WIDTH_PROBE);
        g_o.pb_ph         = PB_FACE;
        g_o.b_pb_moved    = false;
        g_o.sw_n          = 0u;
        g_o.edge_h0       = p_odom->heading_cdeg;
        g_o.edge_face     = g_prof.closest_mm;
        g_o.b_edge_face   = false;
        g_o.b_edge_has[0] = false;
        g_o.b_edge_has[1] = false;
        obst_probe_start(&g_o.probe);
        return;
#endif
        /* 4 Nov: measure the edges for the bypass by turning the car */
        goto_state((uint8_t)OBST_ST_EDGE_SCAN);
        g_o.edge_side     = 1;
        g_o.edge_ph       = EDGE_PH_SETTLE;     /* 4 Nov: first ping straight ahead for the face */
        g_o.edge_until_ms = now_ms;
        g_o.b_edge_face   = false;
        g_o.edge_face     = g_prof.closest_mm;
        g_o.edge_first_l  = 0u;
        g_o.edge_miss     = 0u;
        g_o.edge_ref      = 0u;
        g_o.edge_step     = (int16_t)OBST_EDGE_STEP_DDEG;
        g_o.edge_h0       = p_odom->heading_cdeg;
        g_o.b_edge_has[0] = false;
        g_o.b_edge_has[1] = false;
        g_o.edge_hits[0]  = 0u;
        g_o.edge_hits[1]  = 0u;
        g_o.edge_deg[0]   = 0u;
        g_o.edge_deg[1]   = 0u;
    }
    else if (g_prof.plan != (uint8_t)OBST_PLAN_BLOCKED)
    {
        g_o.b_blocked = false;
        if (g_prof.plan == (uint8_t)OBST_PLAN_CONTINUE)
        {
            g_o.ignore_until_mm = p_odom->dist_mm + OBST_CONTINUE_IGNORE_MM;
        }
        post((uint8_t)MISSION_EV_PROFILE_DONE, g_prof.plan, (int32_t)g_prof.w_mm);
        goto_state((uint8_t)OBST_ST_IDLE);
    }
    else if (g_o.retries < OBST_REVERSE_TRIES)
    {
        g_o.retries++;
        goto_state((uint8_t)OBST_ST_REVERSE);
        g_o.back_mm += OBST_REVERSE_MM;
        send_step((uint8_t)OBST_STEP_MOVE, -OBST_REVERSE_MM, p_odom);
    }
    else
    {
        if (!g_o.b_blocked)
        {
            g_o.b_blocked        = true;
            g_o.blocked_start_ms = now_ms;
            g_o.rescans          = 0u;
        }
        goto_state((uint8_t)OBST_ST_BLOCKED);
        g_o.next_rescan_ms = now_ms + OBST_BLOCKED_RESCAN_MS;
    }
}

/* Edge scan finished: re-plan from the measured edges and report. */
static void edge_finish(void)
{
    obst_edge_plan(&g_prof, g_o.b_edge_has[0], g_o.edge_lat[0], g_o.b_edge_has[1], g_o.edge_lat[1]);
    obst_edge_plan_min(&g_prof, g_o.b_edge_has[0], g_o.edge_min[0], g_o.b_edge_has[1], g_o.edge_min[1]);
    if (g_o.b_edge_has[0] && g_o.b_edge_has[1])
    {
        /* 7 Nov: the reported width, with the calibrated beam angle */
        g_prof.w_mm = obst_edge_width(g_o.edge_phi[0], g_o.edge_r[0], g_o.edge_phi[1], g_o.edge_r[1]);
    }
    log_ev(OBST_LOG_EDGE_RAW_L, g_o.b_edge_has[0] ? (int32_t)(((uint32_t)(uint16_t)g_o.edge_phi[0] << 16) | g_o.edge_r[0]) : 0);
    log_ev(OBST_LOG_EDGE_RAW_R, g_o.b_edge_has[1] ? (int32_t)(((uint32_t)(uint16_t)g_o.edge_phi[1] << 16) | g_o.edge_r[1]) : 0);
    publish_profile();
    log_ev(OBST_LOG_EDGE_FACE, (int32_t)(((uint32_t)g_o.edge_face << 16) | g_o.edge_first_l));
    log_ev(OBST_LOG_EDGE_SIDES, (int32_t)(((uint32_t)g_o.edge_hits[0] << 24) | ((uint32_t)g_o.edge_deg[0] << 16) |
                                          ((uint32_t)g_o.edge_hits[1] << 8) | (uint32_t)g_o.edge_deg[1]));
    log_ev(OBST_LOG_EDGES_CAR, (int32_t)(((uint32_t)(uint16_t)g_prof.left_lat_mm << 16) |
                                         (uint16_t)g_prof.right_lat_mm));
    post((uint8_t)MISSION_EV_PROFILE_DONE, g_prof.plan, (int32_t)g_prof.w_mm);
    goto_state((uint8_t)OBST_ST_IDLE);
}

/* Width probe finished (7 Nov): plan from the measured edges, report. */
static void probe_finish(uint32_t now_ms)
{
    int32_t l = 0;
    int32_t r = 0;
    bool const b_ok = obst_probe_edges(&g_o.probe, &l, &r);

    stop_motion();
    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
    if (b_ok)
    {
        obst_edge_plan(&g_prof, true, l, true, r);
        obst_edge_plan_min(&g_prof, true, l, true, r);
        g_prof.w_mm = (uint16_t)app_max_i32(l - r, 0);
        log_ev(OBST_LOG_PROBE, (int32_t)(((uint32_t)(uint16_t)l << 16) | (uint16_t)r));
    }
    else
    {
        obst_edge_plan(&g_prof, false, 0, false, 0);   /* the servo scan's plan */
        log_ev(OBST_LOG_PROBE, 0x7FFF7FFF);
    }
    /* the bypass plans from where the face is now */
    g_prof.closest_mm = g_o.edge_face;
    g_o.b_edge_face   = true;
    publish_profile();
    post((uint8_t)MISSION_EV_PROFILE_DONE, g_prof.plan, (int32_t)g_prof.w_mm);
    goto_state((uint8_t)OBST_ST_IDLE);
}

/* Width probe: the car's real heading from the line (0.01 deg, + left). */
static int32_t probe_theta(motion_odom_t const * p_odom)
{
    return (app_wrap_cdeg(p_odom->heading_cdeg - g_o.edge_h0) * OBST_GAIN_UNIT) / turn_gain();
}

/* Width probe, one 60 ms slot. */
static void probe_slot(uint32_t now_ms, motion_odom_t const * p_odom)
{
    int32_t const th   = probe_theta(p_odom);
    int32_t const done = p_odom->dist_mm - g_o.phase_start_mm;

    g_o.pb_lat = (int32_t)(((int64_t)done * app_sin_q15(th)) / 32768);
    switch (g_o.pb_ph)
    {
        case PB_FACE:
            if (servo_settled(now_ms) && !g_o.b_wait_motion)
            {
                ping();                 /* median of OBST_SCAN_PINGS: handle_range */
            }
            break;

        case PB_MOVE:
        case PB_TURN:
        case PB_UNTURN:
            if (g_o.b_wait_motion)
            {
                int const res = motion_result(p_odom);

                if ((res != 0) && (g_o.pb_ph == PB_MOVE))
                {
                    g_o.b_pb_moved = true;
                    g_o.sw_n       = 0u;
                    g_o.pb_ph      = PB_FACE;       /* measure the face again */
                }
                else if ((res != 0) && (g_o.pb_ph == PB_TURN))
                {
                    g_o.pb_ph = PB_SERVO;
                    servo_to(obst_out_servo_deg(probe_theta(p_odom), 1), now_ms);
                }
                else if (res != 0)
                {
                    probe_finish(now_ms);
                }
                else
                {
                    /* moving */
                }
            }
            else if ((g_o.pb_ph == PB_TURN) && (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED))
            {
                send_step((uint8_t)OBST_STEP_TURN, slip_ddeg((int16_t)OBST_PROBE_DDEG), p_odom);
            }
            else if ((g_o.pb_ph == PB_UNTURN) && (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED))
            {
                int32_t const dh = app_wrap_cdeg(g_o.edge_h0 - p_odom->heading_cdeg);

                if (app_abs_i32(dh) > 100)
                {
                    send_step((uint8_t)OBST_STEP_TURN, (int16_t)(dh / 10), p_odom);
                }
                else
                {
                    probe_finish(now_ms);
                }
            }
            else
            {
                /* stopping */
            }
            break;

        case PB_SERVO:
            if (servo_settled(now_ms))
            {
                g_o.pb_h1          = p_odom->heading_cdeg;
                g_o.phase_start_mm = p_odom->dist_mm;
                g_o.pb_lat         = 0;
                g_o.b_pb_seen      = false;
                g_o.probe.off      = 0u;
                g_o.pb_ph          = PB_LEFT;
            }
            break;

        case PB_LEFT:
        case PB_RIGHT:
            drive((g_o.pb_ph == PB_LEFT) ? OBST_PROBE_SPEED_MM_S : -OBST_PROBE_SPEED_MM_S, g_o.pb_h1, p_odom);
            ping();
            break;

        default:                        /* PB_BACK: forward to where it turned */
            if (done >= 0)
            {
                stop_motion();
                g_o.pb_ph = PB_UNTURN;
            }
            else
            {
                drive(OBST_PROBE_SPEED_MM_S, g_o.pb_h1, p_odom);
            }
            break;
    }
}

/* Width probe, one range reading. */
static void probe_range(uint16_t r, uint32_t now_ms, motion_odom_t const * p_odom)
{
    int32_t const th   = probe_theta(p_odom);
    int32_t const done = p_odom->dist_mm - g_o.phase_start_mm;
    int32_t const lat  = (int32_t)(((int64_t)done * app_sin_q15(th)) / 32768);
    int32_t const fwd  = (int32_t)(((int64_t)done * app_cos_q15(th)) / 32768);

    if (g_o.pb_ph == PB_FACE)
    {
        uint16_t rm;

        if (!scan_sample(r, &rm))
        {
            return;
        }
        if (rm != OBST_RANGE_NONE)
        {
            g_o.edge_face = rm;
        }
        if (!g_o.b_pb_moved && (app_abs_i32((int32_t)g_o.edge_face - OBST_PROBE_FACE_MM) > 15))
        {
            g_o.pb_ph = PB_MOVE;
            send_step((uint8_t)OBST_STEP_MOVE, (int16_t)((int32_t)g_o.edge_face - OBST_PROBE_FACE_MM), p_odom);
        }
        else
        {
            g_o.pb_face_mm = (int32_t)OBST_US_AHEAD_MM + (int32_t)g_o.edge_face;
            g_o.pb_ph      = PB_TURN;
        }
        return;
    }
    if ((g_o.pb_ph != PB_LEFT) && (g_o.pb_ph != PB_RIGHT))
    {
        return;
    }
    if (obst_probe_ping(&g_o.probe, obst_probe_sensor_lat(lat, th), r,
                        obst_out_expect_mm(g_o.pb_face_mm, fwd, th)))
    {
        g_o.b_pb_seen = true;
    }
    if (g_o.pb_ph == PB_LEFT)
    {
        /* past the left edge (or never on the face), or as far as any
           obstacle could reach, or too close to the face */
        if (((g_o.probe.off >= OBST_PROBE_OFF) && (lat >= 20)) || (lat >= OBST_PROBE_MAX_MM) ||
            obst_out_blocked(g_o.pb_face_mm, fwd, th))
        {
            stop_motion();
            g_o.b_pb_seen = false;
            g_o.probe.off = 0u;
            g_o.pb_ph     = PB_RIGHT;
        }
    }
    else if ((g_o.b_pb_seen && (g_o.probe.off >= OBST_PROBE_OFF)) || (lat <= -OBST_PROBE_MAX_MM))
    {
        stop_motion();
        g_o.pb_ph = PB_BACK;
    }
    else
    {
        /* still on the face, or not there yet */
    }
    (void)now_ms;
}

/* ------------------------------------------------------------------ */
static void handle_request(scan_req_t const * p_req, uint32_t now_ms, motion_odom_t const * p_odom)
{
    switch (p_req->type)
    {
        case SCAN_REQ_MONITOR:
            goto_state((uint8_t)OBST_ST_MONITOR);
            servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
            g_o.win_n       = 0u;
            g_o.b_slow_sent = false;
            g_o.b_obst_sent = false;
            break;

        case SCAN_REQ_PROFILE:
            if (!g_o.b_have_pose)
            {
                store_pose(p_odom);
            }
            g_o.retries       = 0u;
            g_o.b_blocked     = false;
            g_o.b_second_look = false;
            g_o.back_mm       = 0;
            start_scan(now_ms);
            break;

        case SCAN_REQ_BYPASS_LEFT:
        case SCAN_REQ_BYPASS_RIGHT:
            g_o.side    = (p_req->type == (uint8_t)SCAN_REQ_BYPASS_LEFT) ? (uint8_t)OBST_PLAN_LEFT
                                                                         : (uint8_t)OBST_PLAN_RIGHT;
            if (g_prof.plan != g_o.side)
            {
                /* deviation from the profile is signed by the planned side */
                g_prof.deviation_mm = (int16_t)((g_o.side == (uint8_t)OBST_PLAN_LEFT) ?
                    (g_prof.left_lat_mm + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM) :
                    -(-g_prof.right_lat_mm + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM));
                g_prof.plan = g_o.side;
                /* 7 Nov: the step-out leg measures the rest */
                obst_edge_plan_min(&g_prof, g_o.b_edge_has[0], g_o.edge_min[0], g_o.b_edge_has[1], g_o.edge_min[1]);
            }
            g_o.n_steps  = obst_bypass_plan(&g_prof, g_o.side, g_steps);
            g_o.step_i    = 0u;
            g_o.b_joined  = false;
            g_o.scan_pass = 0u;
            g_o.scan_ph   = LS_PH_SWEEP;
            {
                uint8_t k;

                for (k = 0u; k < g_o.n_steps; k++)
                {
                    if (g_steps[k].kind == (uint8_t)OBST_STEP_PASS)
                    {
                        log_ev(OBST_LOG_PLAN, (int32_t)(((uint32_t)(uint16_t)g_prof.deviation_mm << 16) |
                                                        (uint16_t)g_steps[k].value));
                    }
                }
            }
            goto_state((uint8_t)OBST_ST_BYPASS);
            servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
            break;

        case SCAN_REQ_BENCH_SERVO:      /* APP_BENCH: range at any angle */
            goto_state((uint8_t)OBST_ST_MONITOR);
            servo_to(app_clamp_i32(p_req->param, 0, 180), now_ms);
            g_o.win_n       = 0u;
            g_o.b_slow_sent = true;     /* no SLOW_DOWN / OBSTACLE while benching */
            g_o.b_obst_sent = true;
            break;

        case SCAN_REQ_RECOVER:
            goto_state((uint8_t)OBST_ST_SEEK_LINE);
            servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
            g_o.phase_start_mm = p_odom->dist_mm;
            break;

        default:                        /* STOP */
            if ((g_o.state == (uint8_t)OBST_ST_BYPASS) || (g_o.state == (uint8_t)OBST_ST_SEEK_LINE) ||
                (g_o.state == (uint8_t)OBST_ST_SEEK_SWEEP) || (g_o.state == (uint8_t)OBST_ST_EDGE_SCAN) ||
                (g_o.state == (uint8_t)OBST_ST_WIDTH_PROBE))
            {
                stop_motion();
            }
            goto_state((uint8_t)OBST_ST_IDLE);
            servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
            g_o.b_have_pose = false;
            break;
    }
}

/* One range result (r = OBST_RANGE_NONE: no target or no echo). */
static void handle_range(uint16_t r, uint32_t now_ms, motion_odom_t const * p_odom)
{
    switch (g_o.state)
    {
        case OBST_ST_MONITOR:
        {
            imu_snap_t     imu;
            mission_snap_t ms;
            bool           b_masked;

            g_o.win[g_o.win_n % 3u] = r;
            g_o.win_n++;
            g_o.d_mm = (g_o.win_n >= 3u) ? obst_median3(g_o.win[0], g_o.win[1], g_o.win[2])
                                         : OBST_RANGE_NONE;

            /* < 8 cm ahead: TOO_CLOSE, never masked (FR15).  Only while the
               sensor looks forward (+-30 deg): sideways it may see a wall. */
            if ((g_o.d_mm != OBST_RANGE_NONE) && (g_o.d_mm < OBST_TOO_CLOSE_MM) && !g_o.b_too_close &&
                (app_abs_i32(g_o.servo_deg - OBST_SERVO_CENTRE_DEG) <= 30))
            {
                g_o.b_too_close = true;
                (void)tk_set_flg(g_flg_safety, SAFETY_FLT_TOO_CLOSE);
            }
            else if (g_o.b_too_close && ((g_o.d_mm == OBST_RANGE_NONE) || (g_o.d_mm > OBST_TOO_CLOSE_RESET_MM)))
            {
                g_o.b_too_close = false;
            }
            else
            {
                /* no change */
            }

            imu_get_snap(&imu);
            mission_get_snap(&ms);
            b_masked = obst_hump_mask(&g_o.mask, (imu.pitch_valid != 0u) && (imu.ok != 0u),
                                      imu.pitch_cdeg, p_odom->dist_mm, now_ms);
            if (b_masked || (g_o.d_mm == OBST_RANGE_NONE))
            {
                if (g_o.b_slow_sent && (g_o.d_mm == OBST_RANGE_NONE))
                {
                    g_o.b_slow_sent = false;
                    post((uint8_t)MISSION_EV_SLOW_DOWN, 0u, -1);    /* clear */
                }
                break;
            }
            if (!g_o.b_slow_sent && (g_o.d_mm < ms.slow_mm))
            {
                g_o.b_slow_sent = true;
                post((uint8_t)MISSION_EV_SLOW_DOWN, 0u, (int32_t)g_o.d_mm);
            }
            else if (g_o.b_slow_sent && (g_o.d_mm > (ms.slow_mm + 50u)))
            {
                g_o.b_slow_sent = false;
                post((uint8_t)MISSION_EV_SLOW_DOWN, 0u, -1);
            }
            else
            {
                /* no change */
            }
            if (!g_o.b_obst_sent && (g_o.d_mm < ms.stop_mm) && (p_odom->dist_mm >= g_o.ignore_until_mm))
            {
                g_o.b_obst_sent = true;
                store_pose(p_odom);
                g_o.det_mm = g_o.d_mm;
                post((uint8_t)MISSION_EV_OBSTACLE, 0u, (int32_t)g_o.d_mm);
            }
            break;
        }

        case OBST_ST_WIDTH_PROBE:
            probe_range(r, now_ms, p_odom);
            break;

        case OBST_ST_EDGE_SCAN:
        {
            uint16_t rm;

            if ((g_o.edge_ph == EDGE_PH_PING) && scan_sample(r, &rm))
            {
                uint8_t const k   = (g_o.edge_side > 0) ? 0u : 1u;
                int32_t const phi = (app_wrap_cdeg(p_odom->heading_cdeg - g_o.edge_h0) * OBST_GAIN_UNIT) / turn_gain();

                int32_t const lat = obst_edge_wide_lat(phi, rm, g_o.edge_side);
                bool          b_past = false;

                if (!g_o.b_edge_face)
                {
                    /* 4 Nov: the face straight ahead, measured here, not the
                       servo scan's nearest echo (one close stray made the
                       real face look "behind" and the left got 0 echoes) */
                    g_o.b_edge_face = true;
                    if (rm != OBST_RANGE_NONE)
                    {
                        g_o.edge_face = rm;
                    }
                    g_o.edge_ph = EDGE_PH_TURN;
                    break;
                }
                g_o.edge_deg[k] = (uint8_t)app_min_i32(app_abs_i32(phi) / 100, 255);
                if ((k == 0u) && (g_o.edge_first_l == 0u))
                {
                    g_o.edge_first_l = (rm == OBST_RANGE_NONE) ? 0xFFFFu : rm;
                }
                if (obst_edge_on_face(phi, rm, g_o.edge_face) && !obst_edge_past_max(lat, g_o.edge_side))
                {
                    g_o.b_edge_has[k] = true;
                    g_o.edge_lat[k]   = lat;
                    g_o.edge_min[k]   = obst_edge_lat(phi);
                    g_o.edge_phi[k]   = phi;
                    g_o.edge_r[k]     = rm;
                    g_o.edge_lo       = phi;
                    g_o.edge_miss     = 0u;
                    g_o.edge_hits[k]++;
                }
                else
                {
                    if (obst_edge_on_face(phi, rm, g_o.edge_face))
                    {
                        /* 4 Nov: echoes 32 cm away at 50 deg right (mat edge,
                           floor) were taken as the box, edge 28.6 cm right of
                           the line.  An edge further from the line than the
                           largest obstacle is not this obstacle: stop here,
                           that side's edge is not known. */
                        b_past = true;
                    }
                    if ((g_o.edge_miss == 0u) || (g_o.edge_ref > 0u))
                    {
                        g_o.edge_hi = phi;  /* first miss (refine: the nearer one) */
                    }
                    g_o.edge_miss++;
                }
                if (g_o.edge_ref > 0u)
                {
                    /* refining: next halfway, or done; the edge is taken
                       halfway between the last echo and the nearest miss
                       (the last echo alone always reads a little inside) */
                    g_o.edge_phi[k] = (g_o.edge_lo + g_o.edge_hi) / 2;
                    g_o.edge_ref++;
                    g_o.edge_ph = (g_o.edge_ref > OBST_EDGE_REFINE_ITERS) ? EDGE_PH_RETURN : EDGE_PH_TURN;
                }
                else if (b_past || (g_o.edge_miss >= OBST_EDGE_MISSES) ||
                         (app_abs_i32(phi) >= (OBST_EDGE_MAX_DDEG * 10)))
                {
                    /* 7 Nov: a side that ended on clear readings after an
                       echo is refined between the last echo and the first
                       miss; past-max (clutter) and the turn limit are not */
                    g_o.edge_ref = (g_o.b_edge_has[k] && !b_past && (g_o.edge_miss >= OBST_EDGE_MISSES) &&
                                    (OBST_EDGE_REFINE_ITERS > 0u)) ? 1u : 0u;
                    g_o.edge_ph  = (g_o.edge_ref > 0u) ? EDGE_PH_TURN : EDGE_PH_RETURN;
                }
                else
                {
                    g_o.edge_ph = EDGE_PH_TURN;
                }
                if (g_o.edge_ph == EDGE_PH_TURN)
                {
                    int32_t step = g_o.edge_side * OBST_EDGE_STEP_DDEG;

                    if (g_o.edge_ref > 0u)
                    {
                        /* halfway between the last echo and the miss; at
                           least 1.5 deg (a smaller turn may not move the
                           wheels and could read as a stall); the ping is
                           tagged with the heading reached, not asked */
                        step = (((g_o.edge_lo + g_o.edge_hi) / 2) - phi) / 10;
                        if (app_abs_i32(step) < OBST_EDGE_MIN_TURN_DDEG)
                        {
                            step = (step >= 0) ? OBST_EDGE_MIN_TURN_DDEG : -OBST_EDGE_MIN_TURN_DDEG;
                        }
                    }
                    g_o.edge_step = (int16_t)step;
                }
            }
            break;
        }

        case OBST_ST_COARSE_SCAN:
        case OBST_ST_FINE_SCAN:
        {
            uint16_t rm;

            if (!scan_sample(r, &rm))
            {
                break;                      /* more pings at this angle */
            }
            obst_scan_feed(&g_scan, rm);
            g_o.state = (g_scan.phase == (uint8_t)OBST_SCAN_COARSE) ? (uint8_t)OBST_ST_COARSE_SCAN
                                                                     : (uint8_t)OBST_ST_FINE_SCAN;
            if (obst_scan_done(&g_scan))
            {
                plan(now_ms, p_odom);
            }
            else
            {
                servo_to(obst_scan_angle(&g_scan), now_ms);
            }
            break;
        }

        case OBST_ST_BYPASS:
            if ((g_o.step_i < g_o.n_steps) && (g_steps[g_o.step_i].kind == (uint8_t)OBST_STEP_OUT) &&
                g_o.b_wait_motion && servo_settled(now_ms))
            {
                int32_t const s    = (g_o.side == (uint8_t)OBST_PLAN_LEFT) ? 1 : -1;
                int32_t const th   = out_theta(p_odom, s);
                int32_t const done = p_odom->dist_mm - g_o.phase_start_mm;

                obst_out_ping(&g_o.out, (int32_t)(((int64_t)done * app_sin_q15(th)) / 32768),
                              (int32_t)(((int64_t)done * app_cos_q15(th)) / 32768), th, r, g_o.out_face_mm);
            }
            else if ((g_o.step_i < g_o.n_steps) && (g_steps[g_o.step_i].kind == (uint8_t)OBST_STEP_PASS) &&
                (r != OBST_RANGE_NONE) &&
                ((int32_t)r < (app_abs_i32(g_prof.deviation_mm) + 150)))
            {
                g_o.b_seen       = true;            /* obstacle still beside us */
                g_o.last_seen_mm = p_odom->dist_mm;
            }
            break;

        default:
            break;
    }
}

/* One 60 ms slot. */
static void slot(uint32_t now_ms, motion_odom_t const * p_odom)
{
    switch (g_o.state)
    {
        case OBST_ST_MONITOR:
        case OBST_ST_COARSE_SCAN:
        case OBST_ST_FINE_SCAN:
            if (servo_settled(now_ms))
            {
                ping();
            }
            break;

        case OBST_ST_WIDTH_PROBE:
            probe_slot(now_ms, p_odom);
            break;

        case OBST_ST_EDGE_SCAN:
            switch (g_o.edge_ph)
            {
                case EDGE_PH_TURN:
                    if (!g_o.b_wait_motion)
                    {
                        if (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED)
                        {
                            send_turn(slip_ddeg(g_o.edge_step), OBST_EDGE_TURN_DEG_S, p_odom);
                        }
                    }
                    else if (motion_result(p_odom) != 0)
                    {
                        g_o.edge_ph       = EDGE_PH_SETTLE;
                        g_o.edge_until_ms = now_ms + OBST_EDGE_SETTLE_MS;
                    }
                    else
                    {
                        /* turning */
                    }
                    break;

                case EDGE_PH_SETTLE:
                    if ((int32_t)(now_ms - g_o.edge_until_ms) >= 0)
                    {
                        g_o.sw_n    = 0u;
                        g_o.edge_ph = EDGE_PH_PING;
                    }
                    break;

                case EDGE_PH_PING:
                    if (servo_settled(now_ms))
                    {
                        ping();
                    }
                    break;

                default:                    /* EDGE_PH_RETURN: back to where it stopped */
                    if (!g_o.b_wait_motion)
                    {
                        if (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED)
                        {
                            int32_t const dh = app_wrap_cdeg(g_o.edge_h0 - p_odom->heading_cdeg);

                            if (app_abs_i32(dh) > 100)
                            {
                                send_step((uint8_t)OBST_STEP_TURN, (int16_t)(dh / 10), p_odom);
                            }
                            else if (g_o.edge_side > 0)
                            {
                                g_o.edge_side = -1;
                                g_o.edge_miss = 0u;
                                g_o.edge_ref  = 0u;
                                g_o.edge_step = (int16_t)-OBST_EDGE_STEP_DDEG;
                                g_o.edge_ph   = EDGE_PH_TURN;
                            }
                            else
                            {
                                edge_finish();
                            }
                        }
                    }
                    else
                    {
                        (void)motion_result(p_odom);    /* clears b_wait_motion when done */
                    }
                    break;
            }
            break;

        case OBST_ST_REVERSE:
        {
            int const res = motion_result(p_odom);

            if (res != 0)
            {
                start_scan(now_ms);         /* rescan from further back */
            }
            break;
        }

        case OBST_ST_BLOCKED:
            if ((uint32_t)(now_ms - g_o.blocked_start_ms) >= OBST_BLOCKED_TIMEOUT_MS)
            {
                post((uint8_t)MISSION_EV_BLOCKED_TIMEOUT, 0u, 0);
                g_o.b_blocked = false;
                goto_state((uint8_t)OBST_ST_IDLE);
            }
            else if (((int32_t)(now_ms - g_o.next_rescan_ms) >= 0) && (g_o.rescans < OBST_BLOCKED_RESCANS))
            {
                g_o.rescans++;
                g_o.sw_n = 0u;
                obst_scan_start(&g_scan);
                g_o.state = (uint8_t)OBST_ST_COARSE_SCAN;   /* stays "blocked" on return */
                servo_to(obst_scan_angle(&g_scan), now_ms);
            }
            else
            {
                /* waiting */
            }
            break;

        case OBST_ST_BYPASS:
        {
            obst_step_t const * p_st = &g_steps[g_o.step_i];

            if (g_o.step_i >= g_o.n_steps)
            {
                if (g_o.b_joined)
                {
                    g_o.b_have_pose = false;    /* the line was touched: SEEK takes it */
                }
                servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
                post((uint8_t)MISSION_EV_BYPASS_DONE, 0u, 0);
                goto_state((uint8_t)OBST_ST_IDLE);
                break;
            }
            if (p_st->kind == (uint8_t)OBST_STEP_OUT)
            {
                int32_t const s   = (g_o.side == (uint8_t)OBST_PLAN_LEFT) ? 1 : -1;
                int32_t const th  = out_theta(p_odom, s);
                int32_t       done_mm;
                int32_t       lat;

                if (!g_o.b_wait_motion)
                {
                    /* start OUT: the sensor looks along the line at the face */
                    int32_t back = 0;

                    if ((g_steps[0].kind == (uint8_t)OBST_STEP_MOVE) && (g_steps[0].value < 0))
                    {
                        back = -g_steps[0].value;
                    }
                    g_o.b_wait_motion  = true;
                    g_o.phase_start_mm = p_odom->dist_mm;
                    g_o.out_h          = p_odom->heading_cdeg;
                    g_o.out_face_mm    = (int32_t)OBST_US_AHEAD_MM + back +
                                         (int32_t)(g_o.b_edge_face ? g_o.edge_face : g_prof.closest_mm);
                    obst_out_start(&g_o.out, app_abs_i32(g_prof.deviation_mm));
                    servo_to(obst_out_servo_deg(th, s), now_ms);
                }
                done_mm = p_odom->dist_mm - g_o.phase_start_mm;
                lat     = (int32_t)(((int64_t)done_mm * app_sin_q15(th)) / 32768);
                if (obst_out_done(&g_o.out, lat) ||
                    obst_out_blocked(g_o.out_face_mm, (int32_t)(((int64_t)done_mm * app_cos_q15(th)) / 32768), th))
                {
                    /* later legs from the real step-out: PASS shorter by the
                       extra forward travel, JOIN's limit from the real leg */
                    int32_t const extra_fwd = (int32_t)(((int64_t)(done_mm - p_st->value) * app_cos_q15(th)) / 32768);
                    uint8_t       k;

                    stop_motion();
                    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
                    log_ev(OBST_LOG_OUT, (int32_t)(((g_o.out.b_seen ? (uint32_t)(uint16_t)g_o.out.seen_lat : 0xFFFFu) << 16) |
                                                   (uint32_t)app_clamp_i32(lat, 0, 0xFFFF)));
                    for (k = (uint8_t)(g_o.step_i + 1u); k < g_o.n_steps; k++)
                    {
                        if (g_steps[k].kind == (uint8_t)OBST_STEP_PASS)
                        {
                            g_steps[k].value = (int16_t)app_max_i32((int32_t)g_steps[k].value - extra_fwd, 0);
                        }
                        else if (g_steps[k].kind == (uint8_t)OBST_STEP_JOIN)
                        {
                            g_steps[k].value = (int16_t)app_min_i32(((done_mm * OBST_JOIN_MAX_PERMILLE) / 1000) +
                                                                    OBST_JOIN_EXTRA_MM, 0x7FFF);
                        }
                        else
                        {
                            /* unchanged */
                        }
                    }
                    g_o.b_wait_motion = false;
                    g_o.step_i++;
                }
                else
                {
                    drive(OBST_BYPASS_SPEED_MM_S, g_o.out_h, p_odom);
                    if (servo_settled(now_ms))
                    {
                        ping();
                    }
                }
            }
            else if (p_st->kind == (uint8_t)OBST_STEP_PASS)
            {
                int32_t     done_mm;

                if (!g_o.b_wait_motion)
                {
                    /* start PASS: look at the obstacle's side */
                    g_o.b_wait_motion  = true;
                    g_o.b_seen         = false;
                    g_o.phase_start_mm = p_odom->dist_mm;
                    g_o.last_seen_mm   = p_odom->dist_mm;
                    servo_to((g_o.side == (uint8_t)OBST_PLAN_LEFT) ? (OBST_SERVO_CENTRE_DEG - OBST_SIDE_SERVO_DEG)
                                                                   : (OBST_SERVO_CENTRE_DEG + OBST_SIDE_SERVO_DEG),
                             now_ms);
                }
                /* after the start above (3 Nov: computed before it, the first
                   check used the distance since boot and ended PASS at once) */
                done_mm = p_odom->dist_mm - g_o.phase_start_mm;
                /* 7 Nov: the pass ignores the line; the turn back in always
                   follows (the 6 Nov skip fired on straight track) */
                /* 3 Nov: at least the planned distance; then on until the
                   side ping has not seen the obstacle for a car length +
                   margin; never beyond OBST_PASS_MAX_MM */
                if (((done_mm >= p_st->value) &&
                     (!g_o.b_seen || ((p_odom->dist_mm - g_o.last_seen_mm) >= (OBST_ROBOT_LENGTH_MM + OBST_MARGIN_MM)))) ||
                    (done_mm >= OBST_PASS_MAX_MM))
                {
                    stop_motion();
                    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
                    log_ev(OBST_LOG_PASS, (int32_t)(((g_o.b_seen ? 1u : 0u) << 16) |
                                                    (uint32_t)app_clamp_i32(done_mm, 0, 0xFFFF)));
                    g_o.b_wait_motion = false;
                    g_o.step_i++;
                }
                else
                {
                    drive(OBST_BYPASS_SPEED_MM_S, g_o.h0, p_odom);
                    if (servo_settled(now_ms))
                    {
                        ping();
                    }
                }
            }
            else if (p_st->kind == (uint8_t)OBST_STEP_JOIN)
            {
                line_snap_t line;
                int32_t     done_mm;

                if (!g_o.b_wait_motion)
                {
                    g_o.b_wait_motion  = true;
                    g_o.phase_start_mm = p_odom->dist_mm;
                    g_o.join_h         = p_odom->heading_cdeg;
                }
                done_mm = p_odom->dist_mm - g_o.phase_start_mm;
                line_get_snap(&line);
                if (((line.flags & LINE_FLAG_ON_LINE) != 0u) || (done_mm >= p_st->value))
                {
                    stop_motion();
                    g_o.b_joined = ((line.flags & LINE_FLAG_ON_LINE) != 0u);
                    log_ev(OBST_LOG_JOIN, g_o.b_joined ? app_max_i32(done_mm, 1) : 0);
                    g_o.b_wait_motion = false;
                    g_o.step_i++;
                }
                else
                {
                    drive(OBST_BYPASS_SPEED_MM_S, g_o.join_h, p_odom);
                }
            }
            else if (p_st->kind == (uint8_t)OBST_STEP_LINESCAN)
            {
                /* 6 Nov, as the reference robot: spin toward value's side
                   noting the heading where each line sensor goes on to
                   and off the line; once both have crossed it the line
                   runs midway between those four headings: stop, settle,
                   turn there.  If the sweep ends first, sweep back the
                   other way to OBST_LINESCAN_SPARE past the start. */
                line_snap_t   line;
                int32_t const s   = (p_st->value >= 0) ? 1 : -1;
                int32_t const rel = app_wrap_cdeg(p_odom->heading_cdeg - g_o.scan_h0);
                uint8_t       i;

                line_get_snap(&line);
                if (g_o.scan_ph == LS_PH_ALIGN)
                {
                    if (!g_o.b_wait_motion)
                    {
                        if ((int32_t)(now_ms - g_o.scan_until_ms) >= 0)
                        {
                            int32_t const d = g_o.scan_target - rel;   /* encoder heading: slip already in it */

                            if (g_o.b_scan_turned || (app_abs_i32(d) < 200))
                            {
                                /* 7 Nov: < 0 = turned to a one-sensor crossing */
                                log_ev(OBST_LOG_LINESCAN, (g_o.b_scan_part ? -1 : 1) *
                                                          app_max_i32(app_abs_i32(g_o.scan_target) / 10, 1));
                                g_o.b_scan_part = false;
                                g_o.b_joined  = true;
                                g_o.scan_pass = 0u;
                                g_o.scan_ph   = LS_PH_SWEEP;
                                g_o.step_i++;
                            }
                            else if (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED)
                            {
                                g_o.b_scan_turned = true;
                                send_step((uint8_t)OBST_STEP_TURN, (int16_t)(d / 10), p_odom);
                            }
                            else
                            {
                                /* still stopping */
                            }
                        }
                    }
                    else
                    {
                        (void)motion_result(p_odom);    /* the turn to the middle */
                    }
                }
                else if (!g_o.b_wait_motion)
                {
                    if ((g_o.scan_pass < 2u) && (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED))
                    {
                        int16_t amount;

                        if (g_o.scan_pass == 0u)
                        {
                            g_o.scan_h0     = p_odom->heading_cdeg;
                            amount          = slip_ddeg(p_st->value);
                            g_o.ls_part_q   = 0u;
                            g_o.b_scan_part = false;
                        }
                        else
                        {
                            /* back to OBST_LINESCAN_SPARE past where the scan began */
                            amount = (int16_t)((-s * ((OBST_LINESCAN_SPARE_DDEG * 10) + (s * rel))) / 10);
                        }
                        for (i = 0u; i < 2u; i++)
                        {
                            g_o.ls_seen[i] = 0u;
                            g_o.b_ls_prev[i] = false;
                        }
                        send_turn(amount, OBST_LINESCAN_DEG_S, p_odom);
                        g_o.scan_pass++;
                    }
                    else if ((g_o.scan_pass >= 2u) && (g_o.ls_part_q > 0u))
                    {
                        /* 7 Nov: both sweeps done, only one sensor crossed the
                           line (it curves away under the car, 21:10 run):
                           turn to where it crossed and let line following,
                           which can take a bend, carry on from there */
                        g_o.scan_target   = g_o.ls_part_h;
                        g_o.b_scan_part   = true;
                        g_o.b_scan_turned = false;
                        g_o.scan_ph       = LS_PH_ALIGN;
                        g_o.scan_until_ms = now_ms + OBST_LINESCAN_SETTLE_MS;
                    }
                    else if (g_o.scan_pass >= 2u)
                    {
                        /* both sweeps done, no sensor crossed: SEEK takes any
                           line if one was touched on the way in (b_joined) */
                        log_ev(OBST_LOG_LINESCAN, 0);
                        g_o.scan_pass = 0u;
                        g_o.step_i++;
                    }
                    else
                    {
                        /* still stopping */
                    }
                }
                else
                {
                    for (i = 0u; i < 2u; i++)
                    {
                        bool const b_on = ((line.sens & ((i == 0u) ? LINE_SENS_L : LINE_SENS_R)) != 0u);

                        if (b_on && !g_o.b_ls_prev[i] && (g_o.ls_seen[i] == 0u))
                        {
                            g_o.ls_on[i]   = rel;
                            g_o.ls_seen[i] = 1u;
                            if (g_o.ls_part_q == 0u)
                            {
                                g_o.ls_part_q = 1u;         /* went on: a fallback */
                                g_o.ls_part_h = rel;
                            }
                        }
                        else if (!b_on && g_o.b_ls_prev[i] && (g_o.ls_seen[i] == 1u))
                        {
                            g_o.ls_off[i]  = rel;
                            g_o.ls_seen[i] = 2u;
                            if (g_o.ls_part_q < 2u)
                            {
                                g_o.ls_part_q = 2u;         /* a full crossing by one sensor */
                                g_o.ls_part_h = (g_o.ls_on[i] + rel) / 2;
                            }
                        }
                        else
                        {
                            /* no change */
                        }
                        g_o.b_ls_prev[i] = b_on;
                    }
                    if ((g_o.ls_seen[0] == 2u) && (g_o.ls_seen[1] == 2u))
                    {
                        g_o.scan_target   = (g_o.ls_on[0] + g_o.ls_off[0] + g_o.ls_on[1] + g_o.ls_off[1]) / 4;
                        stop_motion();
                        g_o.b_wait_motion = false;
                        g_o.b_scan_turned = false;
                        g_o.scan_ph       = LS_PH_ALIGN;
                        g_o.scan_until_ms = now_ms + OBST_LINESCAN_SETTLE_MS;
                    }
                    else if (motion_result(p_odom) != 0)
                    {
                        /* this sweep is over: the next one starts above */
                    }
                    else
                    {
                        /* still turning */
                    }
                }
            }
            else if (!g_o.b_wait_motion)
            {
                if (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED)
                {
                    send_step(p_st->kind, (p_st->kind == (uint8_t)OBST_STEP_TURN) ? slip_ddeg(p_st->value)
                                                                                  : p_st->value, p_odom);
                }
            }
            else
            {
                int const res = motion_result(p_odom);

                if (res > 0)
                {
                    g_o.step_i++;
                }
                else if (res < 0)
                {
                    post((uint8_t)MISSION_EV_RECOVERY_FAILED, 1u, 0);  /* bypass step failed */
                    goto_state((uint8_t)OBST_ST_IDLE);
                }
                else
                {
                    /* still moving */
                }
            }
            break;
        }

        case OBST_ST_SEEK_LINE:
        {
            line_snap_t line;

            line_get_snap(&line);
            if (((line.flags & LINE_FLAG_ON_LINE) != 0u) &&
                (!g_o.b_have_pose ||
                 obst_line_accept(p_odom->x_mm, p_odom->y_mm, p_odom->heading_cdeg, g_o.x0, g_o.y0, g_o.h0)))
            {
                stop_motion();
                goto_state((uint8_t)OBST_ST_ALIGN);
            }
            else if ((p_odom->dist_mm - g_o.phase_start_mm) > OBST_SEEK_MAX_MM)
            {
                stop_motion();
                g_o.sweep_i = 0u;
                goto_state((uint8_t)OBST_ST_SEEK_SWEEP);     /* not straight ahead: look around */
            }
            else
            {
                drive(OBST_SEEK_SPEED_MM_S, g_o.b_have_pose ? g_o.h0 : p_odom->heading_cdeg, p_odom);
            }
            break;
        }

        case OBST_ST_SEEK_SWEEP:
        {
            static int16_t const k_sweep[3] = { OBST_SWEEP_DDEG, -(2 * OBST_SWEEP_DDEG), OBST_SWEEP_DDEG };
            line_snap_t line;

            line_get_snap(&line);
            if ((line.flags & LINE_FLAG_ON_LINE) != 0u)
            {
                /* any line: line following takes over from here */
                stop_motion();
                g_o.b_have_pose = false;
                post((uint8_t)MISSION_EV_RECOVERY_DONE, 0u, 0);
                goto_state((uint8_t)OBST_ST_IDLE);
            }
            else if (!g_o.b_wait_motion)
            {
                if (g_o.sweep_i < 3u)
                {
                    if (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED)
                    {
                        send_turn(slip_ddeg(k_sweep[g_o.sweep_i]), OBST_SWEEP_TURN_DEG_S, p_odom);
                    }
                }
                else
                {
                    post((uint8_t)MISSION_EV_RECOVERY_FAILED, 0u, 0);
                    goto_state((uint8_t)OBST_ST_IDLE);
                    g_o.b_have_pose = false;
                }
            }
            else if (motion_result(p_odom) != 0)
            {
                g_o.sweep_i++;              /* done or failed: next leg */
            }
            else
            {
                /* turning */
            }
            break;
        }

        case OBST_ST_ALIGN:
        {
            int32_t const dh = app_wrap_cdeg(g_o.h0 - p_odom->heading_cdeg);

            if (!g_o.b_wait_motion && g_o.b_have_pose && (app_abs_i32(dh) > OBST_ALIGN_TOL_CDEG) &&
                (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED))
            {
                send_step((uint8_t)OBST_STEP_TURN, (int16_t)(dh / 10), p_odom);
            }
            else if (g_o.b_wait_motion)
            {
                if (motion_result(p_odom) != 0)
                {
                    g_o.b_wait_motion = false;
                    g_o.b_have_pose   = false;
                    post((uint8_t)MISSION_EV_RECOVERY_DONE, 0u, 0);
                    goto_state((uint8_t)OBST_ST_IDLE);
                }
            }
            else if (p_odom->state == (uint8_t)MOTION_ST_IDLE_BRAKED)    /* braked and aligned */
            {
                g_o.b_have_pose = false;
                post((uint8_t)MISSION_EV_RECOVERY_DONE, 0u, 0);
                goto_state((uint8_t)OBST_ST_IDLE);
            }
            else
            {
                /* still stopping */
            }
            break;
        }

        default:                        /* IDLE, PLAN */
            break;
    }
}

EXPORT void obstacle_task(INT stacd, void * p_exinf)
{
    scan_req_t    req;
    motion_odom_t odom;
    obst_snap_t   snap;

    (void)stacd;
    (void)p_exinf;

    (void)drv_us_init();
    (void)drv_servo_init();
    (void)drv_us_attach();              /* echo_isr on this core (core 1) */
    g_o.servo_deg = OBST_SERVO_CENTRE_DEG;
    g_o.state     = (uint8_t)OBST_ST_IDLE;
    g_o.d_mm      = OBST_RANGE_NONE;

    for (;;)
    {
        UINT     ptn = 0u;
        uint32_t now_ms;
        uint32_t width;

        (void)tk_wai_flg(g_flg_obst, OBST_FLG_RANGE | OBST_FLG_ECHO | OBST_FLG_REQ,
                         TWF_ORW | TWF_BITCLR, &ptn, OBST_WAIT_TIMEOUT_MS);
        safety_heartbeat(SAFETY_HB_OBSTACLE);
        now_ms = (uint32_t)app_time_ms();
        motion_get_odom(&odom);

        if ((ptn & OBST_FLG_REQ) != 0u)
        {
            while (tk_rcv_mbf(g_mbf_scan, &req, TMO_POL) == (INT)sizeof(req))
            {
                handle_request(&req, now_ms, &odom);
            }
        }
        if (drv_us_take(&width))
        {
            g_o.b_ping_out = false;
            g_o.no_echo    = 0u;
            if (g_o.b_no_echo_set)
            {
                g_o.b_no_echo_set = false;
                (void)tk_clr_flg(g_flg_safety, ~(UINT)SAFETY_FLT_NO_ECHO);
            }
            handle_range(obst_echo_to_mm(width), now_ms, &odom);
        }
        if ((ptn & OBST_FLG_RANGE) != 0u)
        {
            if (g_o.b_ping_out)
            {
                /* no echo at all in a whole slot: sensor silent (report only) */
                g_o.b_ping_out = false;
                if (g_o.no_echo < 0xFFFFu)
                {
                    g_o.no_echo++;
                }
                if ((g_o.no_echo >= OBST_NO_ECHO_PINGS) && !g_o.b_no_echo_set)
                {
                    g_o.b_no_echo_set = true;
                    (void)tk_set_flg(g_flg_safety, SAFETY_FLT_NO_ECHO);
                }
                handle_range(OBST_RANGE_NONE, now_ms, &odom);
            }
            slot(now_ms, &odom);
        }

        obst_get_snap(&snap);
        snap.t_ms    = now_ms;
        snap.d_mm    = g_o.d_mm;
        snap.state   = g_o.state;
        snap.retries = g_o.retries;
        snap.no_echo = g_o.no_echo;
        snap.pings   = g_o.pings;
        obst_put_snap(&snap);
    }
}
