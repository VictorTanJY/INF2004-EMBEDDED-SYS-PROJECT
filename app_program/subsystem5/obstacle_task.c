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
 *            two bisection pings per edge -> width, edges, clearances, plan
 *            -> PROFILE_DONE(plan, width).  No clearance: reverse 100 mm and
 *            rescan (twice), then BLOCKED: rescan every 2 s, 5 times; after
 *            10 s BLOCKED_TIMEOUT.
 *  BYPASS    turn out, offset, turn parallel, PASS (servo looking at the
 *            obstacle until it has been seen and passed), turn in, offset
 *            back, align -> BYPASS_DONE.  Commands use AVOID authority;
 *            completion is read from last_done_seq in the odometry snapshot.
 *  RECOVER   creep forward (<= 0.5 m) until a line sensor sees black within
 *            +-60 mm of the original path and +-20 deg of its heading, align
 *            -> RECOVERY_DONE, or RECOVERY_FAILED.
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
    obst_scan_profile(&g_scan, &g_prof);
    publish_profile();
    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);

    if (g_prof.plan != (uint8_t)OBST_PLAN_BLOCKED)
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
            g_o.retries   = 0u;
            g_o.b_blocked = false;
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
            }
            g_o.n_steps = obst_bypass_plan(&g_prof, g_o.side, g_steps);
            g_o.step_i  = 0u;
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
            if ((g_o.state == (uint8_t)OBST_ST_BYPASS) || (g_o.state == (uint8_t)OBST_ST_SEEK_LINE))
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
                post((uint8_t)MISSION_EV_OBSTACLE, 0u, (int32_t)g_o.d_mm);
            }
            break;
        }

        case OBST_ST_COARSE_SCAN:
        case OBST_ST_FINE_SCAN:
            obst_scan_feed(&g_scan, r);
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

        case OBST_ST_BYPASS:
            if ((g_o.step_i < g_o.n_steps) && (g_steps[g_o.step_i].kind == (uint8_t)OBST_STEP_PASS) &&
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
                servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
                post((uint8_t)MISSION_EV_BYPASS_DONE, 0u, 0);
                goto_state((uint8_t)OBST_ST_IDLE);
                break;
            }
            if (p_st->kind == (uint8_t)OBST_STEP_PASS)
            {
                int32_t const done_mm = p_odom->dist_mm - g_o.phase_start_mm;

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
                if ((g_o.b_seen && ((p_odom->dist_mm - g_o.last_seen_mm) >= (OBST_ROBOT_LENGTH_MM + OBST_MARGIN_MM))) ||
                    (done_mm >= p_st->value))
                {
                    stop_motion();
                    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
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
            else if (!g_o.b_wait_motion)
            {
                send_step(p_st->kind, p_st->value, p_odom);
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
                post((uint8_t)MISSION_EV_RECOVERY_FAILED, 0u, 0);
                goto_state((uint8_t)OBST_ST_IDLE);
                g_o.b_have_pose = false;
            }
            else
            {
                drive(OBST_SEEK_SPEED_MM_S, g_o.b_have_pose ? g_o.h0 : p_odom->heading_cdeg, p_odom);
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
