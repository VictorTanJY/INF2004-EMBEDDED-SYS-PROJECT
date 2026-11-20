/** @file obstacle_task.c
 *
 *  @brief Subsystem 5 -- ObstacleTask (priority 9, core 1), Week 6 Section 6.4.
 *  Owner: Buddy 5 (Ryan).
 *
 *  Blocks only on flg_obst (TWF_ORW | TWF_BITCLR, 100 ms timeout so it
 *  heartbeats even when idle): RANGE every 60 ms from range_cyc, ECHO from
 *  echo_isr, REQ after each mbf_scan request.  One ping per 60 ms slot.
 *
 *  MONITOR   servo 90 deg, median of 3: < slow_mm SLOW_DOWN, < stop_mm
 *  OBSTACLE (both masked on a hump), < 8 cm TOO_CLOSE (never masked).
 *  PROFILE   coarse 30..150 by 30, fine +-40 by 5 around the nearest echo,
 *  two bisection pings per edge -> width, edges, clearances, plan.
 *  4 Nov, for a bypass: EDGE_SCAN, sensor straight ahead, the car
 *  turns 5 deg at a time to each side until the echo leaves the
 *  face; the edges from the car's heading re-plan the bypass
 *  -> PROFILE_DONE(plan, width).  No clearance: reverse 100 mm and
 *  rescan (twice), then BLOCKED: rescan every 2 s, 5 times; after
 *  10 s BLOCKED_TIMEOUT.
 *  BYPASS    turn out 45, offset, turn parallel, PASS (servo looking at the
 *  obstacle until it has been seen and passed), turn in 45, JOIN
 *  (on until a line sensor touches the line), OBST_IR_AHEAD_MM on,
 *  LINESCAN (spin slowly until the line is centred under the
 *  sensors) -> BYPASS_DONE.  3 Nov: bypass and sweep
 *  turns are scaled by the turn slip measured at calibration.
 *  Commands use AVOID authority; completion is read from last_done_seq in the
 *  odometry snapshot. RECOVER   creep forward (<= 0.5 m) until a line sensor
 *  sees black within
 *  +-60 mm of the original path and +-20 deg of its heading, align
 *  -> RECOVERY_DONE; else sweep +-45 deg on the spot and take any
 *  line -> RECOVERY_DONE, or RECOVERY_FAILED.
 *  3 Nov (instructor's guidance): every scan angle is the median of
 *  OBST_SCAN_PINGS pings after the servo settles; an unsure profile (see
 *  mission_cfg.h) backs off OBST_SECOND_LOOK_MM and scans once more.
 */

#include "obstacle_task.h"
#include "subsystem5.h"
#include "mission_cfg.h"
#include "obstacle_core.h"
#include "mission_core.h" /* mission_core_scan_req_unpack (CR1) */
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "../app_types.h"
#include "../app_math.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

#define OBST_LINE_SENSORS 2u /* left and right line sensors */

#define OBST_WAIT_TIMEOUT_MS 100
#define OBST_SIDE_SERVO_DEG  75 /* PASS: look 75 deg to the obstacle's side */
#define OBST_HOLD_K          2  /* heading hold: mrad/s per cdeg of error */
#define OBST_HOLD_MAX        1500
#define OBST_ALIGN_TOL_CDEG  300

/* after CONTINUE: no new OBSTACLE for this far */
#define OBST_CONTINUE_IGNORE_MM 300
#define OBST_TOO_CLOSE_RESET_MM 120u

/* Event-history diagnostics (3 Nov) */
/* bypass plan: offset mm (+ left) << 16 | pass mm */
#define OBST_LOG_PLAN 39u
#define OBST_LOG_PASS 40u /* pass leg: driven mm | side ping saw it << 16 */
#define OBST_LOG_JOIN 44u /* back in: line touched after mm, 0 = not found */

/* turned onto the line after 0.1 deg (< 0: one sensor only), 0 = not found */
#define OBST_LOG_LINESCAN 46u

/* scan edges, mm (+ left): left << 16 | right (int16 each) */
#define OBST_LOG_EDGES     47u
#define OBST_LOG_EDGES_CAR 48u /* the same from the edge scan (car turns) */

/* edge scan: hits L << 24 | last deg L << 16 | hits R << 8 | last deg R */
#define OBST_LOG_EDGE_SIDES 49u

/* edge scan: face mm straight ahead << 16 | first left ping mm */
#define OBST_LOG_EDGE_FACE 50u

/* edge scan, last left echo: heading 0.01 deg << 16 | range mm */
#define OBST_LOG_EDGE_RAW_L 71u
#define OBST_LOG_EDGE_RAW_R 72u /* the same on the right (heading < 0) */

/* width probe: left edge mm << 16 | right edge (int16 each, + left);
   0x7FFF7FFF none */
#define OBST_LOG_PROBE 74u

/* step-out: face edge seen at mm (0xFFFF none) << 16 | stepped out mm */
#define OBST_LOG_OUT 60u

/* raw servo-scan point (20 Nov): coarse << 18 | servo deg << 10 | range mm
   (OBST_SCAN_PT_NONE = no echo or beyond 1022 mm) */
#define OBST_LOG_SCAN_PT  85u
#define OBST_SCAN_PT_NONE 1023u

/* line scan phases */
#define LS_PH_SWEEP 0u
#define LS_PH_ALIGN 1u

/* edge scan phases */
#define EDGE_PH_TURN   0u
#define EDGE_PH_SETTLE 1u
#define EDGE_PH_PING   2u
#define EDGE_PH_RETURN 3u

/* width probe (7 Nov) */
#define PB_FACE        0u /* servo straight, median ping of the face */
#define PB_MOVE        1u /* to OBST_PROBE_FACE_MM from it            */
#define PB_TURN        2u /* turn OBST_PROBE_DDEG left                */
#define PB_SERVO       3u /* servo back along the line                */
#define PB_LEFT        4u /* forward: past the left edge              */
#define PB_RIGHT       5u /* back: past the right edge                */
#define PB_BACK        6u /* forward to where it turned               */
#define PB_UNTURN      7u /* turn back to the line's heading          */
#define OBST_GAIN_UNIT 1000

typedef struct
{
    uint8_t state; /**< mission_types_obst_state_t */
    /** servo */
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
    bool                 b_slow_sent;
    bool                 b_obst_sent;
    bool                 b_too_close;
    int32_t              ignore_until_mm;
    obstacle_core_mask_t mask;
    /* pose at the obstacle */
    bool    b_have_pose;
    int32_t x0_mm;
    int32_t y0_mm;
    int32_t h0_cdeg;
    /** planning / blocked */
    uint8_t  retries;
    uint8_t  rescans;
    bool     b_blocked; /**< in the BLOCKED wait (rescans keep it set) */
    uint32_t blocked_start_ms;
    uint32_t next_rescan_ms;
    /** motion steps */
    uint8_t n_steps;
    uint8_t step_i;
    uint8_t side; /**< OBST_PLAN_LEFT / RIGHT */
    uint8_t seq;
    bool    b_wait_motion;
    uint8_t wait_fail_count;
    /** pass / seek */
    int32_t  phase_start_mm;
    int32_t  last_seen_mm;
    bool     b_seen;
    int16_t  pass_limit_mm;
    uint8_t  drive_seq;
    uint32_t pings;
    /** 3 Nov */
    uint16_t
            scan_pings[OBST_SCAN_PINGS]; /**< pings at the current scan angle */
    uint8_t sw_n;
    bool    b_second_look; /**< this obstacle was already scanned twice */
    uint8_t sweep_i;       /**< SEEK_SWEEP step */
    int32_t join_h;        /**< JOIN: heading held while driving back in */
    bool    b_joined;      /**< the line was found: accept it in SEEK    */
    uint8_t scan_pass; /**< LINESCAN: 0 not started, 1 first sweep, 2 second */
    uint8_t scan_ph;   /**< LINESCAN: LS_PH_* */
    uint8_t ls_seen[OBST_LINE_SENSORS]; /**< per sensor: 0 not yet, 1 on,
                                           OBST_LINE_SENSORS on and off */
    bool b_ls_prev[OBST_LINE_SENSORS];  /**< per sensor: on the line at the last
                                           slot  */
    int32_t  ls_on[2];      /**< heading (from scan_h0) it went on, 0.01 deg */
    int32_t  ls_off[2];     /**< ... and off */
    int32_t  scan_target;   /**< the line's heading: midway between them   */
    uint32_t scan_until_ms; /**< settle before the turn to it              */
    bool     b_scan_turned; /**< the turn to it has been sent              */
    uint8_t  ls_part_q;     /**< 7 Nov: best one-sensor crossing: 0 none,
                               1 went on only, 2 on and off             */
    int32_t ls_part_h;      /**< ... its heading (from scan_h0)             */
    bool    b_scan_part;    /**< turning to a one-sensor crossing           */
    /** edge scan (4 Nov) */
    int8_t   edge_side; /**< +1 left, -1 right */
    uint8_t  edge_ph;   /**< EDGE_PH_* */
    uint8_t  edge_miss;
    int32_t  edge_h0; /**< encoder heading where the car stopped */
    uint32_t edge_until_ms;
    bool     b_edge_has[2]; /**< [0] left, [1] right */
    int32_t  edge_lat[2];   /**< wide edge (side choice)           */
    int32_t  edge_min[2]; /**< sensor-position edge (7 Nov): step-out minimum */
    int32_t  edge_phi[2]; /**< 7 Nov: heading of the last echo, 0.01 deg  */
    uint16_t edge_r[2];   /**< ... and its range                          */
    int32_t  edge_lo;     /**< refine: last echo heading                  */
    int32_t  edge_hi;     /**< refine: first miss heading                 */
    uint8_t  edge_ref;    /**< refine iteration, 0 = coarse steps         */
    int16_t  edge_step;   /**< next turn, 0.1 deg (+ left)                */
    uint8_t  edge_hits[2]; /**< on-face pings per side (log)  */
    uint8_t  edge_deg[2];  /**< last heading reached, deg (log) */
    uint16_t edge_face;    /**< face range straight ahead, measured first */
    uint16_t edge_first_l; /**< first ping on the left (log)             */
    bool     b_edge_face;  /**< edge_face measured */
    int32_t  scan_h0;      /**< LINESCAN: heading at the start           */
    uint16_t det_mm;       /**< range when the monitor saw the obstacle  */
    int32_t  back_mm;      /**< backed off since then (second look, reverse) */
    /** width probe (7 Nov) */
    uint8_t probe_ph;      /**< PB_*                                       */
    bool    b_pb_moved;    /**< the approach move has been made            */
    bool    b_pb_seen;     /**< face seen on this pass                     */
    int32_t probe_h1;      /**< heading held while side-stepping           */
    int32_t probe_face_mm; /**< face ahead of the axle before the turn     */
    int32_t probe_lat;     /**< axle sideways (+ left) at the last slot    */
    obstacle_core_probe_t probe;
    /* step-out leg (7 Nov) */
    obstacle_core_out_t out;
    int32_t out_face_mm; /**< face ahead of the axle at the turn-out    */
    int32_t out_h;       /**< heading held while stepping out            */
} obst_ctx_t;

static obstacle_core_scan_t    g_scan;
static obstacle_core_profile_t g_prof;
static obstacle_core_step_t    g_steps[OBST_BYPASS_MAX_STEPS];

static obst_ctx_t g_o;

static void     servo_to(int32_t deg, uint32_t now_ms);
static bool     servo_settled(uint32_t now_ms);
static void     ping(void);
static void     post(uint8_t type, uint16_t arg, int32_t data);
static void     log_ev(uint16_t code, int32_t data);
static int32_t  turn_gain(void);
static int32_t  out_theta(motion_types_odom_t const * p_odom, int32_t sign);
static int16_t  slip_ddeg(int16_t ddeg);
static void     goto_state(uint8_t state);
static void     drive(int32_t speed, int32_t target_h,
                      motion_types_odom_t const * p_odom);
static void     stop_motion(void);
static void     send_step(uint8_t kind, int16_t value,
                          motion_types_odom_t const * p_odom);
static uint16_t scan_median(void);
static bool     scan_sample(uint16_t range, uint16_t * p_r);
static void     send_turn(int16_t ddeg, uint16_t rate_deg_s,
                          motion_types_odom_t const * p_odom);
static int32_t  motion_result(motion_types_odom_t const * p_odom);
static void     store_pose(motion_types_odom_t const * p_odom);
static void     start_scan(uint32_t now_ms);
static void     publish_profile(void);
static void     log_scan(void);
static void log_scan_point(obstacle_core_point_t const * p_pt, bool b_coarse);
static void plan(uint32_t now_ms, motion_types_odom_t const * p_odom);
static void edge_finish(void);
static void probe_finish(uint32_t now_ms);
static int32_t probe_theta(motion_types_odom_t const * p_odom);
static void    probe_slot(uint32_t now_ms, motion_types_odom_t const * p_odom);
static void probe_face_ping(uint16_t range, motion_types_odom_t const * p_odom);
static void probe_side_ping(uint16_t range, int32_t lat, int32_t fwd,
                            int32_t theta);
static void probe_range(uint16_t range, uint32_t now_ms,
                        motion_types_odom_t const * p_odom);
static void start_bypass(uint8_t type, uint32_t now_ms);
static void handle_request(mission_types_scan_req_t const * p_req,
                           uint32_t now_ms, motion_types_odom_t const * p_odom);
static void handle_range(uint16_t range, uint32_t now_ms,
                         motion_types_odom_t const * p_odom);
static void slot(uint32_t now_ms, motion_types_odom_t const * p_odom);
static void edge_slot(uint32_t now_ms, motion_types_odom_t const * p_odom);
static void edge_return(motion_types_odom_t const * p_odom);
static void blocked_slot(uint32_t now_ms);
static void bypass_slot(uint32_t now_ms, motion_types_odom_t const * p_odom);
static void bypass_out(uint32_t now_ms, motion_types_odom_t const * p_odom,
                       obstacle_core_step_t const * p_st);
static void out_start(uint32_t now_ms, motion_types_odom_t const * p_odom,
                      int32_t theta, int32_t sign);
static void out_end(uint32_t now_ms, obstacle_core_step_t const * p_st,
                    int32_t done_mm, int32_t lat, int32_t theta);
static void bypass_pass(uint32_t now_ms, motion_types_odom_t const * p_odom,
                        obstacle_core_step_t const * p_st);
static void bypass_join(motion_types_odom_t const *  p_odom,
                        obstacle_core_step_t const * p_st);
static void bypass_linescan(uint32_t now_ms, motion_types_odom_t const * p_odom,
                            obstacle_core_step_t const * p_st);
static void linescan_align(uint32_t now_ms, motion_types_odom_t const * p_odom,
                           int32_t rel);
static void linescan_next_sweep(uint32_t                     now_ms,
                                motion_types_odom_t const *  p_odom,
                                obstacle_core_step_t const * p_st, int32_t rel);
static void linescan_watch(line_types_snap_t const * p_line, int32_t rel);
static void linescan_partial(uint8_t quality, int32_t heading);
static void bypass_move(motion_types_odom_t const *  p_odom,
                        obstacle_core_step_t const * p_st);
static void seek_line_slot(motion_types_odom_t const * p_odom);
static void seek_sweep_slot(motion_types_odom_t const * p_odom);
static void align_slot(motion_types_odom_t const * p_odom);
static void monitor_range(uint16_t range, uint32_t now_ms,
                          motion_types_odom_t const * p_odom);
static void monitor_too_close(void);
static void monitor_events(mission_types_snap_t const * p_mission,
                           motion_types_odom_t const *  p_odom);
static void edge_range(uint16_t range_min, motion_types_odom_t const * p_odom);
static bool edge_classify(uint8_t side_idx, int32_t phi, uint16_t range_min);
static void edge_next_phase(uint8_t side_idx, int32_t phi, bool b_past);
static int16_t edge_next_step(int32_t phi);
static void    bypass_range(uint16_t range, uint32_t now_ms,
                            motion_types_odom_t const * p_odom);
static void    plan_edges(uint32_t now_ms, motion_types_odom_t const * p_odom);
static void probe_motion(uint32_t now_ms, motion_types_odom_t const * p_odom);
static void echo_missed(uint32_t now_ms, motion_types_odom_t const * p_odom);

EXPORT void obstacle_task_thread (INT stacd, void * p_exinf)
{
    mission_types_scan_req_t req;
    uint16_t            req_word; /* mbf_scan: one packed half-word (CR1) */
    motion_types_odom_t odom;
    mission_types_obst_snap_t snap;

    (void)stacd;
    (void)p_exinf;

    (void)drv_ultrasonic_init();
    (void)drv_servo_init();
    (void)drv_ultrasonic_attach(); /* echo_isr on this core (core 1) */
    g_o.servo_deg = OBST_SERVO_CENTRE_DEG;
    g_o.state     = OBST_ST_IDLE;
    g_o.d_mm      = OBST_RANGE_NONE;

    for (;;)
    {
        UINT     ptn = 0u;
        uint32_t now_ms;
        uint32_t width;

        (void)tk_wai_flg(subsystem5_flg_obst(),
                         OBST_FLG_RANGE | OBST_FLG_ECHO | OBST_FLG_REQ,
                         TWF_ORW | TWF_BITCLR, &ptn, OBST_WAIT_TIMEOUT_MS);
        subsystem4_heartbeat(SAFETY_HB_OBSTACLE);
        now_ms = app_time_ms();
        subsystem2_get_odom(&odom);

        if ((ptn & OBST_FLG_REQ) != 0u)
        {
            /* Cast (INT)sizeof(req_word): a constant that fits. */
            while ((INT)sizeof(req_word)
                   == tk_rcv_mbf(subsystem5_mbf_scan(), &req_word, TMO_POL))
            {
                mission_core_scan_req_unpack(req_word, &req);
                handle_request(&req, now_ms, &odom);
            }
        }

        if (drv_ultrasonic_take(&width))
        {
            g_o.b_ping_out = false;
            g_o.no_echo    = 0u;

            if (g_o.b_no_echo_set)
            {
                /* Cast (UINT)SAFETY_FLT_NO_ECHO: a constant that fits. */
                g_o.b_no_echo_set = false;
                (void)tk_clr_flg(subsystem4_flg_safety(),
                                 ~(UINT)SAFETY_FLT_NO_ECHO);
            }

            handle_range(obstacle_core_echo_to_mm(width), now_ms, &odom);
        }

        if ((ptn & OBST_FLG_RANGE) != 0u)
        {
            if (g_o.b_ping_out)
            {
                echo_missed(now_ms, &odom);
            }

            slot(now_ms, &odom);
        }

        subsystem5_get_obst_snap(&snap);
        snap.t_ms    = now_ms;
        snap.d_mm    = g_o.d_mm;
        snap.state   = g_o.state;
        snap.retries = g_o.retries;
        snap.no_echo = g_o.no_echo;
        snap.pings   = g_o.pings;
        subsystem5_put_obst_snap(&snap);
    }
}

/** No echo at all in a whole slot: the sensor is silent (report only). */
static void echo_missed (uint32_t now_ms, motion_types_odom_t const * p_odom)
{
    g_o.b_ping_out = false;

    if (g_o.no_echo < 0xFFFFu)
    {
        g_o.no_echo++;
    }

    if ((g_o.no_echo >= OBST_NO_ECHO_PINGS) && !g_o.b_no_echo_set)
    {
        g_o.b_no_echo_set = true;
        (void)tk_set_flg(subsystem4_flg_safety(), SAFETY_FLT_NO_ECHO);
    }

    handle_range(OBST_RANGE_NONE, now_ms, p_odom);
}

/* ------------------------------------------------------------------ */
/** Turn the servo to deg and note when it will have settled. */
static void servo_to (int32_t deg, uint32_t now_ms)
{
    int32_t const delta = app_types_abs_i32(deg - g_o.servo_deg);

    if (delta != 0)
    {
        /* Cast (uint32_t)((delta * ...)): delta is a servo move of at most 180
           deg, so the settle time is a few hundred ms. */
        drv_servo_set_deg(deg);
        g_o.servo_deg       = deg;
        g_o.settle_until_ms = now_ms
                              + (uint32_t)((delta * OBST_SERVO_MS_PER_DEG)
                                           + OBST_SERVO_SETTLE_MS);
    }
}

/** True once the servo has had time to reach its last position. */
static bool servo_settled (uint32_t now_ms)
{
    /* Cast (int32_t)(now_ms - ...): signed difference of two wrapping time
       stamps; they are less than 2^31 apart. */
    return ((int32_t)(now_ms - g_o.settle_until_ms) >= 0);
}

/** Send one ultrasonic ping; the echo arrives through echo_isr. */
static void ping (void)
{
    drv_ultrasonic_trigger();
    g_o.b_ping_out = true;
    g_o.pings++;
}

/** Post an event to MissionTask (source: ObstacleTask). */
static void post (uint8_t type, uint16_t arg, int32_t data)
{
    (void)subsystem5_mission_post(type, APP_SRC_OBST_MISSION, arg, data);
}

/** Post a telemetry LOG record (source: ObstacleTask). */
static void log_ev (uint16_t code, int32_t data)
{
    (void)subsystem1_telem_post(COMMS_KIND_LOG, APP_SRC_OBST_MISSION, code,
                                data);
}

/** A real turn (0.1 deg) as the encoders must count it: x the turn slip
   gain measured in the calibration spin (line snapshot, 0 = not measured). */
static int32_t turn_gain (void)
{
    line_types_snap_t line;

    subsystem3_get_snap(&line);
    return (line.turn_gain_pm != 0u) ? line.turn_gain_pm : OBST_GAIN_UNIT;
}

/** Step-out leg: the car's real heading from the line toward side s,
   0.01 deg (encoder heading corrected by the measured turn slip). */
static int32_t out_theta (motion_types_odom_t const * p_odom, int32_t sign)
{
    int32_t const raw =
        sign * app_math_wrap_cdeg(p_odom->heading_cdeg - g_o.h0_cdeg);

    return app_types_clamp_i32((raw * OBST_GAIN_UNIT) / turn_gain(), 0, 9000);
}

/** A real turn (0.1 deg) as the encoders must count it, corrected by the
    measured turn slip. */
static int16_t slip_ddeg (int16_t ddeg)
{
    int32_t const gain = turn_gain();

    /* Casts (int16_t)app_types_clamp_i32(...): the value is -3600..3600;
       (int32_t)ddeg: every int16_t value fits. */
    return (int16_t)app_types_clamp_i32(((int32_t)ddeg * gain) / OBST_GAIN_UNIT,
                                        -3600, 3600);
}

/** Change ObstacleTask's state; no motion is awaited in the new one. */
static void goto_state (uint8_t state)
{
    g_o.state         = state;
    g_o.b_wait_motion = false;
}

/** DRIVE at speed mm/s, steering to hold heading target_h (0.01 deg). */
static void drive (int32_t speed, int32_t target_h,
                   motion_types_odom_t const * p_odom)
{
    int32_t const err = target_h - p_odom->heading_cdeg;
    int32_t const omega =
        app_types_clamp_i32(err * OBST_HOLD_K, -OBST_HOLD_MAX, OBST_HOLD_MAX);

    /* Casts (int16_t)speed: speed is one of the OBST_*_SPEED_MM_S constants
       (<= 60 mm/s); (int16_t)omega: omega is clamped to +-OBST_HOLD_MAX just
       above. */
    g_o.drive_seq = motion_types_seq_next(g_o.drive_seq);
    (void)motion_api_drive(MOTION_SRC_AVOID, g_o.drive_seq, (int16_t)speed,
                           (int16_t)omega);
}

/** Ask MotionTask to stop (bypass source). */
static void stop_motion (void)
{
    (void)motion_api_stop(MOTION_SRC_AVOID, 0u);
}

/** Send one bypass step (MOVE or TURN by value) and wait for its completion. */
static void send_step (uint8_t kind, int16_t value,
                       motion_types_odom_t const * p_odom)
{
    /* Cast (uint16_t)app_types_abs_i32(value): the value is 0..32768. */
    uint16_t const amount = (uint16_t)app_types_abs_i32(value);

    g_o.seq             = motion_types_seq_next(g_o.seq); /* 5-bit seq (CR1) */
    g_o.wait_fail_count = p_odom->fail_count;
    g_o.b_wait_motion   = true;

    if (OBST_STEP_TURN == kind)
    {
        if (value >= 0)
        {
            (void)motion_api_turn_left(MOTION_SRC_AVOID, g_o.seq, amount,
                                       OBST_BYPASS_TURN_DEG_S);
        }
        else
        {
            (void)motion_api_turn_right(MOTION_SRC_AVOID, g_o.seq, amount,
                                        OBST_BYPASS_TURN_DEG_S);
        }
    }
    else
    {
        if (value >= 0)
        {
            (void)motion_api_move_forward(MOTION_SRC_AVOID, g_o.seq, amount,
                                          OBST_BYPASS_SPEED_MM_S);
        }
        else
        {
            (void)motion_api_move_backward(MOTION_SRC_AVOID, g_o.seq, amount,
                                           OBST_BYPASS_SPEED_MM_S);
        }
    }
}

/** Median of the echoes that came back at one scan angle (OBST_RANGE_NONE
   only if none did).  3 Nov: a round bottle answered 1 ping in 3, and a
   median that counted misses as "far" threw that echo away. */
static uint16_t scan_median (void)
{
    uint16_t sorted[OBST_SCAN_PINGS];
    uint8_t  n_pings = 0u;
    uint8_t  idx;
    uint8_t  jdx;

    for (idx = 0u; idx < g_o.sw_n; idx++)
    {
        if (g_o.scan_pings[idx] != OBST_RANGE_NONE)
        {
            sorted[n_pings] = g_o.scan_pings[idx];

            for (jdx = n_pings; (jdx > 0u) && (sorted[jdx - 1u] > sorted[jdx]);
                 jdx--)
            {
                uint16_t const swap = sorted[jdx - 1u];

                sorted[jdx - 1u] = sorted[jdx];
                sorted[jdx]      = swap;
            }

            n_pings++;
        }
    }

    return (0u == n_pings) ? OBST_RANGE_NONE : sorted[(n_pings - 1u) / 2u];
}

/** One ping result at a scan angle: true once OBST_SCAN_PINGS are in
   (*p_r = their median). */
static bool scan_sample (uint16_t range, uint16_t * p_r)
{
    bool b_full;

    g_o.scan_pings[g_o.sw_n] = range;
    g_o.sw_n++;
    b_full = (g_o.sw_n >= OBST_SCAN_PINGS);

    if (b_full)
    {
        *p_r     = scan_median();
        g_o.sw_n = 0u;
    }

    return b_full;
}

/** Send a TURN of ddeg (0.1 deg, + left) at rate_deg_s and wait for its
    completion. */
static void send_turn (int16_t ddeg, uint16_t rate_deg_s,
                       motion_types_odom_t const * p_odom)
{
    /* Cast (uint16_t)app_types_abs_i32(ddeg): the value is 0..32768. */
    uint16_t const amount = (uint16_t)app_types_abs_i32(ddeg);

    g_o.seq             = motion_types_seq_next(g_o.seq); /* 5-bit seq (CR1) */
    g_o.wait_fail_count = p_odom->fail_count;
    g_o.b_wait_motion   = true;

    if (ddeg >= 0)
    {
        (void)motion_api_turn_left(MOTION_SRC_AVOID, g_o.seq, amount,
                                   rate_deg_s);
    }
    else
    {
        (void)motion_api_turn_right(MOTION_SRC_AVOID, g_o.seq, amount,
                                    rate_deg_s);
    }
}

/** 1 = done, -1 = failed, 0 = still running */
static int32_t motion_result (motion_types_odom_t const * p_odom)
{
    int32_t result = 0;

    if ((p_odom->last_done_seq == g_o.seq)
        && (MOTION_SRC_AVOID == p_odom->last_done_src))
    {
        g_o.b_wait_motion = false;
        result            = 1;
    }
    else if ((p_odom->fail_count != g_o.wait_fail_count)
             && (p_odom->last_fail_seq == g_o.seq))
    {
        g_o.b_wait_motion = false;
        result            = -1;
    }
    else
    {
        /* still moving */
    }

    return result;
}

/** Remember the pose where the obstacle was found, for SEEK_LINE and ALIGN. */
static void store_pose (motion_types_odom_t const * p_odom)
{
    g_o.x0_mm       = p_odom->x_mm;
    g_o.y0_mm       = p_odom->y_mm;
    g_o.h0_cdeg     = p_odom->heading_cdeg;
    g_o.b_have_pose = true;
}

/** Start a servo scan: coarse sweep first.  20 Nov: the previous profile is
    withdrawn first, so telemetry reports no width until this scan's. */
static void start_scan (uint32_t now_ms)
{
    g_prof.b_valid = false;
    publish_profile();
    g_o.sw_n = 0u;
    obstacle_core_scan_start(&g_scan);
    goto_state(OBST_ST_COARSE_SCAN);
    servo_to(obstacle_core_scan_angle(&g_scan), now_ms);
}

/** Copy the obstacle profile into the obstacle snapshot. */
static void publish_profile (void)
{
    mission_types_obst_snap_t snap;

    subsystem5_get_obst_snap(&snap);
    snap.valid        = g_prof.b_valid ? 1u : 0u;
    snap.plan         = g_prof.plan;
    snap.w_mm         = g_prof.w_mm;
    snap.closest_mm   = g_prof.closest_mm;
    snap.clr_l_mm     = g_prof.clr_l_mm;
    snap.clr_r_mm     = g_prof.clr_r_mm;
    snap.left_lat_mm  = g_prof.left_lat_mm;
    snap.right_lat_mm = g_prof.right_lat_mm;
    subsystem5_put_obst_snap(&snap);
}

/** Every point of the servo scan that just finished, as LOG records (20 Nov:
    the raw data behind the width estimate). */
static void log_scan (void)
{
    uint8_t idx;

    for (idx = 0u; idx < g_scan.n_coarse; idx++)
    {
        log_scan_point(&g_scan.coarse[idx], true);
    }

    for (idx = 0u; idx < g_scan.n_fine; idx++)
    {
        log_scan_point(&g_scan.fine[idx], false);
    }
}

/** One scan point: servo angle (8 bits), range (10 bits), coarse flag. */
static void log_scan_point (obstacle_core_point_t const * p_pt, bool b_coarse)
{
    uint32_t const range =
        (p_pt->r_mm >= OBST_SCAN_PT_NONE) ? OBST_SCAN_PT_NONE : p_pt->r_mm;

    /* Cast (uint32_t)app_types_clamp_i32(...): the value is 0..255. */
    uint32_t const angle =
        (uint32_t)app_types_clamp_i32(p_pt->angle_deg, 0, 255);

    /* Cast (int32_t)(...): bit pattern of a packed LOG word (19 bits), not an
       arithmetic value. */
    log_ev(OBST_LOG_SCAN_PT,
           (int32_t)(((b_coarse ? 1uL : 0uL) << 18) | (angle << 10) | range));
}

/** The scan finished: decide (PLAN). */
static void plan (uint32_t now_ms, motion_types_odom_t const * p_odom)
{
    obstacle_core_profile_t const * const p_pr = &g_prof;

    /* Cast (uint16_t)app_types_clamp_i32(...): the value is 0..1000. */
    obstacle_core_scan_detected(
        &g_scan, g_o.det_mm,
        (uint16_t)app_types_clamp_i32(g_o.back_mm, 0, 1000), &g_prof);
    publish_profile();
    log_scan();

    if (p_pr->b_valid)
    {
        /* Casts (int32_t)(((uint32_t) ...)): bit pattern of a packed LOG word,
           not an arithmetic value; (uint32_t)(uint16_t)p_pr->left_lat_m ...:
           every uint16_t value fits; (uint16_t)p_pr->left_lat_mm,
           (uint16_t)p_pr->right_lat_mm: low 16 bits (two's complement) of the
           value, packed into a LOG word. */
        log_ev(OBST_LOG_EDGES,
               (int32_t)(((uint32_t)(uint16_t)p_pr->left_lat_mm << 16)
                         | (uint16_t)p_pr->right_lat_mm));
    }

    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);

    if (g_prof.b_valid && !g_prof.b_sure && !g_o.b_second_look
        && (0 == OBST_SINGLE_SWEEP))
    {
        /* unsure estimate: look again from a little further back once;
           the second profile plans with OBST_WIDTH_MAX if still unsure */
        g_o.b_second_look = true;
        goto_state(OBST_ST_REVERSE);
        g_o.back_mm += OBST_SECOND_LOOK_MM;
        send_step(OBST_STEP_MOVE, -OBST_SECOND_LOOK_MM, p_odom);
    }
    else if ((OBST_PLAN_LEFT == g_prof.plan) || (OBST_PLAN_RIGHT == g_prof.plan)
             || ((OBST_SINGLE_SWEEP != 0) && g_prof.b_valid
                 && (OBST_PLAN_BLOCKED == g_prof.plan)))
    {
        /* 9 Nov: with one sweep a "blocked" plan (no side clear enough by
           this scan's clearances) does not back off and sweep again: the
           side comes from the edges alone (the smaller step-out) and the
           step-out leg pings the face live */
        g_o.b_blocked = false;
        plan_edges(now_ms, p_odom);
    }
    else if (g_prof.plan != OBST_PLAN_BLOCKED)
    {
        g_o.b_blocked = false;

        if (OBST_PLAN_CONTINUE == g_prof.plan)
        {
            g_o.ignore_until_mm = p_odom->dist_mm + OBST_CONTINUE_IGNORE_MM;
        }

        post(MISSION_EV_PROFILE_DONE, g_prof.plan, g_prof.w_mm);
        goto_state(OBST_ST_IDLE);
    }
    else if (g_o.retries < OBST_REVERSE_TRIES)
    {
        g_o.retries++;
        goto_state(OBST_ST_REVERSE);
        g_o.back_mm += OBST_REVERSE_MM;
        send_step(OBST_STEP_MOVE, -OBST_REVERSE_MM, p_odom);
    }
    else
    {
        if (!g_o.b_blocked)
        {
            g_o.b_blocked        = true;
            g_o.blocked_start_ms = now_ms;
            g_o.rescans          = 0u;
        }

        goto_state(OBST_ST_BLOCKED);
        g_o.next_rescan_ms = now_ms + OBST_BLOCKED_RESCAN_MS;
    }
}

/** PLAN chose a side: measure the edges for the bypass first (or, with one
   sweep, bypass at once). */
static void plan_edges (uint32_t now_ms, motion_types_odom_t const * p_odom)
{
#if OBST_SINGLE_SWEEP
    /* 9 Nov: plan from this sweep, by the same side rule as the edge
       scans (the smaller step-out), and bypass at once */
    (void)now_ms; /* used by the edge scans only */
    (void)p_odom;
    {
        uint16_t const w_seen =
            g_prof.w_mm; /* beam-corrected width, for the report */

        obstacle_core_edge_plan(&g_prof, false, 0, false,
                                0); /* plans from the wider plan edges */
        g_prof.w_mm = w_seen;
    }
    g_o.b_edge_face = false;
    publish_profile();
    post(MISSION_EV_PROFILE_DONE, g_prof.plan, g_prof.w_mm);
    goto_state(OBST_ST_IDLE);
#elif OBST_WIDTH_PROBE
    /* 7 Nov: measure the edges by side-stepping past them */
    (void)now_ms; /* used by the edge scans only */
    goto_state(OBST_ST_WIDTH_PROBE);
    g_o.probe_ph      = PB_FACE;
    g_o.b_pb_moved    = false;
    g_o.sw_n          = 0u;
    g_o.edge_h0       = p_odom->heading_cdeg;
    g_o.edge_face     = g_prof.closest_mm;
    g_o.b_edge_face   = false;
    g_o.b_edge_has[0] = false;
    g_o.b_edge_has[1] = false;
    obstacle_core_probe_start(&g_o.probe);
#else
    /* 4 Nov: measure the edges for the bypass by turning the car */
    goto_state(OBST_ST_EDGE_SCAN);
    g_o.edge_side = 1;
    g_o.edge_ph =
        EDGE_PH_SETTLE; /* 4 Nov: first ping straight ahead for the face */
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
#endif
}

/** Edge scan finished: re-plan from the measured edges and report. */
static void edge_finish (void)
{
    obstacle_core_edge_plan(&g_prof, g_o.b_edge_has[0], g_o.edge_lat[0],
                            g_o.b_edge_has[1], g_o.edge_lat[1]);
    obstacle_core_edge_plan_min(&g_prof, g_o.b_edge_has[0], g_o.edge_min[0],
                                g_o.b_edge_has[1], g_o.edge_min[1]);

    if (g_o.b_edge_has[0] && g_o.b_edge_has[1])
    {
        /* 7 Nov: the reported width, with the calibrated beam angle */
        g_prof.w_mm = obstacle_core_edge_width(g_o.edge_phi[0], g_o.edge_r[0],
                                               g_o.edge_phi[1], g_o.edge_r[1]);
    }

    /* Casts (int32_t)(((uint32_t) ...)),
       (int32_t)(((uint32_t)g_o.edge_face ...)),
       (int32_t)(((uint32_t)g_o.edge_hits[ ...)): bit pattern of a packed LOG
       word, not an arithmetic value; (uint32_t)(uint16_t)g_o.edge_phi[0] ...,
       (uint32_t)(uint16_t)g_o.edge_phi[1] ..., (uint32_t)g_o.edge_face,
       (uint32_t)(uint16_t)g_prof.left_lat_ ...: every uint16_t value fits;
       (uint16_t)g_o.edge_phi[0], (uint16_t)g_o.edge_phi[1],
       (uint16_t)g_prof.left_lat_mm, (uint16_t)g_prof.right_lat_mm: low 16 bits
       (two's complement) of the value, packed into a LOG word;
       (uint32_t)g_o.edge_hits[0], (uint32_t)g_o.edge_deg[0],
       (uint32_t)g_o.edge_hits[1], (uint32_t)g_o.edge_deg[1]: every uint8_t
       value fits. */
    log_ev(OBST_LOG_EDGE_RAW_L,
           g_o.b_edge_has[0]
               ? (int32_t)(((uint32_t)(uint16_t)g_o.edge_phi[0] << 16)
                           | g_o.edge_r[0])
               : 0);
    log_ev(OBST_LOG_EDGE_RAW_R,
           g_o.b_edge_has[1]
               ? (int32_t)(((uint32_t)(uint16_t)g_o.edge_phi[1] << 16)
                           | g_o.edge_r[1])
               : 0);
    publish_profile();
    log_ev(OBST_LOG_EDGE_FACE,
           (int32_t)(((uint32_t)g_o.edge_face << 16) | g_o.edge_first_l));
    log_ev(OBST_LOG_EDGE_SIDES, (int32_t)(((uint32_t)g_o.edge_hits[0] << 24)
                                          | ((uint32_t)g_o.edge_deg[0] << 16)
                                          | ((uint32_t)g_o.edge_hits[1] << 8)
                                          | (uint32_t)g_o.edge_deg[1]));
    log_ev(OBST_LOG_EDGES_CAR,
           (int32_t)(((uint32_t)(uint16_t)g_prof.left_lat_mm << 16)
                     | (uint16_t)g_prof.right_lat_mm));
    post(MISSION_EV_PROFILE_DONE, g_prof.plan, g_prof.w_mm);
    goto_state(OBST_ST_IDLE);
}

/** Width probe finished (7 Nov): plan from the measured edges, report. */
static void probe_finish (uint32_t now_ms)
{
    int32_t    left  = 0;
    int32_t    right = 0;
    bool const b_ok  = obstacle_core_probe_edges(&g_o.probe, &left, &right);

    stop_motion();
    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);

    if (b_ok)
    {
        /* Casts (uint16_t)app_types_max_i32(left - ...): edges are within the
           800 mm profile range, so the width is 0..1600 mm;
           (int32_t)(((uint32_t) ...)): bit pattern of a packed LOG word, not an
           arithmetic value; (uint32_t)(uint16_t)left: every uint16_t value
           fits; (uint16_t)left, (uint16_t)right: low 16 bits (two's complement)
           of the value, packed into a LOG word. */
        obstacle_core_edge_plan(&g_prof, true, left, true, right);
        obstacle_core_edge_plan_min(&g_prof, true, left, true, right);
        g_prof.w_mm = (uint16_t)app_types_max_i32(left - right, 0);
        log_ev(OBST_LOG_PROBE,
               (int32_t)(((uint32_t)(uint16_t)left << 16) | (uint16_t)right));
    }
    else
    {
        obstacle_core_edge_plan(&g_prof, false, 0, false,
                                0); /* the servo scan's plan */
        log_ev(OBST_LOG_PROBE, 0x7FFF7FFF);
    }

    /* the bypass plans from where the face is now */
    g_prof.closest_mm = g_o.edge_face;
    g_o.b_edge_face   = true;
    publish_profile();
    post(MISSION_EV_PROFILE_DONE, g_prof.plan, g_prof.w_mm);
    goto_state(OBST_ST_IDLE);
}

/** Width probe: the car's real heading from the line (0.01 deg, + left). */
static int32_t probe_theta (motion_types_odom_t const * p_odom)
{
    return (app_math_wrap_cdeg(p_odom->heading_cdeg - g_o.edge_h0)
            * OBST_GAIN_UNIT)
           / turn_gain();
}

/** Width probe, one 60 ms slot. */
static void probe_slot (uint32_t now_ms, motion_types_odom_t const * p_odom)
{
    int32_t const theta = probe_theta(p_odom);
    int32_t const done  = p_odom->dist_mm - g_o.phase_start_mm;

    /* Casts (int32_t)(((int64_t)done * ...)): |sin|, |cos| <= 1.0 in Q15, so
       the result is no larger than the int32_t factor; (int64_t)done: every
       int32_t value fits. */
    g_o.probe_lat =
        (int32_t)(((int64_t)done * app_math_sin_q15(theta)) / 32768);

    switch (g_o.probe_ph)
    {
        case PB_FACE:
            if (servo_settled(now_ms) && !g_o.b_wait_motion)
            {
                ping(); /* median of OBST_SCAN_PINGS: handle_range */
            }
        break;

        case PB_MOVE:
        case PB_TURN:
        case PB_UNTURN:
            probe_motion(now_ms, p_odom);
        break;

        case PB_SERVO:
            if (servo_settled(now_ms))
            {
                g_o.probe_h1       = p_odom->heading_cdeg;
                g_o.phase_start_mm = p_odom->dist_mm;
                g_o.probe_lat      = 0;
                g_o.b_pb_seen      = false;
                g_o.probe.off      = 0u;
                g_o.probe_ph       = PB_LEFT;
            }
        break;

        case PB_LEFT:
        case PB_RIGHT:
            drive((PB_LEFT == g_o.probe_ph) ? OBST_PROBE_SPEED_MM_S
                                            : -OBST_PROBE_SPEED_MM_S,
                  g_o.probe_h1, p_odom);
            ping();
        break;

        default: /* PB_BACK: forward to where it turned */
            if (done >= 0)
            {
                stop_motion();
                g_o.probe_ph = PB_UNTURN;
            }
            else
            {
                drive(OBST_PROBE_SPEED_MM_S, g_o.probe_h1, p_odom);
            }
        break;
    }
}

/** Width probe, PB_MOVE / PB_TURN / PB_UNTURN: start the move or turn, or
   go on when it is done. */
static void probe_motion (uint32_t now_ms, motion_types_odom_t const * p_odom)
{
    if (g_o.b_wait_motion)
    {
        int32_t const res = motion_result(p_odom);

        if ((res != 0) && (PB_MOVE == g_o.probe_ph))
        {
            g_o.b_pb_moved = true;
            g_o.sw_n       = 0u;
            g_o.probe_ph   = PB_FACE; /* measure the face again */
        }
        else if ((res != 0) && (PB_TURN == g_o.probe_ph))
        {
            g_o.probe_ph = PB_SERVO;
            servo_to(obstacle_core_out_servo_deg(probe_theta(p_odom), 1),
                     now_ms);
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
    else if ((PB_TURN == g_o.probe_ph)
             && (MOTION_ST_IDLE_BRAKED == p_odom->state))
    {
        send_step(OBST_STEP_TURN, slip_ddeg(OBST_PROBE_DDEG), p_odom);
    }
    else if ((PB_UNTURN == g_o.probe_ph)
             && (MOTION_ST_IDLE_BRAKED == p_odom->state))
    {
        int32_t const dh_cdeg =
            app_math_wrap_cdeg(g_o.edge_h0 - p_odom->heading_cdeg);

        if (app_types_abs_i32(dh_cdeg) > 100)
        {
            /* Cast (int16_t)(dh_cdeg / 10): a wrapped heading difference is
               within +-18000 0.01 deg, so /10 fits. */
            send_step(OBST_STEP_TURN, (int16_t)(dh_cdeg / 10), p_odom);
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
}

/** Width probe, one range reading. */
static void probe_range (uint16_t range, uint32_t now_ms,
                         motion_types_odom_t const * p_odom)
{
    /* Casts (int32_t)(((int64_t)done * ...)): |sin|, |cos| <= 1.0 in Q15, so
       the result is no larger than the int32_t factor; (int64_t)done: every
       int32_t value fits. */
    int32_t const theta = probe_theta(p_odom);
    int32_t const done  = p_odom->dist_mm - g_o.phase_start_mm;
    int32_t const lat =
        (int32_t)(((int64_t)done * app_math_sin_q15(theta)) / 32768);
    int32_t const fwd =
        (int32_t)(((int64_t)done * app_math_cos_q15(theta)) / 32768);

    if (PB_FACE == g_o.probe_ph)
    {
        probe_face_ping(range, p_odom);
    }
    else if ((PB_LEFT == g_o.probe_ph) || (PB_RIGHT == g_o.probe_ph))
    {
        probe_side_ping(range, lat, fwd, theta);
    }
    else
    {
        /* no ping is needed in the other phases */
    }

    (void)now_ms;
}

/** probe_range() in PB_FACE: the face distance from the median of a
    sweep's pings; move to OBST_PROBE_FACE_MM first if it is not there. */
static void probe_face_ping (uint16_t range, motion_types_odom_t const * p_odom)
{
    uint16_t range_min;

    if (scan_sample(range, &range_min))
    {
        /* 9 Nov: only a ping near where the face should be is the face
           (02:59 the beam passed an off-centre box, hit something 20 cm
           further, and the approach drove into the box) */
        g_o.edge_face = obstacle_core_probe_face(range_min, g_o.edge_face);

        /* Cast (int32_t)g_o.edge_face: every uint16_t value fits. */
        if (!g_o.b_pb_moved
            && (app_types_abs_i32((int32_t)g_o.edge_face - OBST_PROBE_FACE_MM)
                > 15))
        {
            /* Casts (int16_t)((int32_t)g_o.edge_face - ...): edge_face is a
               range below 4000 mm; (int32_t)g_o.edge_face: every uint16_t value
               fits. */
            g_o.probe_ph = PB_MOVE;
            send_step(OBST_STEP_MOVE,
                      (int16_t)((int32_t)g_o.edge_face - OBST_PROBE_FACE_MM),
                      p_odom);
            g_o.edge_face = OBST_PROBE_FACE_MM; /* where the face should be
                                                   after the move */
        }
        else
        {
            /* Casts (int32_t)OBST_US_AHEAD_MM: a constant that fits;
               (int32_t)g_o.edge_face: every uint16_t value fits. */
            g_o.probe_face_mm =
                (int32_t)OBST_US_AHEAD_MM + (int32_t)g_o.edge_face;
            g_o.probe_ph = PB_TURN;
        }
    }
}

/** probe_range() in PB_LEFT / PB_RIGHT: one ping of the side probe at
    lateral offset lat and forward offset fwd (mm), car turned theta. */
static void probe_side_ping (uint16_t range, int32_t lat, int32_t fwd,
                             int32_t theta)
{
    if (obstacle_core_probe_ping(
            &g_o.probe, obstacle_core_probe_sensor_lat(lat, theta), range,
            obstacle_core_out_expect_mm(g_o.probe_face_mm, fwd, theta)))
    {
        g_o.b_pb_seen = true;
    }

    if (PB_LEFT == g_o.probe_ph)
    {
        /* past the left edge (or never on the face), or as far as any
           obstacle could reach, or too close to the face */
        if (((g_o.probe.off >= OBST_PROBE_OFF) && (lat >= 20))
            || (lat >= OBST_PROBE_MAX_MM)
            || obstacle_core_out_blocked(g_o.probe_face_mm, fwd, theta))
        {
            stop_motion();
            g_o.b_pb_seen = false;
            g_o.probe.off = 0u;
            g_o.probe_ph  = PB_RIGHT;
        }
    }
    else if ((g_o.b_pb_seen && (g_o.probe.off >= OBST_PROBE_OFF))
             || (lat <= -OBST_PROBE_MAX_MM))
    {
        stop_motion();
        g_o.probe_ph = PB_BACK;
    }
    else
    {
        /* still on the face, or not there yet */
    }
}

/* ------------------------------------------------------------------ */
/** handle_request(): bypass on the side of type (SCAN_REQ_BYPASS_LEFT or
    _RIGHT): the step plan from the profile, then OBST_ST_BYPASS. */
static void start_bypass (uint8_t type, uint32_t now_ms)
{
    g_o.side =
        (SCAN_REQ_BYPASS_LEFT == type) ? OBST_PLAN_LEFT : OBST_PLAN_RIGHT;

    if (g_prof.plan != g_o.side)
    {
        /* deviation from the profile is signed by the planned side */
        /* Cast (int16_t)((OBST_PLAN_LEFT == ...)): edge offsets are
           within +-800 mm plus fixed margins of a few hundred mm. */
        g_prof.deviation_mm =
            (int16_t)((OBST_PLAN_LEFT == g_o.side)
                          ? (g_prof.left_lat_mm + OBST_MARGIN_MM
                             + (OBST_ROBOT_WIDTH_MM / 2) + OBST_EDGE_ALLOW_MM)
                          : -(-g_prof.right_lat_mm + OBST_MARGIN_MM
                              + (OBST_ROBOT_WIDTH_MM / 2)
                              + OBST_EDGE_ALLOW_MM));
        g_prof.plan = g_o.side;
        /* 7 Nov: the step-out leg measures the rest */
        obstacle_core_edge_plan_min(&g_prof, g_o.b_edge_has[0], g_o.edge_min[0],
                                    g_o.b_edge_has[1], g_o.edge_min[1]);
    }

    g_o.n_steps   = obstacle_core_bypass_plan(&g_prof, g_o.side, g_steps);
    g_o.step_i    = 0u;
    g_o.b_joined  = false;
    g_o.scan_pass = 0u;
    g_o.scan_ph   = LS_PH_SWEEP;
    {
        uint8_t kdx;

        for (kdx = 0u; kdx < g_o.n_steps; kdx++)
        {
            if (OBST_STEP_PASS == g_steps[kdx].kind)
            {
                /* Casts (int32_t)(((uint32_t) ...)): bit pattern of a
                   packed LOG word, not an arithmetic value;
                   (uint32_t)(uint16_t)g_prof.deviation ...: every
                   uint16_t value fits; (uint16_t)g_prof.deviation_mm,
                   (uint16_t)g_steps[kdx].value: low 16 bits (two's
                   complement) of the value, packed into a LOG word. */
                log_ev(OBST_LOG_PLAN,
                       (int32_t)(((uint32_t)(uint16_t)g_prof.deviation_mm << 16)
                                 | (uint16_t)g_steps[kdx].value));
            }
        }
    }
    goto_state(OBST_ST_BYPASS);
    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
}

/** Carry out one request from MissionTask: monitor, profile, bypass left or
    right, or recover the line. */
static void handle_request (mission_types_scan_req_t const * p_req,
                            uint32_t now_ms, motion_types_odom_t const * p_odom)
{
    switch (p_req->type)
    {
        case SCAN_REQ_MONITOR:
            goto_state(OBST_ST_MONITOR);
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
            start_bypass(p_req->type, now_ms);
        break;

        case SCAN_REQ_RECOVER:
            goto_state(OBST_ST_SEEK_LINE);
            servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
            g_o.phase_start_mm = p_odom->dist_mm;
        break;

        default: /* STOP */
            if ((OBST_ST_BYPASS == g_o.state)
                || (OBST_ST_SEEK_LINE == g_o.state)
                || (OBST_ST_SEEK_SWEEP == g_o.state)
                || (OBST_ST_EDGE_SCAN == g_o.state)
                || (OBST_ST_WIDTH_PROBE == g_o.state))
            {
                stop_motion();
            }

            goto_state(OBST_ST_IDLE);
            servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
            g_o.b_have_pose = false;
        break;
    }
}

/** One range result (r = OBST_RANGE_NONE: no target or no echo). */
static void handle_range (uint16_t range, uint32_t now_ms,
                          motion_types_odom_t const * p_odom)
{
    switch (g_o.state)
    {
        case OBST_ST_MONITOR:
            monitor_range(range, now_ms, p_odom);
        break;

        case OBST_ST_WIDTH_PROBE:
            probe_range(range, now_ms, p_odom);
        break;

        case OBST_ST_EDGE_SCAN:
        {
            uint16_t range_min;

            if ((EDGE_PH_PING == g_o.edge_ph) && scan_sample(range, &range_min))
            {
                edge_range(range_min, p_odom);
            }
        }
        break;

        case OBST_ST_COARSE_SCAN:
        case OBST_ST_FINE_SCAN:
        {
            uint16_t range_min;

            /* false: more pings at this angle */
            if (scan_sample(range, &range_min))
            {
                obstacle_core_scan_feed(&g_scan, range_min);
                g_o.state = (OBST_SCAN_COARSE == g_scan.phase)
                                ? OBST_ST_COARSE_SCAN
                                : OBST_ST_FINE_SCAN;

                if (obstacle_core_scan_done(&g_scan))
                {
                    plan(now_ms, p_odom);
                }
                else
                {
                    servo_to(obstacle_core_scan_angle(&g_scan), now_ms);
                }
            }
        }
        break;

        case OBST_ST_BYPASS:
            bypass_range(range, now_ms, p_odom);
        break;

        default:
        break;
    }
}

/** MONITOR: filter the range, then raise TOO_CLOSE, SLOW_DOWN and
   OBSTACLE as needed. */
static void monitor_range (uint16_t range, uint32_t now_ms,
                           motion_types_odom_t const * p_odom)
{
    imu_types_snap_t     imu;
    mission_types_snap_t mission;
    bool                 b_masked;

    g_o.win[g_o.win_n % 3u] = range;
    g_o.win_n++;
    g_o.d_mm = (g_o.win_n >= 3u)
                   ? obstacle_core_median3(g_o.win[0], g_o.win[1], g_o.win[2])
                   : OBST_RANGE_NONE;
    monitor_too_close();

    subsystem4_get_imu_snap(&imu);
    subsystem5_get_mission_snap(&mission);
    b_masked = obstacle_core_hump_mask(
        &g_o.mask, (imu.pitch_valid != 0u) && (imu.b_ok != 0u), imu.pitch_cdeg,
        p_odom->dist_mm, now_ms);

    if (b_masked || (OBST_RANGE_NONE == g_o.d_mm))
    {
        if (g_o.b_slow_sent && (OBST_RANGE_NONE == g_o.d_mm))
        {
            g_o.b_slow_sent = false;
            post(MISSION_EV_SLOW_DOWN, 0u, -1); /* clear */
        }
    }
    else
    {
        monitor_events(&mission, p_odom);
    }
}

/** < 8 cm ahead: TOO_CLOSE, never masked (FR15).  Only while the sensor
   looks forward (+-30 deg): sideways it may see a wall. */
static void monitor_too_close (void)
{
    if ((g_o.d_mm != OBST_RANGE_NONE) && (g_o.d_mm < OBST_TOO_CLOSE_MM)
        && !g_o.b_too_close
        && (app_types_abs_i32(g_o.servo_deg - OBST_SERVO_CENTRE_DEG) <= 30))
    {
        g_o.b_too_close = true;
        (void)tk_set_flg(subsystem4_flg_safety(), SAFETY_FLT_TOO_CLOSE);
    }
    else if (g_o.b_too_close
             && ((OBST_RANGE_NONE == g_o.d_mm)
                 || (g_o.d_mm > OBST_TOO_CLOSE_RESET_MM)))
    {
        g_o.b_too_close = false;
    }
    else
    {
        /* no change */
    }
}

/** MONITOR, a valid unmasked range: SLOW_DOWN on / off, then OBSTACLE. */
static void monitor_events (mission_types_snap_t const * p_mission,
                            motion_types_odom_t const *  p_odom)
{
    if (!g_o.b_slow_sent && (g_o.d_mm < p_mission->slow_mm))
    {
        g_o.b_slow_sent = true;
        post(MISSION_EV_SLOW_DOWN, 0u, g_o.d_mm);
    }
    else if (g_o.b_slow_sent && (g_o.d_mm > (p_mission->slow_mm + 50u)))
    {
        g_o.b_slow_sent = false;
        post(MISSION_EV_SLOW_DOWN, 0u, -1);
    }
    else
    {
        /* no change */
    }

    if (!g_o.b_obst_sent && (g_o.d_mm < p_mission->stop_mm)
        && (p_odom->dist_mm >= g_o.ignore_until_mm))
    {
        g_o.b_obst_sent = true;
        store_pose(p_odom);
        g_o.det_mm = g_o.d_mm;
        post(MISSION_EV_OBSTACLE, 0u, g_o.d_mm);
    }
}

/** EDGE_SCAN, one median range at the current heading. */
static void edge_range (uint16_t range_min, motion_types_odom_t const * p_odom)
{
    uint8_t const side_idx = (g_o.edge_side > 0) ? 0u : 1u;
    int32_t const phi = (app_math_wrap_cdeg(p_odom->heading_cdeg - g_o.edge_h0)
                         * OBST_GAIN_UNIT)
                        / turn_gain();

    if (!g_o.b_edge_face)
    {
        /* 4 Nov: the face straight ahead, measured here, not the
           servo scan's nearest echo (one close stray made the
           real face look "behind" and the left got 0 echoes) */
        g_o.b_edge_face = true;

        if (range_min != OBST_RANGE_NONE)
        {
            g_o.edge_face = range_min;
        }

        g_o.edge_ph = EDGE_PH_TURN;
    }
    else
    {
        bool b_past;

        /* Cast (uint8_t)app_types_min_i32(...): the value is 0..255. */
        g_o.edge_deg[side_idx] =
            (uint8_t)app_types_min_i32(app_types_abs_i32(phi) / 100, 255);

        if ((0u == side_idx) && (0u == g_o.edge_first_l))
        {
            g_o.edge_first_l =
                (OBST_RANGE_NONE == range_min) ? 0xFFFFu : range_min;
        }

        b_past = edge_classify(side_idx, phi, range_min);
        edge_next_phase(side_idx, phi, b_past);

        if (EDGE_PH_TURN == g_o.edge_ph)
        {
            g_o.edge_step = edge_next_step(phi);
        }
    }
}

/** EDGE_SCAN: record an echo on the face, or count a miss.  Returns true
   when the echo is past the largest obstacle (clutter beyond it). */
static bool edge_classify (uint8_t side_idx, int32_t phi, uint16_t range_min)
{
    int32_t const lat =
        obstacle_core_edge_wide_lat(phi, range_min, g_o.edge_side);
    bool b_past = false;

    if (obstacle_core_edge_on_face(phi, range_min, g_o.edge_face)
        && !obstacle_core_edge_past_max(lat, g_o.edge_side))
    {
        g_o.b_edge_has[side_idx] = true;
        g_o.edge_lat[side_idx]   = lat;
        g_o.edge_min[side_idx]   = obstacle_core_edge_lat(phi);
        g_o.edge_phi[side_idx]   = phi;
        g_o.edge_r[side_idx]     = range_min;
        g_o.edge_lo              = phi;
        g_o.edge_miss            = 0u;
        g_o.edge_hits[side_idx]++;
    }
    else
    {
        if (obstacle_core_edge_on_face(phi, range_min, g_o.edge_face))
        {
            /* 4 Nov: echoes 32 cm away at 50 deg right (mat edge,
               floor) were taken as the box, edge 28.6 cm right of
               the line.  An edge further from the line than the
               largest obstacle is not this obstacle: stop here,
               that side's edge is not known. */
            b_past = true;
        }

        if ((0u == g_o.edge_miss) || (g_o.edge_ref > 0u))
        {
            g_o.edge_hi = phi; /* first miss (refine: the nearer one) */
        }

        g_o.edge_miss++;
    }

    return b_past;
}

/** EDGE_SCAN: after one reading, turn on, refine, or return. */
static void edge_next_phase (uint8_t side_idx, int32_t phi, bool b_past)
{
    if (g_o.edge_ref > 0u)
    {
        /* refining: next halfway, or done; the edge is taken
           halfway between the last echo and the nearest miss
           (the last echo alone always reads a little inside) */
        g_o.edge_phi[side_idx] = (g_o.edge_lo + g_o.edge_hi) / 2;
        g_o.edge_ref++;
        g_o.edge_ph = (g_o.edge_ref > OBST_EDGE_REFINE_ITERS) ? EDGE_PH_RETURN
                                                              : EDGE_PH_TURN;
    }
    else if (b_past || (g_o.edge_miss >= OBST_EDGE_MISSES)
             || (app_types_abs_i32(phi) >= (OBST_EDGE_MAX_DDEG * 10)))
    {
        /* 7 Nov: a side that ended on clear readings after an
           echo is refined between the last echo and the first
           miss; past-max (clutter) and the turn limit are not */
        g_o.edge_ref = (g_o.b_edge_has[side_idx] && !b_past
                        && (g_o.edge_miss >= OBST_EDGE_MISSES)
                        && (OBST_EDGE_REFINE_ITERS > 0u))
                           ? 1u
                           : 0u;
        g_o.edge_ph  = (g_o.edge_ref > 0u) ? EDGE_PH_TURN : EDGE_PH_RETURN;
    }
    else
    {
        g_o.edge_ph = EDGE_PH_TURN;
    }
}

/** EDGE_SCAN: size of the next turn, 0.1 deg. */
static int16_t edge_next_step (int32_t phi)
{
    int32_t step = g_o.edge_side * OBST_EDGE_STEP_DDEG;

    if (g_o.edge_ref > 0u)
    {
        /* halfway between the last echo and the miss; at
           least 1.5 deg (a smaller turn may not move the
           wheels and could read as a stall); the ping is
           tagged with the heading reached, not asked */
        step = (((g_o.edge_lo + g_o.edge_hi) / 2) - phi) / 10;

        if (app_types_abs_i32(step) < OBST_EDGE_MIN_TURN_DDEG)
        {
            step = (step >= 0) ? OBST_EDGE_MIN_TURN_DDEG
                               : -OBST_EDGE_MIN_TURN_DDEG;
        }
    }

    /* Cast (int16_t)step: step is at most OBST_EDGE_STEP_DDEG or half an edge
       interval (< 180 deg). */
    return (int16_t)step;
}

/** BYPASS: pings during OUT and PASS. */
static void bypass_range (uint16_t range, uint32_t now_ms,
                          motion_types_odom_t const * p_odom)
{
    /* Cast (int32_t)range: every uint16_t value fits. */
    if ((g_o.step_i < g_o.n_steps)
        && (OBST_STEP_OUT == g_steps[g_o.step_i].kind) && g_o.b_wait_motion
        && servo_settled(now_ms))
    {
        int32_t const sign  = (OBST_PLAN_LEFT == g_o.side) ? 1 : -1;
        int32_t const theta = out_theta(p_odom, sign);
        int32_t const done  = p_odom->dist_mm - g_o.phase_start_mm;

        /* Casts (int32_t)(((int64_t)done * ...)): |sin|, |cos| <= 1.0 in Q15,
           so the result is no larger than the int32_t factor; (int64_t)done:
           every int32_t value fits. */
        obstacle_core_out_ping(
            &g_o.out,
            (int32_t)(((int64_t)done * app_math_sin_q15(theta)) / 32768),
            (int32_t)(((int64_t)done * app_math_cos_q15(theta)) / 32768), theta,
            range, g_o.out_face_mm);
    }
    else if ((g_o.step_i < g_o.n_steps)
             && (OBST_STEP_PASS == g_steps[g_o.step_i].kind)
             && (range != OBST_RANGE_NONE)
             && ((int32_t)range
                 < (app_types_abs_i32(g_prof.deviation_mm) + 150)))
    {
        g_o.b_seen       = true; /* obstacle still beside us */
        g_o.last_seen_mm = p_odom->dist_mm;
    }
    else
    {
        /* No action required. */
    }
}

/** One 60 ms slot. */
static void slot (uint32_t now_ms, motion_types_odom_t const * p_odom)
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
            edge_slot(now_ms, p_odom);
        break;

        case OBST_ST_REVERSE:
        {
            int32_t const res = motion_result(p_odom);

            if (res != 0)
            {
                start_scan(now_ms); /* rescan from further back */
            }
        }
        break;

        case OBST_ST_BLOCKED:
            blocked_slot(now_ms);
        break;

        case OBST_ST_BYPASS:
            bypass_slot(now_ms, p_odom);
        break;

        case OBST_ST_SEEK_LINE:
            seek_line_slot(p_odom);
        break;

        case OBST_ST_SEEK_SWEEP:
            seek_sweep_slot(p_odom);
        break;

        case OBST_ST_ALIGN:
            align_slot(p_odom);
        break;

        default: /* IDLE, PLAN */
        break;
    }
}

/** EDGE_SCAN, one slot: turn, settle, ping, and finally turn back. */
static void edge_slot (uint32_t now_ms, motion_types_odom_t const * p_odom)
{
    switch (g_o.edge_ph)
    {
        case EDGE_PH_TURN:
            if (!g_o.b_wait_motion)
            {
                if (MOTION_ST_IDLE_BRAKED == p_odom->state)
                {
                    send_turn(slip_ddeg(g_o.edge_step), OBST_EDGE_TURN_DEG_S,
                              p_odom);
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
            /* Cast (int32_t)(now_ms - ...): signed difference of two wrapping
               time stamps; they are less than 2^31 apart. */
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

        default: /* EDGE_PH_RETURN: back to where it stopped */
            edge_return(p_odom);
        break;
    }
}

/** EDGE_PH_RETURN: turn back to the starting heading, then scan the other
   side or finish. */
static void edge_return (motion_types_odom_t const * p_odom)
{
    if (g_o.b_wait_motion)
    {
        (void)motion_result(p_odom); /* clears b_wait_motion when done */
    }
    else if (MOTION_ST_IDLE_BRAKED == p_odom->state)
    {
        int32_t const dh_cdeg =
            app_math_wrap_cdeg(g_o.edge_h0 - p_odom->heading_cdeg);

        if (app_types_abs_i32(dh_cdeg) > 100)
        {
            /* Cast (int16_t)(dh_cdeg / 10): a wrapped heading difference is
               within +-18000 0.01 deg, so /10 fits. */
            send_step(OBST_STEP_TURN, (int16_t)(dh_cdeg / 10), p_odom);
        }
        else if (g_o.edge_side > 0)
        {
            g_o.edge_side = -1;
            g_o.edge_miss = 0u;
            g_o.edge_ref  = 0u;
            g_o.edge_step = -OBST_EDGE_STEP_DDEG;
            g_o.edge_ph   = EDGE_PH_TURN;
        }
        else
        {
            edge_finish();
        }
    }
    else
    {
        /* still stopping */
    }
}

/** BLOCKED, one slot: give up after the timeout, or rescan now and then. */
static void blocked_slot (uint32_t now_ms)
{
    /* Cast (int32_t)(now_ms - ...): signed difference of two wrapping time
       stamps; they are less than 2^31 apart. */
    if ((now_ms - g_o.blocked_start_ms) >= OBST_BLOCKED_TIMEOUT_MS)
    {
        post(MISSION_EV_BLOCKED_TIMEOUT, 0u, 0);
        g_o.b_blocked = false;
        goto_state(OBST_ST_IDLE);
    }
    else if (((int32_t)(now_ms - g_o.next_rescan_ms) >= 0)
             && (g_o.rescans < OBST_BLOCKED_RESCANS))
    {
        g_o.rescans++;
        g_o.sw_n = 0u;
        obstacle_core_scan_start(&g_scan);
        g_o.state = OBST_ST_COARSE_SCAN; /* stays "blocked" on return */
        servo_to(obstacle_core_scan_angle(&g_scan), now_ms);
    }
    else
    {
        /* waiting */
    }
}

/** BYPASS, one slot: run the current step of the planned bypass. */
static void bypass_slot (uint32_t now_ms, motion_types_odom_t const * p_odom)
{
    if (g_o.step_i >= g_o.n_steps)
    {
        if (g_o.b_joined)
        {
            g_o.b_have_pose = false; /* the line was touched: SEEK takes it */
        }

        servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
        post(MISSION_EV_BYPASS_DONE, 0u, 0);
        goto_state(OBST_ST_IDLE);
    }
    else
    {
        obstacle_core_step_t const * p_st = &g_steps[g_o.step_i];

        if (OBST_STEP_OUT == p_st->kind)
        {
            bypass_out(now_ms, p_odom, p_st);
        }
        else if (OBST_STEP_PASS == p_st->kind)
        {
            bypass_pass(now_ms, p_odom, p_st);
        }
        else if (OBST_STEP_JOIN == p_st->kind)
        {
            bypass_join(p_odom, p_st);
        }
        else if (OBST_STEP_LINESCAN == p_st->kind)
        {
            bypass_linescan(now_ms, p_odom, p_st);
        }
        else
        {
            bypass_move(p_odom, p_st);
        }
    }
}

/** OBST_STEP_OUT: step out at the turned heading, the sensor looking along
   the line at the face, until clear of it. */
static void bypass_out (uint32_t now_ms, motion_types_odom_t const * p_odom,
                        obstacle_core_step_t const * p_st)
{
    int32_t const sign  = (OBST_PLAN_LEFT == g_o.side) ? 1 : -1;
    int32_t const theta = out_theta(p_odom, sign);
    int32_t       done_mm;
    int32_t       lat;

    if (!g_o.b_wait_motion)
    {
        out_start(now_ms, p_odom, theta, sign);
    }

    /* Casts (int32_t)(((int64_t)done_mm * ...)): |sin| <= 1.0 in Q15, so the
       result is no larger than the int32_t factor; (int64_t)done_mm: every
       int32_t value fits. */
    done_mm = p_odom->dist_mm - g_o.phase_start_mm;
    lat     = (int32_t)(((int64_t)done_mm * app_math_sin_q15(theta)) / 32768);

    /* Casts (int32_t)(((int64_t)done_mm * ...)): |cos| <= 1.0 in Q15, so the
       result is no larger than the int32_t factor; (int64_t)done_mm: every
       int32_t value fits. */
    if (obstacle_core_out_done(&g_o.out, lat)
        || obstacle_core_out_blocked(
            g_o.out_face_mm,
            (int32_t)(((int64_t)done_mm * app_math_cos_q15(theta)) / 32768),
            theta))
    {
        out_end(now_ms, p_st, done_mm, lat, theta);
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

/** Start OUT: the sensor looks along the line at the face. */
static void out_start (uint32_t now_ms, motion_types_odom_t const * p_odom,
                       int32_t theta, int32_t sign)
{
    int32_t back = 0;

    if ((OBST_STEP_MOVE == g_steps[0].kind) && (g_steps[0].value < 0))
    {
        back = -g_steps[0].value;
    }

    /* Casts (int32_t)OBST_US_AHEAD_MM: a constant that fits;
       (int32_t)(g_o.b_edge_face ? ...): the value is 0..65535. */
    g_o.b_wait_motion  = true;
    g_o.phase_start_mm = p_odom->dist_mm;
    g_o.out_h          = p_odom->heading_cdeg;
    g_o.out_face_mm =
        (int32_t)OBST_US_AHEAD_MM + back
        + (int32_t)(g_o.b_edge_face ? g_o.edge_face : g_prof.closest_mm);
    obstacle_core_out_start(&g_o.out, app_types_abs_i32(g_prof.deviation_mm));
    servo_to(obstacle_core_out_servo_deg(theta, sign), now_ms);
}

/** End OUT.  The later legs follow from the real step-out: PASS shorter by
   the extra forward travel, JOIN's limit from the real leg. */
static void out_end (uint32_t now_ms, obstacle_core_step_t const * p_st,
                     int32_t done_mm, int32_t lat, int32_t theta)
{
    /* Casts (int32_t)(((int64_t)(done_mm - ...))): |sin|, |cos| <= 1.0 in Q15,
       so the result is no larger than the int32_t factor;
       (int64_t)(done_mm - p_st->value): every int value fits. */
    int32_t const extra_fwd =
        (int32_t)(((int64_t)(done_mm - p_st->value) * app_math_cos_q15(theta))
                  / 32768);
    uint8_t kdx;

    /* Casts (int32_t)(((g_o.out.b_seen ? ...))): bit pattern of a packed LOG
       word, not an arithmetic value; (uint16_t)g_o.out.seen_lat: low 16 bits
       (two's complement) of the value, packed into a LOG word;
       (uint32_t)app_types_clamp_i32(lat ...): the value is 0..65535. */
    stop_motion();
    servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
    log_ev(OBST_LOG_OUT,
           (int32_t)(((g_o.out.b_seen ? (uint16_t)g_o.out.seen_lat : 0xFFFFu)
                      << 16)
                     | (uint32_t)app_types_clamp_i32(lat, 0, 0xFFFF)));

    /* Cast (uint8_t)(g_o.step_i + 1u): step_i < n_steps <=
       OBST_BYPASS_MAX_STEPS (10). */
    for (kdx = (uint8_t)(g_o.step_i + 1u); kdx < g_o.n_steps; kdx++)
    {
        if (OBST_STEP_PASS == g_steps[kdx].kind)
        {
            /* Casts (int16_t)app_types_max_i32(...): the planned PASS leg (<=
               OBST_PASS_MAX_MM) only gets shorter, never below 0;
               (int32_t)g_steps[kdx].value: every int16_t value fits. */
            g_steps[kdx].value = (int16_t)app_types_max_i32(
                (int32_t)g_steps[kdx].value - extra_fwd, 0);
        }
        else if (OBST_STEP_JOIN == g_steps[kdx].kind)
        {
            /* Cast (int16_t)app_types_min_i32(...): clamped to 0x7FFF; the leg
               is a positive distance. */
            g_steps[kdx].value = (int16_t)app_types_min_i32(
                ((done_mm * OBST_JOIN_MAX_PERMILLE) / 1000)
                    + OBST_JOIN_EXTRA_MM,
                0x7FFF);
        }
        else
        {
            /* unchanged */
        }
    }

    g_o.b_wait_motion = false;
    g_o.step_i++;
}

/** OBST_STEP_PASS: drive past the obstacle, the servo watching its side. */
static void bypass_pass (uint32_t now_ms, motion_types_odom_t const * p_odom,
                         obstacle_core_step_t const * p_st)
{
    int32_t done_mm;

    if (!g_o.b_wait_motion)
    {
        /* start PASS: look at the obstacle's side */
        g_o.b_wait_motion  = true;
        g_o.b_seen         = false;
        g_o.phase_start_mm = p_odom->dist_mm;
        g_o.last_seen_mm   = p_odom->dist_mm;
        servo_to((OBST_PLAN_LEFT == g_o.side)
                     ? (OBST_SERVO_CENTRE_DEG - OBST_SIDE_SERVO_DEG)
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
    if (((done_mm >= p_st->value)
         && (!g_o.b_seen
             || ((p_odom->dist_mm - g_o.last_seen_mm)
                 >= (OBST_ROBOT_LENGTH_MM + OBST_MARGIN_MM))))
        || (done_mm >= OBST_PASS_MAX_MM))
    {
        /* Casts (int32_t)(((g_o.b_seen ? 1u : 0u) ...)): bit pattern of a
           packed LOG word, not an arithmetic value;
           (uint32_t)app_types_clamp_i32(...): the value is 0..65535. */
        stop_motion();
        servo_to(OBST_SERVO_CENTRE_DEG, now_ms);
        log_ev(OBST_LOG_PASS,
               (int32_t)(((g_o.b_seen ? 1u : 0u) << 16)
                         | (uint32_t)app_types_clamp_i32(done_mm, 0, 0xFFFF)));
        g_o.b_wait_motion = false;
        g_o.step_i++;
    }
    else
    {
        drive(OBST_BYPASS_SPEED_MM_S, g_o.h0_cdeg, p_odom);

        if (servo_settled(now_ms))
        {
            ping();
        }
    }
}

/** OBST_STEP_JOIN: drive on until a line sensor sees the line, or give up
   after the step's distance. */
static void bypass_join (motion_types_odom_t const *  p_odom,
                         obstacle_core_step_t const * p_st)
{
    line_types_snap_t line;
    int32_t           done_mm;

    if (!g_o.b_wait_motion)
    {
        g_o.b_wait_motion  = true;
        g_o.phase_start_mm = p_odom->dist_mm;
        g_o.join_h         = p_odom->heading_cdeg;
    }

    done_mm = p_odom->dist_mm - g_o.phase_start_mm;
    subsystem3_get_snap(&line);

    if (((line.flags & LINE_FLAG_ON_LINE) != 0u) || (done_mm >= p_st->value))
    {
        stop_motion();
        g_o.b_joined = ((line.flags & LINE_FLAG_ON_LINE) != 0u);
        log_ev(OBST_LOG_JOIN, g_o.b_joined ? app_types_max_i32(done_mm, 1) : 0);
        g_o.b_wait_motion = false;
        g_o.step_i++;
    }
    else
    {
        drive(OBST_BYPASS_SPEED_MM_S, g_o.join_h, p_odom);
    }
}

/** OBST_STEP_LINESCAN (6 Nov): spin toward value's side noting the heading
   where each line sensor goes on to and off the line; once both have
   crossed it the line runs midway between those four headings: stop,
   settle, turn there.  If the sweep ends first, sweep back the other way to
   OBST_LINESCAN_SPARE past the start. */
static void bypass_linescan (uint32_t                     now_ms,
                             motion_types_odom_t const *  p_odom,
                             obstacle_core_step_t const * p_st)
{
    line_types_snap_t line;
    int32_t const rel = app_math_wrap_cdeg(p_odom->heading_cdeg - g_o.scan_h0);

    subsystem3_get_snap(&line);

    if (LS_PH_ALIGN == g_o.scan_ph)
    {
        linescan_align(now_ms, p_odom, rel);
    }
    else if (!g_o.b_wait_motion)
    {
        linescan_next_sweep(now_ms, p_odom, p_st, rel);
    }
    else
    {
        linescan_watch(&line, rel);

        if ((2u == g_o.ls_seen[0]) && (2u == g_o.ls_seen[1]))
        {
            g_o.scan_target =
                (g_o.ls_on[0] + g_o.ls_off[0] + g_o.ls_on[1] + g_o.ls_off[1])
                / 4;
            stop_motion();
            g_o.b_wait_motion = false;
            g_o.b_scan_turned = false;
            g_o.scan_ph       = LS_PH_ALIGN;
            g_o.scan_until_ms = now_ms + OBST_LINESCAN_SETTLE_MS;
        }
        else if (motion_result(p_odom) != 0)
        {
            /* this sweep is over: the next one starts in
               linescan_next_sweep() */
        }
        else
        {
            /* still turning */
        }
    }
}

/** LINESCAN, LS_PH_ALIGN: after the settle time, turn to the line found. */
static void linescan_align (uint32_t now_ms, motion_types_odom_t const * p_odom,
                            int32_t rel)
{
    /* Cast (int32_t)(now_ms - ...): signed difference of two wrapping time
       stamps; they are less than 2^31 apart. */
    if (g_o.b_wait_motion)
    {
        (void)motion_result(p_odom); /* the turn to the middle */
    }
    else if ((int32_t)(now_ms - g_o.scan_until_ms) >= 0)
    {
        /* encoder heading: slip already in it */
        int32_t const delta = g_o.scan_target - rel;

        if (g_o.b_scan_turned || (app_types_abs_i32(delta) < 200))
        {
            /* 7 Nov: < 0 = turned to a one-sensor crossing */
            log_ev(OBST_LOG_LINESCAN,
                   (g_o.b_scan_part ? -1 : 1)
                       * app_types_max_i32(
                           app_types_abs_i32(g_o.scan_target) / 10, 1));
            g_o.b_scan_part = false;
            g_o.b_joined    = true;
            g_o.scan_pass   = 0u;
            g_o.scan_ph     = LS_PH_SWEEP;
            g_o.step_i++;
        }
        else if (MOTION_ST_IDLE_BRAKED == p_odom->state)
        {
            /* Cast (int16_t)(delta / 10): a wrapped heading difference is
               within +-18000 0.01 deg, so /10 fits. */
            g_o.b_scan_turned = true;
            send_step(OBST_STEP_TURN, (int16_t)(delta / 10), p_odom);
        }
        else
        {
            /* still stopping */
        }
    }
    else
    {
        /* settling */
    }
}

/** LINESCAN, between sweeps: start the next sweep, or end the scan. */
static void linescan_next_sweep (uint32_t                     now_ms,
                                 motion_types_odom_t const *  p_odom,
                                 obstacle_core_step_t const * p_st, int32_t rel)
{
    if ((g_o.scan_pass < 2u) && (MOTION_ST_IDLE_BRAKED == p_odom->state))
    {
        int32_t const sign = (p_st->value >= 0) ? 1 : -1;
        int16_t       amount;
        uint8_t       idx;

        if (0u == g_o.scan_pass)
        {
            g_o.scan_h0     = p_odom->heading_cdeg;
            amount          = slip_ddeg(p_st->value);
            g_o.ls_part_q   = 0u;
            g_o.b_scan_part = false;
        }
        else
        {
            /* back to OBST_LINESCAN_SPARE past where the scan began */
            /* Cast (int16_t)((-sign * ...)): rel is a wrapped heading (+-18000
               0.01 deg), so the turn is within +-2250 0.1 deg. */
            amount =
                (int16_t)((-sign
                           * ((OBST_LINESCAN_SPARE_DDEG * 10) + (sign * rel)))
                          / 10);
        }

        for (idx = 0u; idx < OBST_LINE_SENSORS; idx++)
        {
            g_o.ls_seen[idx]   = 0u;
            g_o.b_ls_prev[idx] = false;
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

/** LINESCAN, during a sweep: note where each sensor goes on to and off the
   line. */
static void linescan_watch (line_types_snap_t const * p_line, int32_t rel)
{
    uint8_t idx;

    for (idx = 0u; idx < OBST_LINE_SENSORS; idx++)
    {
        bool const b_on =
            ((p_line->sens & ((0u == idx) ? LINE_SENS_L : LINE_SENS_R)) != 0u);

        if (b_on && !g_o.b_ls_prev[idx] && (0u == g_o.ls_seen[idx]))
        {
            g_o.ls_on[idx]   = rel;
            g_o.ls_seen[idx] = 1u;
            linescan_partial(1u, rel); /* went on: a fallback */
        }
        else if (!b_on && g_o.b_ls_prev[idx] && (1u == g_o.ls_seen[idx]))
        {
            g_o.ls_off[idx]  = rel;
            g_o.ls_seen[idx] = 2u;
            /* a full crossing by one sensor */
            linescan_partial(2u, (g_o.ls_on[idx] + rel) / 2);
        }
        else
        {
            /* no change */
        }

        g_o.b_ls_prev[idx] = b_on;
    }
}

/** Keep the best one-sensor fallback: quality 1 = went on the line,
   2 = crossed it; a better quality replaces a worse one. */
static void linescan_partial (uint8_t quality, int32_t heading)
{
    if (g_o.ls_part_q < quality)
    {
        g_o.ls_part_q = quality;
        g_o.ls_part_h = heading;
    }
}

/** OBST_STEP_MOVE / OBST_STEP_TURN: one motion command, then wait for it. */
static void bypass_move (motion_types_odom_t const *  p_odom,
                         obstacle_core_step_t const * p_st)
{
    if (!g_o.b_wait_motion)
    {
        if (MOTION_ST_IDLE_BRAKED == p_odom->state)
        {
            send_step(p_st->kind,
                      (OBST_STEP_TURN == p_st->kind) ? slip_ddeg(p_st->value)
                                                     : p_st->value,
                      p_odom);
        }
    }
    else
    {
        int32_t const res = motion_result(p_odom);

        if (res > 0)
        {
            g_o.step_i++;
        }
        else if (res < 0)
        {
            post(MISSION_EV_RECOVERY_FAILED, 1u, 0); /* bypass step failed */
            goto_state(OBST_ST_IDLE);
        }
        else
        {
            /* still moving */
        }
    }
}

/** SEEK_LINE, one slot: drive on until the line is found. */
static void seek_line_slot (motion_types_odom_t const * p_odom)
{
    line_types_snap_t line;

    subsystem3_get_snap(&line);

    if (((line.flags & LINE_FLAG_ON_LINE) != 0u)
        && (!g_o.b_have_pose
            || obstacle_core_line_accept(p_odom->x_mm, p_odom->y_mm,
                                         p_odom->heading_cdeg, g_o.x0_mm,
                                         g_o.y0_mm, g_o.h0_cdeg)))
    {
        stop_motion();
        goto_state(OBST_ST_ALIGN);
    }
    else if ((p_odom->dist_mm - g_o.phase_start_mm) > OBST_SEEK_MAX_MM)
    {
        stop_motion();
        g_o.sweep_i = 0u;
        goto_state(OBST_ST_SEEK_SWEEP); /* not straight ahead: look around */
    }
    else
    {
        drive(OBST_SEEK_SPEED_MM_S,
              g_o.b_have_pose ? g_o.h0_cdeg : p_odom->heading_cdeg, p_odom);
    }
}

/** SEEK_SWEEP, one slot: turn left, right and back looking for any line. */
static void seek_sweep_slot (motion_types_odom_t const * p_odom)
{
    static int16_t const k_sweep[3] = {OBST_SWEEP_DDEG, -(2 * OBST_SWEEP_DDEG),
                                       OBST_SWEEP_DDEG};
    line_types_snap_t    line;

    subsystem3_get_snap(&line);

    if ((line.flags & LINE_FLAG_ON_LINE) != 0u)
    {
        /* any line: line following takes over from here */
        stop_motion();
        g_o.b_have_pose = false;
        post(MISSION_EV_RECOVERY_DONE, 0u, 0);
        goto_state(OBST_ST_IDLE);
    }
    else if (g_o.b_wait_motion)
    {
        if (motion_result(p_odom) != 0)
        {
            g_o.sweep_i++; /* done or failed: next leg */
        }
    }
    else if (g_o.sweep_i >= 3u)
    {
        post(MISSION_EV_RECOVERY_FAILED, 0u, 0);
        goto_state(OBST_ST_IDLE);
        g_o.b_have_pose = false;
    }
    else if (MOTION_ST_IDLE_BRAKED == p_odom->state)
    {
        send_turn(slip_ddeg(k_sweep[g_o.sweep_i]), OBST_SWEEP_TURN_DEG_S,
                  p_odom);
    }
    else
    {
        /* still stopping */
    }
}

/** ALIGN, one slot: turn back to the heading held before the obstacle. */
static void align_slot (motion_types_odom_t const * p_odom)
{
    int32_t const dh_cdeg =
        app_math_wrap_cdeg(g_o.h0_cdeg - p_odom->heading_cdeg);

    if (!g_o.b_wait_motion && g_o.b_have_pose
        && (app_types_abs_i32(dh_cdeg) > OBST_ALIGN_TOL_CDEG)
        && (MOTION_ST_IDLE_BRAKED == p_odom->state))
    {
        /* Cast (int16_t)(dh_cdeg / 10): a wrapped heading difference is within
           +-18000 0.01 deg, so /10 fits. */
        send_step(OBST_STEP_TURN, (int16_t)(dh_cdeg / 10), p_odom);
    }
    else if (g_o.b_wait_motion)
    {
        if (motion_result(p_odom) != 0)
        {
            g_o.b_wait_motion = false;
            g_o.b_have_pose   = false;
            post(MISSION_EV_RECOVERY_DONE, 0u, 0);
            goto_state(OBST_ST_IDLE);
        }
    }
    else if (MOTION_ST_IDLE_BRAKED == p_odom->state) /* braked and aligned */
    {
        g_o.b_have_pose = false;
        post(MISSION_EV_RECOVERY_DONE, 0u, 0);
        goto_state(OBST_ST_IDLE);
    }
    else
    {
        /* still stopping */
    }
}

/*** end of file ***/
