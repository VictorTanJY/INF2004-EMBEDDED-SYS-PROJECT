/*
 *  test_s5_main.c
 *  Host unit tests for subsystem 5: the system statechart (mission_core.c)
 *  and the obstacle geometry (obstacle_core.c).
 *  Obstacles are simulated as flat-faced boxes seen by a 15 deg cone: a
 *  ping returns the nearest face point inside +-7.5 deg of the servo angle.
 *  That is a model of the HC-SR04, not the sensor: width accuracy on the
 *  real robot is measured on boxes of known width (A11, FR10).
 *  Build and run:  make -C testing/subsystem5/host
 */

#include "test_s5_main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_types.h"
#include "app_pack.h"
#include "mission_core.h"
#include "mission_cfg.h"
#include "obstacle_core.h"

#define T_SIDES          2   /* left, right */
#define T_SCAN_MAX_PINGS 60  /* a scan ends well before this */
#define T_BOX_MIN_MM     100 /* box widths 10..20 cm */
#define T_BOX_MAX_MM     200
#define T_BOX_OFFSET_MM  40 /* box centre offsets */
#define T_COL_LEFT_MM    40 /* obstacle columns -40..60 mm */
#define T_COL_RIGHT_MM   60
#define T_COL_SIDE_MM    60  /* obstacle columns -60, 0, 60 mm */
#define T_CASES3         3   /* values tried per parameter */
#define T_PROBE_PASSES   2   /* step out, step back */
#define T_COS_VALS       3   /* entries of cos_vals[] */
#define T_CORNER_PTS     5u  /* entries of pts[] */
#define T_WALL_PTS       13u /* entries of pts[] */

#define CHECK(id, cond)                                                      \
    do                                                                       \
    {                                                                        \
        if (cond)                                                            \
        {                                                                    \
            g_pass++;                                                        \
        }                                                                    \
        else                                                                 \
        {                                                                    \
            g_fail++;                                                        \
            printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); \
        }                                                                    \
    } while (0)

typedef struct
{
    float64_t diff, omega, coef;
} box_t; /* face distance, width, centre lateral (+ left) */

static int32_t g_pass;
static int32_t g_fail;

static mission_core_t        g_mc;
static mission_core_out_t    g_out;
static mission_core_inputs_t g_in;
static uint32_t              g_now;

/* ---- obstacle geometry --------------------------------------------- */
/* 4 Nov edge scan model: car heading phi (deg, + left) about the axle, the
   sensor OBST_US_AHEAD_MM ahead looking along phi; the beam hears the box
   up to g_edge_beam deg off its axis (7 Nov: the black box ~36, the blue
   box's left side ~20: it depends on the box, so the firmware assumes
   none); a box face d mm ahead of the sensor, w wide, centred c (+ left). */
static float64_t g_edge_beam = 36.0;

static uint16_t g_edge_clutter =
    0u; /* range of clutter seen beyond 25 deg right, 0 = none */

static int32_t  g_edge_phi[2]; /* last echo per side (left, right), 0.01 deg */
static uint16_t g_edge_r[2];

static int32_t g_pings;

static void     ev(uint8_t type, uint16_t val_a, int32_t data);
static void     tick(uint32_t delay_ms);
static int32_t  find(uint8_t kind, uint8_t cmd);
static uint8_t  last_seq(void);
static void     boot_to_idle(void);
static void     calibrate(void);
static void     test_mission(void);
static void     edge_sim(float64_t diff, float64_t omega, float64_t coef,
                         obstacle_core_profile_t * p_prof);
static int32_t  out_sim(float64_t diff, float64_t omega, float64_t coef,
                        int32_t sign, int32_t min_lat, float64_t beam_deg,
                        float64_t * p_edge);
static uint16_t edge_ping(float64_t diff, float64_t omega, float64_t coef,
                          float64_t phi_deg);
static uint16_t ping(box_t const * p_b, int32_t n_b, int32_t angle_deg);
static void     scan_box(box_t const * p_b, int32_t n_b,
                         obstacle_core_profile_t * p_prof);
static void     test_obstacle(void);
static void     test_bench(void);
static void     test_pack_s5(void);

int main (void)
{
    test_mission();
    test_bench();
    test_obstacle();
    test_pack_s5();
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (0 == g_fail) ? 0 : 1;
}

static void ev (uint8_t type, uint16_t val_a, int32_t data)
{
    mission_types_evt_t evt = {type, 0u, val_a, data};

    g_out.n_acts = 0u;
    mission_core_event(&g_mc, &evt, g_now, &g_out);
}

static void tick (uint32_t delay_ms)
{
    uint32_t elapsed;

    for (elapsed = 0u; elapsed < delay_ms; elapsed += 100u)
    {
        g_now += 100u;
        g_out.n_acts = 0u;
        mission_core_tick(&g_mc, &g_in, g_now, &g_out);
        if (g_out.n_acts > 0u)
        {
            return;
        } /* caller inspects the actions */
    }
}

/* Find an action; returns its index or -1. */
static int32_t find (uint8_t kind, uint8_t cmd)
{
    uint8_t idx;

    for (idx = 0u; idx < g_out.n_acts; idx++)
    {
        if ((g_out.act[idx].kind == kind) && (g_out.act[idx].cmd == cmd))
        {
            return (int32_t)idx;
        }
    }
    return -1;
}

static uint8_t last_seq (void)
{
    int32_t idx;

    for (idx = (int32_t)g_out.n_acts - 1; idx >= 0; idx--)
    {
        if ((MISSION_ACT_MOTION == g_out.act[idx].kind)
            && (g_out.act[idx].seq != 0u))
        {
            return g_out.act[idx].seq;
        }
    }
    return 0u;
}

static void boot_to_idle (void)
{
    g_now = 1000u;
    memset(&g_in, 0, sizeof(g_in));
    mission_core_init(&g_mc, g_now);
    g_in.b_motion_hw_ok  = true;
    g_in.b_line_hw_ok    = true;
    g_in.b_imu_ok        = true;
    g_in.b_timer_checked = true;
    g_in.b_timer_ok      = true;
    tick(100u);
}

static void calibrate (void)
{
    uint8_t seq;

    ev(MISSION_EV_BTN_CALIBRATE, 0u, 0);
    tick(MISSION_CAL_STILL_MS + 100u); /* STILL -> SPIN */
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    ev(MISSION_EV_CAL_DONE, 3u, 1);
    ev(MISSION_EV_CAL_DONE, 4u, 1);
    tick(100u);
}

static void test_mission (void)
{
    char    map[4];
    uint8_t seq;

    /* S5-H01: BOOT -> IDLE when every self-test passed; -> FAULT after 3 s
     * otherwise */
    boot_to_idle();
    CHECK("S5-H01", MISSION_ST_IDLE == g_mc.snap.state);
    g_now = 0u;
    mission_core_init(&g_mc, g_now);
    g_in.b_imu_ok = false;
    tick(2900u);
    CHECK("S5-H01", MISSION_ST_BOOT == g_mc.snap.state);
    tick(300u);
    CHECK("S5-H01", (MISSION_ST_FAULT == g_mc.snap.state)
                        && (MISSION_CAUSE_SELFTEST == g_mc.snap.cause));

    /* S5-H02: START refused until calibrated; calibration sequence */
    boot_to_idle();
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    CHECK("S5-H02",
          (MISSION_ST_IDLE == g_mc.snap.state)
              && (find(MISSION_ACT_LOG, MISSION_LOG_NOT_CALIBRATED) >= 0));
    ev(MISSION_EV_BTN_CALIBRATE, 0u, 0);
    CHECK("S5-H02", (MISSION_ST_CALIBRATING == g_mc.snap.state)
                        && (MISSION_CAL_PH_STILL == g_mc.snap.cal_phase));
    tick(MISSION_CAL_STILL_MS + 100u);
    CHECK("S5-H02", (MISSION_CAL_PH_SPIN == g_mc.snap.cal_phase)
                        && (find(MISSION_ACT_MOTION, MOTION_CMD_TURN) >= 0));
    CHECK("S5-H02",
          3600 == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].arg0);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, (uint16_t)(seq + 1u),
       MOTION_SRC_MISSION); /* stale seq: ignored */
    CHECK("S5-H02", MISSION_CAL_PH_SPIN == g_mc.snap.cal_phase);
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H02", MISSION_CAL_PH_FINISH == g_mc.snap.cal_phase);
    ev(MISSION_EV_CAL_DONE, 3u, 1);
    ev(MISSION_EV_CAL_DONE, 4u, 0); /* IMU failed */
    tick(100u);
    CHECK("S5-H02", (MISSION_ST_IDLE == g_mc.snap.state)
                        && (MISSION_CAL_IR == g_mc.snap.calibrated));
    calibrate();
    CHECK("S5-H02", MISSION_CAL_ALL == g_mc.snap.calibrated);

    /* S5-H03: MQTT map (not hard-coded): no map -> barcodes reported only */
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    CHECK("S5-H03", (MISSION_ST_LINE_FOLLOW == g_mc.snap.state)
                        && (MOTION_SRC_LINE == g_mc.snap.auth));
    CHECK("S5-H03", (find(MISSION_ACT_SCAN, SCAN_REQ_MONITOR) >= 0)
                        && (find(MISSION_ACT_NEW_RUN, 0u) >= 0));
    ev(MISSION_EV_BARCODE, 0u, 'A');
    CHECK("S5-H03",
          (MISSION_NAV_NONE == g_mc.snap.pending_nav)
              && (find(MISSION_ACT_LOG, MISSION_LOG_NAV_UNMAPPED) >= 0));
    memcpy(map, "AFZW", 4);
    ev(MISSION_EV_REMOTE_MAP, 0u,
       mission_core_map_pack(map)); /* refused while running */
    CHECK("S5-H03", '\0' == g_mc.snap.nav_map[0]);
    ev(MISSION_EV_REMOTE_STOP, 0u, 0);
    CHECK("S5-H03", (MISSION_ST_MISSION_COMPLETE == g_mc.snap.state)
                        && (find(MISSION_ACT_LOG, MISSION_LOG_SUMMARY) >= 0));
    memcpy(map, "AFAW", 4);
    ev(MISSION_EV_REMOTE_MAP, 0u,
       mission_core_map_pack(map)); /* duplicate: refused */
    CHECK("S5-H03", ('\0' == g_mc.snap.nav_map[0])
                        && (find(MISSION_ACT_LOG, MISSION_LOG_MAP_BAD) >= 0));
    memcpy(map, "AF*W", 4);
    CHECK("S5-H03", !mission_core_map_unpack(mission_core_map_pack(map), map));
    memcpy(map, "QFZW", 4); /* left Q, right F, straight Z, U-turn W */
    ev(MISSION_EV_REMOTE_MAP, 0u, mission_core_map_pack(map));
    CHECK("S5-H03", 0 == memcmp(g_mc.snap.nav_map, "QFZW", 4));

    /* S5-H04: barcode latched, executed at the NEXT junction (FR4) */
    ev(MISSION_EV_BTN_START_STOP, 0u, 0);
    CHECK("S5-H04", (MISSION_ST_LINE_FOLLOW == g_mc.snap.state)
                        && (2u == g_mc.snap.runs));
    ev(MISSION_EV_BARCODE, 0u, 'F');
    CHECK("S5-H04", (MISSION_NAV_RIGHT == g_mc.snap.pending_nav)
                        && ('F' == g_mc.snap.pending_char));
    ev(MISSION_EV_BARCODE, 0u, 'X'); /* unmapped: does not clear the latch */
    CHECK("S5-H04", MISSION_NAV_RIGHT == g_mc.snap.pending_nav);
    ev(MISSION_EV_JUNCTION, 0u, 0);
    CHECK("S5-H04", (MISSION_ST_EXECUTE_NAV == g_mc.snap.state)
                        && (MOTION_SRC_MISSION == g_mc.snap.auth));
    CHECK("S5-H04", (find(MISSION_ACT_MOTION, MOTION_CMD_MOVE) >= 0)
                        && (MISSION_NAV_NONE == g_mc.snap.pending_nav));
    seq = last_seq();
    ev(MISSION_EV_BARCODE, 0u,
       'Q'); /* read during the manoeuvre: latched for later */
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK(
        "S5-H04",
        (find(MISSION_ACT_MOTION, MOTION_CMD_TURN) >= 0)
            && (-900
                == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].arg0));
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04", (MISSION_ST_LINE_FOLLOW == g_mc.snap.state)
                        && (MOTION_SRC_LINE == g_mc.snap.auth));
    CHECK("S5-H04", MISSION_NAV_LEFT == g_mc.snap.pending_nav);
    ev(MISSION_EV_JUNCTION, 0u, 0);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04",
          900 == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].arg0);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    ev(MISSION_EV_BARCODE, 0u, 'W');
    ev(MISSION_EV_JUNCTION, 0u, 0);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04", 1800
                        == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)]
                               .arg0); /* U-turn */
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    ev(MISSION_EV_BARCODE, 0u, 'Z');
    ev(MISSION_EV_JUNCTION, 0u, 0);
    CHECK(
        "S5-H04",
        (find(MISSION_ACT_MOTION, MOTION_CMD_MOVE) >= 0)
            && (MISSION_NAV_STRAIGHT_MM
                == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_MOVE)].arg0));
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04", MISSION_ST_LINE_FOLLOW == g_mc.snap.state);
    ev(MISSION_EV_JUNCTION, 0u, 0); /* no latch: straight through */
    CHECK("S5-H04", MISSION_ST_LINE_FOLLOW == g_mc.snap.state);

    /* S5-H05: obstacle -> profile -> bypass -> recovery -> line follow */
    ev(MISSION_EV_SLOW_DOWN, 0u, 380);
    CHECK("S5-H05", g_mc.snap.speed_cap_mm_s
                        == /* the cap only ever lowers the speed */
                        ((MISSION_CRUISE_MM_S < MISSION_SLOW_SPEED_MM_S)
                             ? MISSION_CRUISE_MM_S
                             : MISSION_SLOW_SPEED_MM_S));
    ev(MISSION_EV_OBSTACLE, 0u, 190);
    CHECK("S5-H05", (MISSION_ST_OBSTACLE_HANDLING == g_mc.snap.state)
                        && (MOTION_SRC_AVOID == g_mc.snap.auth));
    CHECK("S5-H05", (find(MISSION_ACT_MOTION, MOTION_CMD_STOP) >= 0)
                        && (find(MISSION_ACT_SCAN, SCAN_REQ_PROFILE) >= 0));
    ev(MISSION_EV_PROFILE_DONE, OBST_PLAN_LEFT, 150);
    CHECK("S5-H05", (MISSION_ST_AVOIDING == g_mc.snap.state)
                        && (find(MISSION_ACT_SCAN, SCAN_REQ_BYPASS_LEFT) >= 0));
    ev(MISSION_EV_BYPASS_DONE, 0u, 0);
    CHECK("S5-H05", (MISSION_ST_LINE_RECOVERY == g_mc.snap.state)
                        && (find(MISSION_ACT_SCAN, SCAN_REQ_RECOVER) >= 0));
    ev(MISSION_EV_RECOVERY_DONE, 0u, 0);
    CHECK("S5-H05", (MISSION_ST_LINE_FOLLOW == g_mc.snap.state)
                        && (MISSION_CRUISE_MM_S == g_mc.snap.speed_cap_mm_s));
    ev(MISSION_EV_OBSTACLE, 0u, 190);
    ev(MISSION_EV_PROFILE_DONE, OBST_PLAN_CONTINUE, 0);
    CHECK("S5-H05", MISSION_ST_LINE_FOLLOW == g_mc.snap.state);
    ev(MISSION_EV_OBSTACLE, 0u, 190);
    ev(MISSION_EV_BLOCKED_TIMEOUT, 0u, 0);
    CHECK("S5-H05", (MISSION_ST_FAULT == g_mc.snap.state)
                        && (MISSION_CAUSE_BLOCKED == g_mc.snap.cause));

    /* S5-H06: FAULT needs an operator reset that SafetyTask accepts */
    ev(MISSION_EV_BTN_RESET, 0u, 0);
    CHECK("S5-H06", (MISSION_ST_FAULT == g_mc.snap.state)
                        && (find(MISSION_ACT_SAFETY_RESET, 0u) >= 0));
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    CHECK("S5-H06", MISSION_ST_FAULT == g_mc.snap.state);
    ev(MISSION_EV_SAFETY_OK, 0u, 0);
    CHECK("S5-H06",
          (MISSION_ST_BOOT == g_mc.snap.state)
              && (find(MISSION_ACT_MOTION, MOTION_CMD_CLEAR_ESTOP) >= 0));
    tick(100u);
    CHECK("S5-H06", (MISSION_ST_IDLE == g_mc.snap.state)
                        && (MISSION_CAL_ALL == g_mc.snap.calibrated));
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    ev(MISSION_EV_FAULT, 0u, 0x0002); /* TOO_CLOSE from SafetyTask */
    CHECK("S5-H06", (MISSION_ST_FAULT == g_mc.snap.state)
                        && (0x0002u == g_mc.snap.fault_bits));

    /* S5-H07: line lost -> bounded +-60 deg search -> FAULT, or found */
    ev(MISSION_EV_SAFETY_OK, 0u, 0);
    tick(100u);
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    ev(MISSION_EV_LINE_LOST, 0u, 0);
    CHECK(
        "S5-H07",
        (MISSION_ST_LINE_SEARCH == g_mc.snap.state)
            && (600
                == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].arg0));
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H07",
          -1200 == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].arg0);
    g_in.b_on_line = true;
    tick(200u);
    CHECK("S5-H07", MISSION_ST_LINE_FOLLOW == g_mc.snap.state);
    g_in.b_on_line = false;
    ev(MISSION_EV_LINE_LOST, 0u, 0);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    seq = last_seq();
    ev(MISSION_EV_MOTION_FAILED, seq, MOTION_SRC_MISSION);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H07", (MISSION_ST_FAULT == g_mc.snap.state)
                        && (MISSION_CAUSE_LINE_LOST == g_mc.snap.cause));

    /* S5-H08: an mbf_mission drop while running is a fault; set only in IDLE */
    ev(MISSION_EV_SAFETY_OK, 0u, 0);
    tick(100u);
    ev(MISSION_EV_REMOTE_SET, 1u, 300);
    CHECK("S5-H08", (300u == g_mc.snap.cruise_mm_s)
                        && (find(MISSION_ACT_LOG, MISSION_LOG_PARAM_SET) >= 0));
    ev(MISSION_EV_REMOTE_SET, 3u, 25);
    CHECK("S5-H08", 250u == g_mc.snap.stop_mm);
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    ev(MISSION_EV_REMOTE_SET, 1u, 150);
    CHECK("S5-H08", 300u == g_mc.snap.cruise_mm_s);
    ev(MISSION_EV_HUMP_START, 0u, 0);
    CHECK("S5-H08", MISSION_HUMP_SPEED_MM_S == g_mc.snap.speed_cap_mm_s);
    ev(MISSION_EV_HUMP_END, 0u, 21);
    CHECK("S5-H08", 300u == g_mc.snap.speed_cap_mm_s);
    g_in.mbf_drops = 1u;
    tick(100u);
    CHECK("S5-H08", (MISSION_ST_FAULT == g_mc.snap.state)
                        && (MISSION_CAUSE_MBF_OVERFLOW == g_mc.snap.cause));
}

static void edge_sim (float64_t diff, float64_t omega, float64_t coef,
                      obstacle_core_profile_t * p_prof)
{
    bool    b_has[2]   = {false, false};
    int32_t lat[2]     = {0, 0};
    int32_t minimum[2] = {0, 0};
    int32_t kdx;

    for (kdx = 0; kdx < T_SIDES; kdx++)
    {
        int32_t const side = (0 == kdx) ? 1 : -1;
        int32_t       phi  = 0;
        uint32_t      miss = 0u;

        int32_t  low    = 0;
        int32_t  high   = 0;
        bool     b_past = false;
        uint32_t iter;

        while ((miss < OBST_EDGE_MISSES)
               && (abs(phi) < (OBST_EDGE_MAX_DDEG * 10)))
        {
            uint16_t ret;

            phi += side * OBST_EDGE_STEP_DDEG * 10;
            ret = edge_ping(diff, omega, coef, phi / 100.0);
            if ((side < 0) && (phi <= -2500) && (g_edge_clutter != 0u))
            {
                ret = g_edge_clutter;
            } /* something to the right */
            if (obstacle_core_edge_on_face(phi, ret, (uint16_t)diff))
            {
                if (obstacle_core_edge_past_max(
                        obstacle_core_edge_wide_lat(phi, ret, side), side))
                {
                    b_past = true;
                    break;
                }
                b_has[kdx]      = true;
                lat[kdx]        = obstacle_core_edge_wide_lat(phi, ret, side);
                minimum[kdx]    = obstacle_core_edge_lat(phi);
                miss            = 0u;
                g_edge_phi[kdx] = phi;
                g_edge_r[kdx]   = ret;
                low             = phi;
            }
            else
            {
                if (0u == miss)
                {
                    high = phi;
                }
                miss++;
            }
        }
        /* 7 Nov: refine between the last echo and the first miss, as the task
         */
        for (iter = 0u; b_has[kdx] && !b_past && (miss >= OBST_EDGE_MISSES)
                        && (iter < OBST_EDGE_REFINE_ITERS);
             iter++)
        {
            uint16_t ret;

            phi = (low + high) / 2;
            ret = edge_ping(diff, omega, coef, phi / 100.0);
            if (obstacle_core_edge_on_face(phi, ret, (uint16_t)diff)
                && !obstacle_core_edge_past_max(
                    obstacle_core_edge_wide_lat(phi, ret, side), side))
            {
                lat[kdx]        = obstacle_core_edge_wide_lat(phi, ret, side);
                minimum[kdx]    = obstacle_core_edge_lat(phi);
                g_edge_phi[kdx] = phi;
                g_edge_r[kdx]   = ret;
                low             = phi;
            }
            else
            {
                high = phi;
            }
            g_edge_phi[kdx] =
                (low + high)
                / 2; /* the edge is between the last echo and the miss */
        }
    }
    obstacle_core_edge_plan(p_prof, b_has[0], lat[0], b_has[1], lat[1]);
    obstacle_core_edge_plan_min(p_prof, b_has[0], minimum[0], b_has[1],
                                minimum[1]);
}

/* 7 Nov step-out leg model: from where the edge scan stopped (box face d
   ahead of the sensor, w wide, centred c, + left), the car turns 45 deg
   to side s and drives out; the servo shaft sits OBST_US_AHEAD -
   OBST_SERVO_ARM ahead of the axle and the sensor looks along the line,
   hearing the nearest face point within +-beam_deg; beam_deg 0: only the
   face straight ahead of it answers (a flat face reflects straight back,
   as the 7 Nov edge scans suggest).  A ping every 60 ms
   at the bypass speed.  Returns the axle's lateral (toward s) when the
   leg ends; *p_edge = the box edge on that side (toward s). */
static int32_t out_sim (float64_t diff, float64_t omega, float64_t coef,
                        int32_t sign, int32_t min_lat, float64_t beam_deg,
                        float64_t * p_edge)
{
    float64_t const     theta = (45.0 * M_PI) / 180.0;
    float64_t const     shaft = OBST_US_AHEAD_MM - OBST_SERVO_ARM_MM;
    float64_t const     face  = OBST_US_AHEAD_MM + diff; /* ahead of the axle */
    float64_t const     low   = (sign * coef) - (omega / 2.0);
    float64_t const     high = (sign * coef) + (omega / 2.0); /* box toward s */
    float64_t const     step = (OBST_BYPASS_SPEED_MM_S * 60.0) / 1000.0;
    obstacle_core_out_t out_state;
    float64_t           time_s;

    *p_edge = high;
    obstacle_core_out_start(&out_state, min_lat);
    for (time_s = 0.0; time_s < 2000.0; time_s += step)
    {
        float64_t const accel_x = time_s * sin(theta);
        float64_t const accel_y = time_s * cos(theta);
        float64_t const sum_x   = accel_x + (shaft * sin(theta));
        float64_t const sum_y =
            accel_y + (shaft * cos(theta)) + OBST_SERVO_ARM_MM;
        float64_t best = 1e9;
        float64_t x_val;
        uint16_t  ret;

        if (obstacle_core_out_done(&out_state, (int32_t)accel_x))
        {
            return (int32_t)accel_x;
        }
        for (x_val = low; x_val <= high; x_val += 1.0)
        {
            float64_t const delta_x = x_val - sum_x;
            float64_t const delta_y = face - sum_y;
            float64_t const radius2 =
                sqrt((delta_x * delta_x) + (delta_y * delta_y));

            if ((delta_y > 0.0)
                && (fabs(atan2(delta_x, delta_y))
                    <= ((fmax(beam_deg, 0.2) * M_PI) / 180.0))
                && (radius2 < best))
            {
                best = radius2;
            }
        }
        ret = (best > 4000.0) ? OBST_RANGE_NONE : (uint16_t)best;
        obstacle_core_out_ping(&out_state, (int32_t)accel_x, (int32_t)accel_y,
                               4500, ret, (int32_t)face);
    }
    return -1;
}

static uint16_t edge_ping (float64_t diff, float64_t omega, float64_t coef,
                           float64_t phi_deg)
{
    float64_t const phase = (phi_deg * M_PI) / 180.0;
    float64_t const sum_x = OBST_US_AHEAD_MM * sin(phase);
    float64_t const sum_y = OBST_US_AHEAD_MM * cos(phase);
    float64_t       best  = 1e9;
    float64_t       x_val;

    for (x_val = coef - (omega / 2.0); x_val <= (coef + (omega / 2.0));
         x_val += 1.0)
    {
        float64_t const delta_x = x_val - sum_x;
        float64_t const delta_y = (OBST_US_AHEAD_MM + diff) - sum_y;
        float64_t const ang     = atan2(delta_x, delta_y) - phase;
        float64_t const radius =
            sqrt((delta_x * delta_x) + (delta_y * delta_y));

        if ((fabs(ang) <= ((g_edge_beam * M_PI) / 180.0)) && (radius < best))
        {
            best = radius;
        }
    }
    return (best > 4000.0) ? OBST_RANGE_NONE : (uint16_t)best;
}

static uint16_t ping (box_t const * p_b, int32_t n_b, int32_t angle_deg)
{
    float64_t best = 1e9;
    int32_t   kdx;

    for (kdx = 0; kdx < n_b; kdx++)
    {
        float64_t lat;

        for (lat = p_b[kdx].coef - (p_b[kdx].omega / 2.0);
             lat <= (p_b[kdx].coef + (p_b[kdx].omega / 2.0) + 1e-9); lat += 1.0)
        {
            /* 7 Nov: the sensor faces along the servo horn, OBST_SERVO_ARM_MM
               out from the shaft; box d is from the face with the servo at
               90 deg, so the shaft sits ARM behind that point */
            float64_t const accel_r = (angle_deg * M_PI) / 180.0;
            float64_t const sum_x   = -OBST_SERVO_ARM_MM * cos(accel_r);
            float64_t const sum_y   = OBST_SERVO_ARM_MM * (sin(accel_r) - 1.0);
            float64_t       val_a =
                (atan2(p_b[kdx].diff - sum_y, -(lat - sum_x)) * 180.0) / M_PI;
            float64_t radius =
                sqrt(((p_b[kdx].diff - sum_y) * (p_b[kdx].diff - sum_y))
                     + ((lat - sum_x) * (lat - sum_x)));

            if ((fabs(val_a - angle_deg) <= 7.5) && (radius < best))
            {
                best = radius;
            }
        }
    }
    return (best > 4000.0) ? OBST_RANGE_NONE : (uint16_t)lround(best);
}

static void scan_box (box_t const * p_b, int32_t n_b,
                      obstacle_core_profile_t * p_prof)
{
    obstacle_core_scan_t scan;

    obstacle_core_scan_start(&scan);
    g_pings = 0;
    while (!obstacle_core_scan_done(&scan) && (g_pings < T_SCAN_MAX_PINGS))
    {
        obstacle_core_scan_feed(
            &scan, ping(p_b, n_b, obstacle_core_scan_angle(&scan)));
        g_pings++;
    }
    obstacle_core_scan_profile(&scan, p_prof);
}

static void test_obstacle (void)
{
    obstacle_core_profile_t prof;
    obstacle_core_step_t    steps[OBST_BYPASS_MAX_STEPS];
    float64_t               worst = 0.0;
    int32_t                 width;
    int32_t                 cnt;

    /* S5-H09: helpers */
    CHECK("S5-H09",
          (200u == obstacle_core_median3(300u, 120u, 200u))
              && (100u == obstacle_core_median3(OBST_RANGE_NONE, 100u, 90u)));
    CHECK("S5-H09",
          (200u == obstacle_core_echo_to_mm(1160u))
              && (OBST_RANGE_NONE == obstacle_core_echo_to_mm(23200u)));
    CHECK("S5-H09", OBST_RANGE_NONE == obstacle_core_echo_to_mm(0u));

    /* S5-H10: width of 10-20 cm boxes (instructor, 23 Oct) at 20 cm, centred
       and offset; model error target: within 10 % (grade A) */
    for (width = T_BOX_MIN_MM; width <= T_BOX_MAX_MM; width += 50)
    {
        for (cnt = -T_BOX_OFFSET_MM; cnt <= T_BOX_OFFSET_MM;
             cnt += T_BOX_OFFSET_MM)
        {
            box_t     box = {200.0, (float64_t)width, (float64_t)cnt};
            float64_t err;

            scan_box(&box, 1, &prof);
            err = fabs((float64_t)prof.w_mm - width) / width;
            if (err > worst)
            {
                worst = err;
            }
            printf("box %3d mm at %+3d mm: width %3u mm  edges %+d/%+d  "
                   "closest %u  plan %u  pings %d\n",
                   width, cnt, (uint32_t)prof.w_mm, prof.left_lat_mm,
                   prof.right_lat_mm, (uint32_t)prof.closest_mm,
                   (uint32_t)prof.plan, g_pings);
            CHECK("S5-H10", g_pings <= 30); /* ~1.8 s of 60 ms pings */
            CHECK("S5-H10", prof.b_valid && (prof.closest_mm >= 195u)
                                && (prof.closest_mm <= 215u));
            CHECK("S5-H10", (OBST_PLAN_LEFT == prof.plan)
                                || (OBST_PLAN_RIGHT
                                    == prof.plan)); /* free space around */
        }
    }
    printf("worst width error on the beam model: %.1f %%\n", worst * 100.0);
    CHECK("S5-H10", worst <= 0.10);

    /* S5-H11: plan: free both sides -> the side needing less deviation */
    {
        box_t box = {200.0, 150.0, 30.0}; /* centred a little to the left */
        scan_box(&box, 1, &prof);
        CHECK("S5-H11",
              (OBST_PLAN_RIGHT == prof.plan) && (prof.deviation_mm < 0));
    }
    {
        box_t box[2] = {{200.0, 150.0, 0.0},
                        {260.0, 120.0, -200.0}}; /* wall on the right */
        scan_box(box, 2, &prof);
        CHECK("S5-H11",
              (OBST_PLAN_LEFT == prof.plan)
                  && (prof.clr_r_mm < (uint32_t)(OBST_ROBOT_WIDTH_MM
                                                 + (2 * OBST_MARGIN_MM))));
    }
    {
        /* 9 Nov 02:59: a wall right of the track, NEARER than the box on
           the line; the box was read as the wall (edges 7 to 40 cm right)
           and the approach drove into the box.  The box in the path is the
           obstacle; never CONTINUE, edges around the box. */
        int32_t col;

        for (col = -T_COL_LEFT_MM; col <= T_COL_RIGHT_MM; col += 20)
        {
            box_t box[2] = {{240.0, 150.0, (float64_t)col},
                            {170.0, 300.0, -330.0}}; /* wall 18-48 cm right */

            scan_box(box, 2, &prof);
            printf("box at %+d mm, wall right: edges %+d / %+d, closest %u, "
                   "plan %u\n",
                   col, prof.left_lat_mm, prof.right_lat_mm,
                   (uint32_t)prof.closest_mm, (uint32_t)prof.plan);
            CHECK("S5-H24", prof.b_valid && (prof.plan != OBST_PLAN_CONTINUE)
                                && (prof.closest_mm >= 225u)
                                && (prof.closest_mm <= 260u)
                                && (prof.right_lat_mm > ((col - 75) - 60))
                                && (prof.left_lat_mm < ((col + 75) + 60)));
        }
    }
    {
        /* 9 Nov single sweep: the plan straight from the servo scan.  A
           15 cm box 6 cm either side of the line: the car goes the short
           way, and the planned step-out clears the box's near edge
           by half the car + the margin (the step-out leg still pings live) */
        int32_t col;

        for (col = -T_COL_SIDE_MM; col <= T_COL_SIDE_MM; col += T_COL_SIDE_MM)
        {
            box_t         box   = {200.0, 150.0, (float64_t)col};
            int32_t const extra = OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2);

            uint16_t w_seen;

            scan_box(&box, 1, &prof);
            w_seen = prof.w_mm;
            obstacle_core_edge_plan(&prof, false, 0, false, 0);
            prof.w_mm = w_seen;
            CHECK("S5-H25", abs((int32_t)w_seen - 150) <= 15);
            printf("single sweep: box at %+d mm -> plan %u, step-out %d mm, "
                   "width %u\n",
                   col, (uint32_t)prof.plan, prof.deviation_mm,
                   (uint32_t)prof.w_mm);
            if (col > 0)
            {
                CHECK("S5-H25",
                      (OBST_PLAN_RIGHT == prof.plan)
                          && (-prof.deviation_mm >= ((75 - col) + extra)));
            }
            else if (col < 0)
            {
                CHECK("S5-H25",
                      (OBST_PLAN_LEFT == prof.plan)
                          && (prof.deviation_mm >= ((75 + col) + extra)));
            }
            else
            {
                CHECK("S5-H25",
                      ((OBST_PLAN_LEFT == prof.plan)
                       || (OBST_PLAN_RIGHT == prof.plan))
                          && (abs(prof.deviation_mm) >= (75 + extra)));
            }
        }
    }
    {
        /* 9 Nov: the approach ping before the width probe */
        CHECK("S5-H24",
              300u
                  == obstacle_core_probe_face(470u, 300u)); /* passed the box */
        CHECK("S5-H24",
              300u == obstacle_core_probe_face(OBST_RANGE_NONE, 300u));
        CHECK("S5-H24",
              330u == obstacle_core_probe_face(330u, 300u)); /* the face */
    }
    {
        box_t box[3] = {
            {200.0, 150.0, 0.0}, {250.0, 120.0, -200.0}, {250.0, 120.0, 200.0}};
        scan_box(box, 3, &prof);
        CHECK("S5-H11", OBST_PLAN_BLOCKED == prof.plan);
    }
    {
        box_t box = {200.0, 100.0, 340.0}; /* clearly beside the path */
        scan_box(&box, 1, &prof);
        CHECK("S5-H11", OBST_PLAN_CONTINUE == prof.plan);
    }
    {
        /* 3 Nov: a box 260 mm off the line is only partly seen (unsure),
           but even a 200 mm object centred on what was seen stays out of
           the car's corridor: continue */
        box_t box = {200.0, 100.0, 260.0};
        scan_box(&box, 1, &prof);
        CHECK("S5-H11", !prof.b_sure && (OBST_PLAN_CONTINUE == prof.plan)
                            && (prof.plan_r_mm > ((OBST_ROBOT_WIDTH_MM / 2)
                                                  + OBST_MARGIN_MM)));
    }
    {
        obstacle_core_point_t none[3] = {{80, OBST_RANGE_NONE},
                                         {90, OBST_RANGE_NONE},
                                         {100, OBST_RANGE_NONE}};
        obstacle_core_profile(none, 3u, none, 0u, &prof);
        CHECK("S5-H11", !prof.b_valid && (OBST_PLAN_CONTINUE == prof.plan));
    }
    {
        /* 3 Nov: a round bottle seen at 16 cm while driving, no echo at
           all in the scan: plan around a 20 cm object there, never CONTINUE */
        obstacle_core_point_t none[3] = {{80, OBST_RANGE_NONE},
                                         {90, OBST_RANGE_NONE},
                                         {100, OBST_RANGE_NONE}};
        int32_t const need = (OBST_WIDTH_MAX_MM / 2) + (OBST_ROBOT_WIDTH_MM / 2)
                             + OBST_MARGIN_MM;

        obstacle_core_profile_detected(none, 3u, none, 0u, 160u, 0u, &prof);
        CHECK("S5-H16", prof.b_valid && (160u == prof.closest_mm)
                            && ((OBST_PLAN_LEFT == prof.plan)
                                || (OBST_PLAN_RIGHT == prof.plan))
                            && (abs(prof.deviation_mm) >= need));
        /* 4 Nov: seen at 192 mm, backed off 60 mm, still no echo: still a
         * bypass */
        obstacle_core_profile_detected(none, 3u, none, 0u, 192u, 60u, &prof);
        CHECK("S5-H16", prof.b_valid && (252u == prof.closest_mm)
                            && (prof.plan != OBST_PLAN_CONTINUE));
        /* the scan saw it off to the side and clear: the detection does not
         * override */
        {
            box_t box = {200.0, 100.0, 260.0};
            scan_box(&box, 1, &prof);
            CHECK("S5-H16", OBST_PLAN_CONTINUE == prof.plan);
        }
    }

    /* S5-H21 (7 Nov): edge scan + step-out leg, end to end, on the two
       7 Nov runs and offset boxes.  The car must end clear of the box's
       edge by half its width + the margin (FR11), and not more than 10 cm
       further out than that (the step-out is measured, not padded).
       Black: 9.5 cm box, face 32.6 cm, heard 36 deg off the beam (read
       37.9 cm wide with a 7.5 deg correction).  Blue: ~15 cm box across
       the line, more to the left, its left side heard only 20 deg off
       (with a 30 deg correction the car stepped out 10.6 cm and hit it). */
    {
        static struct
        {
            float64_t    diff, omega, coef, beam, out_beam;
            char const * p_name;
        } const runs[] = {
            {326.0, 95.0, 0.0, 36.0, 0.0, "black 9.5 cm"},
            {326.0, 95.0, 0.0, 36.0, 15.0, "black, 15 deg beam out"},
            {326.0, 95.0, 0.0, 36.0, 36.0, "black, 36 deg beam out"},
            {326.0, 150.0, 15.0, 20.0, 0.0, "blue 15 cm, 1.5 cm left"},
            {326.0, 150.0, 15.0, 20.0, 15.0, "blue, 15 deg beam out"},
            {326.0, 150.0, 15.0, 20.0, 36.0, "blue, 36 deg beam out"},
            {200.0, 150.0, -60.0, 36.0, 0.0, "15 cm, 6 cm right"},
            {200.0, 100.0, 60.0, 20.0, 0.0, "10 cm, 6 cm left"},
            {200.0, 100.0, 60.0, 20.0, 15.0, "10 cm, 6 cm left, 15 deg"},
            {250.0, 200.0, 0.0, 36.0, 0.0, "20 cm centred"},
            {250.0, 200.0, 0.0, 36.0, 15.0, "20 cm centred, 15 deg"},
        };
        uint32_t idx;

        for (idx = 0u; idx < (sizeof(runs) / sizeof(runs[0])); idx++)
        {
            int32_t   sign;
            int32_t   lat;
            float64_t edge;

            memset(&prof, 0, sizeof(prof));
            prof.clr_l_mm = OBST_CLEAR_MAX_MM;
            prof.clr_r_mm = OBST_CLEAR_MAX_MM;
            g_edge_beam   = runs[idx].beam;
            edge_sim(runs[idx].diff, runs[idx].omega, runs[idx].coef, &prof);
            g_edge_beam = 36.0;
            sign        = (OBST_PLAN_LEFT == prof.plan) ? 1 : -1;
            lat = out_sim(runs[idx].diff, runs[idx].omega, runs[idx].coef, sign,
                          abs(prof.deviation_mm), runs[idx].out_beam, &edge);
            printf("step-out %-26s: edges %+4d/%+4d, %s, plan %3d mm, stepped "
                   "%3d mm, need %3.0f\n",
                   runs[idx].p_name, prof.left_lat_mm, prof.right_lat_mm,
                   (sign > 0) ? "left " : "right", abs(prof.deviation_mm), lat,
                   edge + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2));
            CHECK("S5-H21", (OBST_PLAN_LEFT == prof.plan)
                                || (OBST_PLAN_RIGHT == prof.plan));
            CHECK("S5-H21", lat > 0);
            CHECK("S5-H21", lat >= (int32_t)(edge + OBST_MARGIN_MM
                                             + (OBST_ROBOT_WIDTH_MM / 2)));
            /* not padded: a flat face answering straight back ends the leg
               within the allowance + a ping or two of the need, or of the
               plan's minimum if that was already further out */
            CHECK("S5-H21",
                  (runs[idx].out_beam > 0.0)
                      || (lat <= (int32_t)fmax(
                              edge + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2)
                                  + 40.0,
                              (float64_t)abs(prof.deviation_mm) + 10.0)));
        }
    }

    /* S5-H22 (7 Nov): reported width from the last echoes with the
       calibrated beam angle (OBST_WIDTH_BEAM_CDEG).  Boxes that echo like
       the calibration box read within 10 % (grade A) at 15-33 cm; a box
       heard much further off the beam reads wide (printed, the known
       limit: it only affects the dashboard number). */
    {
        static float64_t const widths[3] = {100.0, 150.0, 200.0};
        static float64_t const dists[3]  = {150.0, 250.0, 326.0};
        float64_t              worst_w   = 0.0;
        int32_t                val_a;
        int32_t                val_b2;

        for (val_a = 0; val_a < T_CASES3; val_a++)
        {
            for (val_b2 = 0; val_b2 < T_CASES3; val_b2++)
            {
                uint16_t  width;
                float64_t elem;

                memset(&prof, 0, sizeof(prof));
                prof.clr_l_mm = OBST_CLEAR_MAX_MM;
                prof.clr_r_mm = OBST_CLEAR_MAX_MM;
                g_edge_beam   = OBST_WIDTH_BEAM_CDEG / 100.0;
                edge_sim(dists[val_b2], widths[val_a], 0.0, &prof);
                g_edge_beam = 36.0;
                width = obstacle_core_edge_width(g_edge_phi[0], g_edge_r[0],
                                                 g_edge_phi[1], g_edge_r[1]);
                elem  = fabs(width - widths[val_a]) / widths[val_a];
                if (elem > worst_w)
                {
                    worst_w = elem;
                }
                printf("width: %3.0f mm box at %3.0f mm -> %3u mm (%4.1f %%)\n",
                       widths[val_a], dists[val_b2], (uint32_t)width,
                       elem * 100.0);
            }
        }
        CHECK("S5-H22", worst_w <= 0.10);
        memset(&prof, 0, sizeof(prof));
        g_edge_beam = 36.0;
        edge_sim(326.0, 95.0, 0.0, &prof);
        printf("width: 95 mm box heard to 36 deg (black box) -> %u mm with the "
               "%d deg calibration\n",
               (uint32_t)obstacle_core_edge_width(g_edge_phi[0], g_edge_r[0],
                                                  g_edge_phi[1], g_edge_r[1]),
               OBST_WIDTH_BEAM_CDEG / 100);
    }

    /* S5-H23 (7 Nov): width probe, as the task runs it: the car turned 65
       deg, the servo looking back along the line, drives left past the
       left edge, then back past the right edge, a ping every 60 ms at 60
       mm/s.  The face echoes while the sensor is within the face plus a
       run-on past each edge (15-25 mm; 20 measured on the car, corrected
       by OBST_PROBE_EDGE_OVER_MM).  10-20 cm boxes, centred and 6 cm off:
       with the run-on as corrected, width within 5 %%; 5 mm more or less
       run-on than corrected costs 10 mm of width (15 %% on a 10 cm box,
       so OBST_PROBE_EDGE_OVER_MM must be measured); edges within 15 mm. */
    {
        static float64_t const widths[3]   = {100.0, 150.0, 200.0};
        static float64_t const cos_vals[3] = {0.0, 60.0, -60.0};
        static float64_t const runs[3]     = {15.0, 20.0, 25.0};
        float64_t const        th_deg      = OBST_PROBE_DDEG / 10.0;
        int32_t const          theta       = OBST_PROBE_DDEG * 10;
        float64_t              worst       = 0.0;
        float64_t              worst_off   = 0.0;
        int32_t                val_a2;
        int32_t                val_b3;
        int32_t                cnt4;

        for (val_a2 = 0; val_a2 < T_CASES3; val_a2++)
        {
            for (val_b3 = 0; val_b3 < T_CASES3; val_b3++)
            {
                for (cnt4 = 0; cnt4 < T_CASES3; cnt4++)
                {
                    float64_t const omega = widths[val_a2];
                    float64_t const coef  = cos_vals[val_b3];
                    float64_t const run   = runs[cnt4];
                    int32_t const face = OBST_US_AHEAD_MM + OBST_PROBE_FACE_MM;
                    float64_t const step =
                        (OBST_PROBE_SPEED_MM_S * 60.0) / 1000.0;
                    obstacle_core_probe_t probe;
                    float64_t             diff = 0.0;
                    int32_t               pass;
                    int32_t               left;
                    int32_t               ret;
                    float64_t             elem;

                    obstacle_core_probe_start(&probe);
                    for (pass = 0; pass < T_PROBE_PASSES; pass++)
                    {
                        bool b_seen = false;

                        probe.off = 0u;
                        for (;;)
                        {
                            int32_t const lat =
                                (int32_t)(diff * sin((th_deg * M_PI) / 180.0));
                            int32_t const fwd =
                                (int32_t)(diff * cos((th_deg * M_PI) / 180.0));
                            int32_t const slope =
                                obstacle_core_probe_sensor_lat(lat, theta);
                            int32_t const expected =
                                obstacle_core_out_expect_mm(face, fwd, theta);
                            bool const b_on =
                                (slope <= (coef + (omega / 2.0) + run))
                                && (slope >= ((coef - (omega / 2.0)) - run));

                            if (obstacle_core_probe_ping(
                                    &probe, slope,
                                    b_on ? (uint16_t)expected : OBST_RANGE_NONE,
                                    expected))
                            {
                                b_seen = true;
                            }
                            if (0 == pass)
                            {
                                if (((probe.off >= OBST_PROBE_OFF)
                                     && (lat >= 20))
                                    || (lat >= OBST_PROBE_MAX_MM))
                                {
                                    break;
                                }
                                diff += step;
                            }
                            else
                            {
                                if ((b_seen && (probe.off >= OBST_PROBE_OFF))
                                    || (lat <= -OBST_PROBE_MAX_MM))
                                {
                                    break;
                                }
                                diff -= step;
                            }
                        }
                    }
                    CHECK("S5-H23",
                          obstacle_core_probe_edges(&probe, &left, &ret));
                    elem = fabs((left - ret) - omega) / omega;
                    if ((1 == cnt4) && (elem > worst))
                    {
                        worst = elem;
                    }
                    if ((cnt4 != 1) && (elem > worst_off))
                    {
                        worst_off = elem;
                    }
                    if ((1 == cnt4) || (elem > 0.10))
                    {
                        printf("probe: %3.0f mm box at %+3.0f, run-on %2.0f -> "
                               "edges %+4d / %+4d, width %3d (%.1f %%)\n",
                               omega, coef, run, (int32_t)left, (int32_t)ret,
                               (int32_t)(left - ret), elem * 100.0);
                    }
                    CHECK("S5-H23",
                          (fabs(left - (coef + (omega / 2.0))) <= 15.0)
                              && (fabs(ret - (coef - (omega / 2.0))) <= 15.0));
                }
            }
        }
        printf("probe: worst width error %.1f %% (run-on as corrected), %.1f "
               "%% (5 mm off)\n",
               worst * 100.0, worst_off * 100.0);
        CHECK("S5-H23", (worst <= 0.05) && (worst_off <= 0.15));
    }

    /* S5-H21: the servo looks back along the line (> 90 = left), and the
       leg ends before the front corner reaches the face */
    CHECK("S5-H21", (45 == obstacle_core_out_servo_deg(4500, 1))
                        && (135 == obstacle_core_out_servo_deg(4500, -1)));
    CHECK("S5-H21", !obstacle_core_out_blocked(411, 100, 4500)
                        && obstacle_core_out_blocked(411, 250, 4500));

    /* S5-H17 (4 Nov): edge scan with the car turning.  The wide edges
       (side choice) never inside the truth; side away from the centre.
       7 Nov: the offset is now only the step-out leg's minimum; clearing
       the box end to end is S5-H21. */
    {
        static float64_t const cos_vals[3] = {0.0, -62.0, 62.0};
        int32_t                elapsed;

        for (elapsed = 0; elapsed < T_COS_VALS; elapsed++)
        {
            memset(&prof, 0, sizeof(prof));
            prof.clr_l_mm = OBST_CLEAR_MAX_MM;
            prof.clr_r_mm = OBST_CLEAR_MAX_MM;
            edge_sim(195.0, 154.0, cos_vals[elapsed], &prof);
            printf("edge scan: box 154 at %+.0f -> edges %+d / %+d, plan %u, "
                   "offset %d\n",
                   cos_vals[elapsed], prof.left_lat_mm, prof.right_lat_mm,
                   (uint32_t)prof.plan, prof.deviation_mm);
            CHECK("S5-H17", prof.b_sure
                                && (prof.left_lat_mm
                                    >= ((cos_vals[elapsed] + 77.0) - 5.0)));
            CHECK("S5-H17",
                  prof.right_lat_mm <= ((cos_vals[elapsed] - 77.0) + 5.0));
            CHECK("S5-H17", (cos_vals[elapsed] > 0.0)
                                ? (OBST_PLAN_RIGHT == prof.plan)
                                : (OBST_PLAN_LEFT == prof.plan));
            CHECK("S5-H17", abs(prof.deviation_mm)
                                >= (OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2)
                                    + OBST_EDGE_ALLOW_MM));
        }
    }

    {
        /* 4 Nov: centred box, no echo at all on the left; the servo scan had
           seen it -73..+87 (+ left): use its left edge, so the side is
           decided by a real reading, not by "unseen = 20 cm more" */
        memset(&prof, 0, sizeof(prof));
        prof.b_valid   = true;
        prof.w_mm      = 160u;
        prof.plan_l_mm = 73;
        prof.plan_r_mm = -87;
        obstacle_core_edge_plan(&prof, false, 0, true, -88);
        /* 5 Nov: the step-outs are within 2 cm (203 / 218): the side whose
           edge was measured (right) is taken, its edge being known */
        CHECK("S5-H17",
              (73 == prof.left_lat_mm) && (OBST_PLAN_RIGHT == prof.plan));
        /* 5 Nov tie rule: centred within 2 cm -> measured side, then left */
        memset(&prof, 0, sizeof(prof));
        obstacle_core_edge_plan(&prof, true, 80, true,
                                -90); /* 1 cm right of centre: tie */
        CHECK("S5-H18", OBST_PLAN_LEFT == prof.plan);
        memset(&prof, 0, sizeof(prof));
        obstacle_core_edge_plan(&prof, true, 90, true,
                                -80); /* 1 cm left of centre: tie, still left */
        CHECK("S5-H18", OBST_PLAN_LEFT == prof.plan);
        memset(&prof, 0, sizeof(prof));
        obstacle_core_edge_plan(&prof, true, 120, true,
                                -60); /* 3 cm left: right is 6 cm shorter */
        CHECK("S5-H18", OBST_PLAN_RIGHT == prof.plan);
        /* 6 Nov: no echo either side; servo scan put the box 25..134 mm
           left: the right is the short way round */
        memset(&prof, 0, sizeof(prof));
        prof.b_valid   = true;
        prof.w_mm      = 109u;
        prof.plan_l_mm = 134;
        prof.plan_r_mm = 25;
        prof.plan = OBST_PLAN_LEFT; /* what the servo scan's own rules said */
        obstacle_core_edge_plan(&prof, false, 0, false, 0);
        CHECK("S5-H20",
              (OBST_PLAN_RIGHT == prof.plan)
                  && (-(-25 + OBST_MARGIN_MM + (OBST_ROBOT_WIDTH_MM / 2)
                        + OBST_EDGE_ALLOW_MM)
                      == prof.deviation_mm));
        /* nothing from either scan: unchanged */
        memset(&prof, 0, sizeof(prof));
        prof.plan = OBST_PLAN_CONTINUE;
        obstacle_core_edge_plan(&prof, false, 0, false, 0);
        CHECK("S5-H20", OBST_PLAN_CONTINUE == prof.plan);
        /* without a servo reading: as far as the largest obstacle */
        memset(&prof, 0, sizeof(prof));
        obstacle_core_edge_plan(&prof, false, 0, true, -88);
        CHECK("S5-H17", ((-88 + OBST_WIDTH_MAX_MM) == prof.left_lat_mm));
    }
    {
        /* 4 Nov run: box 150 at 245 mm; clutter answering at 320 mm from 25 deg
           right on: its echoes are past where any 20 cm obstacle's edge can be,
           so the right edge stays the box's own and the offset clears the box
         */
        memset(&prof, 0, sizeof(prof));
        prof.clr_l_mm  = OBST_CLEAR_MAX_MM;
        prof.clr_r_mm  = OBST_CLEAR_MAX_MM;
        g_edge_clutter = 320u;
        prof.clr_l_mm =
            20u; /* 4 Nov: servo scan "saw" the box's left half as a blocker */
        edge_sim(245.0, 150.0, 0.0, &prof);
        g_edge_clutter = 0u;
        printf("edge scan with clutter right: edges %+d / %+d, plan %u, offset "
               "%d\n",
               prof.left_lat_mm, prof.right_lat_mm, (uint32_t)prof.plan,
               prof.deviation_mm);
        CHECK("S5-H17", prof.right_lat_mm >= -(OBST_WIDTH_MAX_MM));
        /* clutter only on the right: never pass on the right (towards it) */
        CHECK("S5-H17", OBST_PLAN_LEFT == prof.plan);
    }

    /* S5-H12: bypass path returns to the original line, clears the margin.
       3 Nov: simulated about the axle (sensor OBST_US_AHEAD_MM in front,
       line sensors OBST_IR_AHEAD_MM in front, rear LENGTH - US_AHEAD
       behind); JOIN ends where the line sensors reach the line (y = 0). */
    {
        box_t     box = {200.0, 150.0, 0.0};
        uint8_t   count;
        uint8_t   idx;
        float64_t x_val    = -OBST_US_AHEAD_MM;
        float64_t y_val    = 0.0;
        float64_t height   = 0.0;
        float64_t max_lat  = 0.0;
        bool      b_joined = false;

        scan_box(&box, 1, &prof);
        count = obstacle_core_bypass_plan(&prof, prof.plan, steps);
        for (idx = 0; idx < count; idx++)
        {
            float64_t       diff = ((OBST_STEP_TURN == steps[idx].kind)
                                    || (OBST_STEP_LINESCAN == steps[idx].kind))
                                       ? 0.0
                                       : (float64_t)steps[idx].value;
            float64_t const coef = cos((height * M_PI) / 180.0);
            float64_t const sine = sin((height * M_PI) / 180.0);

            if (OBST_STEP_TURN == steps[idx].kind)
            {
                height += steps[idx].value / 10.0;
            }
            if (OBST_STEP_LINESCAN == steps[idx].kind)
            {
                /* spins toward value's side until parallel to the line */
                CHECK("S5-H12",
                      ((steps[idx].value > 0) == (height < 0))
                          && (fabs(height) <= fabs(steps[idx].value / 10.0)));
                height = 0.0;
            }
            if (OBST_STEP_JOIN == steps[idx].kind)
            {
                diff     = (-(y_val + (OBST_IR_AHEAD_MM * sine)) / sine);
                b_joined = (diff >= 0.0) && (diff <= steps[idx].value);
            }
            x_val += diff * coef;
            y_val += diff * sine;
            if (fabs(y_val) > max_lat)
            {
                max_lat = fabs(y_val);
            }
            if (OBST_STEP_PASS == steps[idx].kind)
            {
                /* before turning back in, the rear is past the far side
                   of a 200 mm deep box (face at closest from the sensor) */
                CHECK("S5-H12",
                      (x_val - (OBST_ROBOT_LENGTH_MM - OBST_US_AHEAD_MM))
                          >= ((float64_t)prof.closest_mm + 150.0));
            }
        }
        printf("bypass: %u steps, axle ends (%.0f, %.0f) heading %.1f, max "
               "lateral %.0f mm\n",
               (uint32_t)count, x_val, y_val, height, max_lat);
        CHECK("S5-H12", b_joined && (count <= OBST_BYPASS_MAX_STEPS));
        CHECK("S5-H12", (fabs(y_val) < 2.0) && (fabs(height) < 0.1)
                            && (x_val > ((float64_t)prof.closest_mm + 150.0)));
        /* FR11: max deviation <= w/2 + robot/2 + 100 mm; clearance >= 50 mm */
        CHECK("S5-H12",
              max_lat <= (((150.0 / 2) + (OBST_ROBOT_WIDTH_MM / 2)) + 100.0));
        CHECK("S5-H12", ((max_lat - (OBST_ROBOT_WIDTH_MM / 2.0)) - 75.0)
                            >= OBST_MARGIN_MM);
    }
    {
        /* 45 deg turns, slow bypass, turns at the calibration spin rate */
        CHECK("S5-H12",
              (450 == OBST_TURN_OUT_DDEG)
                  && (OBST_BYPASS_TURN_DEG_S == MISSION_CAL_SPIN_RATE));
    }

    /* S5-H15 (3 Nov, instructor's guidance; the robot read a bottle as 2 mm
       and as 586 mm): */
    {
        /* (a) a 60 mm bottle: reported small, but planned as OBST_WIDTH_MAX
           around its nearest point, so the pass clears a 20 cm object */
        box_t         box  = {200.0, 60.0, 0.0};
        int32_t const need = (OBST_WIDTH_MAX_MM / 2) + (OBST_ROBOT_WIDTH_MM / 2)
                             + OBST_MARGIN_MM;

        scan_box(&box, 1, &prof);
        CHECK("S5-H15", prof.b_valid && !prof.b_sure
                            && ((int32_t)prof.w_mm < OBST_WIDTH_MIN_MM));
        CHECK("S5-H15",
              ((OBST_PLAN_LEFT == prof.plan) || (OBST_PLAN_RIGHT == prof.plan))
                  && (abs(prof.deviation_mm) >= need));
    }
    {
        /* (b) plan edges are never inside the estimated edges */
        box_t box = {200.0, 150.0, 20.0};

        scan_box(&box, 1, &prof);
        CHECK("S5-H15", prof.b_sure && (prof.plan_l_mm >= prof.left_lat_mm)
                            && (prof.plan_r_mm <= prof.right_lat_mm));
    }
    {
        /* (f) guard: the profile itself sees nothing usable (no fine
           scan), yet the raw reading straight ahead is
           150 mm: plan a bypass for a 20 cm object, never CONTINUE */
        obstacle_core_point_t pts[5];
        uint8_t               kdx;

        for (kdx = 0u; kdx < T_CORNER_PTS; kdx++)
        {
            pts[kdx].angle_deg = (int16_t)(30 + (30 * (int32_t)kdx));
            pts[kdx].r_mm      = OBST_RANGE_NONE;
        }
        pts[2].r_mm = 150u; /* 90 deg */
        obstacle_core_profile(pts, 0u, pts, 5u,
                              &prof); /* no fine scan at all */
        CHECK("S5-H15",
              (prof.plan != OBST_PLAN_CONTINUE) && (150u == prof.closest_mm));
        CHECK("S5-H15", (prof.plan_l_mm >= (OBST_WIDTH_MAX_MM / 2))
                            && (prof.plan_r_mm <= -(OBST_WIDTH_MAX_MM / 2)));
    }
    {
        /* (d) one dropout inside a face does not split it (2-miss edges) */
        obstacle_core_point_t pts[13];
        uint8_t               kdx;

        for (kdx = 0u; kdx < T_WALL_PTS; kdx++)
        {
            pts[kdx].angle_deg = (int16_t)(60 + (5 * (int32_t)kdx));
            pts[kdx].r_mm =
                ((0u == kdx) || (12u == kdx)) ? OBST_RANGE_NONE : 200u;
        }
        pts[6].r_mm = OBST_RANGE_NONE; /* the dropout, straight ahead */
        obstacle_core_profile(pts, 13u, pts, 0u, &prof);
        CHECK("S5-H15", prof.b_valid && (prof.left_lat_mm > 50)
                            && (prof.right_lat_mm < -50));
    }

    /* S5-H13: seek-line acceptance window and the hump mask */
    CHECK("S5-H13", obstacle_core_line_accept(500, 40, 1000, 0, 0, 0));
    CHECK("S5-H13", !obstacle_core_line_accept(500, 80, 0, 0, 0, 0));
    CHECK("S5-H13", !obstacle_core_line_accept(500, 0, 2500, 0, 0, 0));
    CHECK("S5-H13",
          (0 == obstacle_core_cross_track_mm(0, 100, 0, 0, 9000))
              && (-100 == obstacle_core_cross_track_mm(100, 0, 0, 0, 9000)));
    {
        obstacle_core_mask_t mask = {false, 0u, 0};
        CHECK("S5-H13", !obstacle_core_hump_mask(&mask, true, 200, 0, 1000u));
        CHECK("S5-H13", obstacle_core_hump_mask(&mask, true, 700, 0, 1100u));
        CHECK("S5-H13", obstacle_core_hump_mask(&mask, true, 700, 500, 2000u));
        CHECK("S5-H13", !obstacle_core_hump_mask(&mask, true, 700, 650,
                                                 2100u)); /* > 600 mm: lapsed */
        CHECK("S5-H13", !obstacle_core_hump_mask(&mask, true, 700, 700, 2200u));
        CHECK("S5-H13", !obstacle_core_hump_mask(&mask, false, 700, 700,
                                                 2300u)); /* pitch not valid */
        CHECK("S5-H13", obstacle_core_hump_mask(&mask, true, 700, 700,
                                                2400u)); /* re-armed */
        CHECK("S5-H13", !obstacle_core_hump_mask(&mask, true, 700, 700,
                                                 6500u)); /* > 4 s */
    }
}

static void test_bench (void)
{
#if APP_BENCH
    /* S5-H14: bench mode (APP_BENCH=1): set bench_* in IDLE drives the hardware
     */
    boot_to_idle();
    ev(MISSION_EV_REMOTE_SET, 101u, 500);
    CHECK(
        "S5-H14",
        (find(MISSION_ACT_MOTION, MOTION_CMD_MOVE) >= 0)
            && (500
                == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_MOVE)].arg0));
    ev(MISSION_EV_REMOTE_SET, 102u, -900);
    CHECK("S5-H14",
          -900 == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].arg0);
    ev(MISSION_EV_REMOTE_SET, 103u, 300);
    CHECK("S5-H14",
          300 == g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_MOVE)].arg1);
    CHECK("S5-H14", MISSION_CRUISE_MM_S
                        == g_mc.snap.cruise_mm_s); /* not a run parameter */
    printf("bench mode checks run\n");
#else
    /* without APP_BENCH the ids are not parameters at all: nothing happens */
    boot_to_idle();
    ev(MISSION_EV_REMOTE_SET, 101u, 500);
    CHECK("S5-H14", 0u == g_out.n_acts);
#endif
}

/* S5-H26: Change Requirement 1, packed mbf_mission / mbf_scan and snapshots */
static void test_pack_s5 (void)
{
    mission_types_evt_t       evt;
    mission_types_evt_t       decoded;
    mission_types_scan_req_t  req;
    mission_types_scan_req_t  req_out;
    mission_types_snap_t      mission;
    mission_types_snap_t      mission_r;
    mission_types_obst_snap_t snap_o;
    mission_types_obst_snap_t orr;
    uint32_t                  words_min[MISSION_SNAP_WORDS];
    uint32_t                  words_out[OBST_SNAP_WORDS];
    static int32_t const      datas[] = {0,   -1,   65535, -65536,
                                         'F', 3600, -3600, 0x07FF};
    uint32_t                  kdx;
    bool                      b_ok = true;

    for (kdx = 0u; kdx < (sizeof(datas) / sizeof(datas[0])); kdx++)
    {
        evt.type = MISSION_EV_REMOTE_SET;
        evt.src  = 5u;
        evt.arg  = (uint16_t)(kdx * 15u);
        evt.data = datas[kdx];
        mission_core_evt_unpack(mission_core_evt_pack(&evt), &decoded);
        if ((decoded.type != evt.type) || (decoded.src != evt.src)
            || (decoded.arg != evt.arg) || (decoded.data != evt.data))
        {
            b_ok = false;
        }
    }
    CHECK("S5-H26", b_ok);
    evt.type = MISSION_EV_OBSTACLE;
    evt.arg  = 200u;
    evt.data = 100000; /* saturate */
    mission_core_evt_unpack(mission_core_evt_pack(&evt), &decoded);
    CHECK("S5-H26", (127u == decoded.arg) && (65535 == decoded.data));
    evt.type = MISSION_EV_REMOTE_MAP;
    evt.src  = 1u;
    evt.arg  = 0u;
    evt.data = (int32_t)((uint32_t)'A' | ((uint32_t)'F' << 8)
                         | ((uint32_t)'Z' << 16) | ((uint32_t)'%' << 24));
    mission_core_evt_unpack(mission_core_evt_pack(&evt), &decoded);
    CHECK("S5-H26", (MISSION_EV_REMOTE_MAP == decoded.type)
                        && (decoded.data == evt.data) && (1u == decoded.src));
    CHECK("S5-H26", MISSION_EV_COUNT <= 32u);
    req.type  = SCAN_REQ_STOP;
    req.seq   = 37u;
    req.param = 180;
    mission_core_scan_req_unpack(mission_core_scan_req_pack(&req), &req_out);
    CHECK("S5-H26", (req_out.type == req.type) && (5u == req_out.seq)
                        && (180 == req_out.param));

    CHECK("S5-H26", (MISSION_SNAP_BITS == mission_core_snap_layout_bits())
                        && (MISSION_SNAP_WORDS == PK_WORDS(MISSION_SNAP_BITS))
                        && (OBST_SNAP_BITS == obstacle_core_snap_layout_bits())
                        && (OBST_SNAP_WORDS == PK_WORDS(OBST_SNAP_BITS)));
    memset(&mission, 0, sizeof(mission));
    mission.t_ms           = 777u;
    mission.fault_bits     = 0x0400u | (0xFu << 16);
    mission.cruise_mm_s    = 350u;
    mission.speed_cap_mm_s = 150u;
    mission.slow_mm        = 600u;
    mission.stop_mm        = 400u;
    mission.lost_mm        = 150u;
    mission.junction_mm    = 40u;
    mission.runs           = 12u;
    mission.state          = 10u;
    mission.auth           = 2u;
    mission.pending_nav    = MISSION_NAV_NONE;
    mission.pending_char   = '\0';
    mission.nav_map[0]     = 'A';
    mission.nav_map[1]     = 'F';
    mission.nav_map[2]     = 'Z';
    mission.nav_map[3]     = 'W';
    mission.calibrated     = 3u;
    mission.cal_phase      = 3u;
    mission.cause          = 6u;
    mission_core_snap_pack(&mission, words_min);
    memset(&mission_r, 0x5A,
           sizeof(mission_r)); /* every field must be written */
    mission_core_snap_unpack(words_min, &mission_r);
    {
        uint32_t words2[MISSION_SNAP_WORDS];

        mission_core_snap_pack(&mission_r, words2);
        CHECK("S5-H26", 0 == memcmp(words_min, words2, sizeof(words_min)));
    }
    memset(&mission_r, 0, sizeof(mission_r)); /* same padding as m */
    mission_core_snap_unpack(words_min, &mission_r);
    CHECK("S5-H26", 0 == memcmp(&mission, &mission_r, sizeof(mission)));
    mission.pending_nav  = 2u;
    mission.pending_char = 'Q';
    mission_core_snap_pack(&mission, words_min);
    mission_core_snap_unpack(words_min, &mission_r);
    CHECK("S5-H26",
          (2u == mission_r.pending_nav) && ('Q' == mission_r.pending_char));

    memset(&snap_o, 0, sizeof(snap_o));
    snap_o.t_ms         = 5u;
    snap_o.pings        = 4000000000u;
    snap_o.d_mm         = OBST_RANGE_NONE;
    snap_o.w_mm         = 1600u;
    snap_o.closest_mm   = 4094u;
    snap_o.clr_l_mm     = OBST_RANGE_NONE;
    snap_o.clr_r_mm     = 500u;
    snap_o.left_lat_mm  = -2047;
    snap_o.right_lat_mm = 812;
    snap_o.no_echo      = 255u;
    snap_o.valid        = 1u;
    snap_o.plan         = 4u;
    snap_o.state        = 12u;
    snap_o.retries      = 2u;
    obstacle_core_snap_pack(&snap_o, words_out);
    memset(&orr, 0x5A, sizeof(orr)); /* every field must be written */
    obstacle_core_snap_unpack(words_out, &orr);
    {
        uint32_t words2[OBST_SNAP_WORDS];

        obstacle_core_snap_pack(&orr, words2);
        CHECK("S5-H26", 0 == memcmp(words_out, words2, sizeof(words_out)));
    }
    memset(&orr, 0, sizeof(orr)); /* same padding as o */
    obstacle_core_snap_unpack(words_out, &orr);
    CHECK("S5-H26", 0 == memcmp(&snap_o, &orr, sizeof(snap_o)));
    snap_o.d_mm    = 4095u;
    snap_o.no_echo = 1000u; /* 4095 is not "none": saturates below it */
    obstacle_core_snap_pack(&snap_o, words_out);
    obstacle_core_snap_unpack(words_out, &orr);
    CHECK("S5-H26", (4094u == orr.d_mm) && (255u == orr.no_echo));
}
/*** end of file ***/

/*** end of file ***/
