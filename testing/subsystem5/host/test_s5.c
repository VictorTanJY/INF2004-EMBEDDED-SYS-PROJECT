/*
 *  test_s5.c
 *  Host unit tests for subsystem 5: the system statechart (mission_core.c)
 *  and the obstacle geometry (obstacle_core.c).
 *  Obstacles are simulated as flat-faced boxes seen by a 15 deg cone: a
 *  ping returns the nearest face point inside +-7.5 deg of the servo angle.
 *  That is a model of the HC-SR04, not the sensor: width accuracy on the
 *  real robot is measured on boxes of known width (A11, FR10).
 *  Build and run:  make -C testing/subsystem5/host
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mission_core.h"
#include "mission_cfg.h"
#include "obstacle_core.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

static mission_core_t   g_mc;
static mission_out_t    g_out;
static mission_inputs_t g_in;
static uint32_t         g_now;

static void ev(uint8_t type, uint16_t a, int32_t data)
{
    mission_evt_t e = { type, 0u, a, data };

    g_out.n = 0u;
    mission_core_event(&g_mc, &e, g_now, &g_out);
}

static void tick(uint32_t ms)
{
    uint32_t t;

    for (t = 0u; t < ms; t += 100u)
    {
        g_now += 100u;
        g_out.n = 0u;
        mission_core_tick(&g_mc, &g_in, g_now, &g_out);
        if (g_out.n > 0u) { return; }   /* caller inspects the actions */
    }
}

/* Find an action; returns its index or -1. */
static int find(uint8_t kind, uint8_t cmd)
{
    uint8_t i;

    for (i = 0u; i < g_out.n; i++)
    {
        if ((g_out.act[i].kind == kind) && (g_out.act[i].cmd == cmd)) { return (int)i; }
    }
    return -1;
}

static uint8_t last_seq(void)
{
    int i;

    for (i = (int)g_out.n - 1; i >= 0; i--)
    {
        if ((g_out.act[i].kind == MISSION_ACT_MOTION) && (g_out.act[i].seq != 0u)) { return g_out.act[i].seq; }
    }
    return 0u;
}

static void boot_to_idle(void)
{
    g_now = 1000u;
    memset(&g_in, 0, sizeof(g_in));
    mission_core_init(&g_mc, g_now);
    g_in.b_motion_hw_ok = true;
    g_in.b_line_hw_ok = true;
    g_in.b_imu_ok = true;
    g_in.b_timer_checked = true;
    g_in.b_timer_ok = true;
    tick(100u);
}

static void calibrate(void)
{
    uint8_t seq;

    ev(MISSION_EV_BTN_CALIBRATE, 0u, 0);
    tick(1100u);                                    /* STILL -> SPIN */
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    ev(MISSION_EV_CAL_DONE, 3u, 1);
    ev(MISSION_EV_CAL_DONE, 4u, 1);
    tick(100u);
}

static void test_mission(void)
{
    char map[4];
    uint8_t seq;

    /* S5-H01: BOOT -> IDLE when every self-test passed; -> FAULT after 3 s otherwise */
    boot_to_idle();
    CHECK("S5-H01", g_mc.snap.state == MISSION_ST_IDLE);
    g_now = 0u;
    mission_core_init(&g_mc, g_now);
    g_in.b_imu_ok = false;
    tick(2900u);
    CHECK("S5-H01", g_mc.snap.state == MISSION_ST_BOOT);
    tick(300u);
    CHECK("S5-H01", g_mc.snap.state == MISSION_ST_FAULT && g_mc.snap.cause == MISSION_CAUSE_SELFTEST);

    /* S5-H02: START refused until calibrated; calibration sequence */
    boot_to_idle();
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    CHECK("S5-H02", g_mc.snap.state == MISSION_ST_IDLE && find(MISSION_ACT_LOG, MISSION_LOG_NOT_CALIBRATED) >= 0);
    ev(MISSION_EV_BTN_CALIBRATE, 0u, 0);
    CHECK("S5-H02", g_mc.snap.state == MISSION_ST_CALIBRATING && g_mc.snap.cal_phase == MISSION_CAL_PH_STILL);
    tick(1100u);
    CHECK("S5-H02", g_mc.snap.cal_phase == MISSION_CAL_PH_SPIN && find(MISSION_ACT_MOTION, MOTION_CMD_TURN) >= 0);
    CHECK("S5-H02", g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].p0 == 3600);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, (uint16_t)(seq + 1u), MOTION_SRC_MISSION);    /* stale seq: ignored */
    CHECK("S5-H02", g_mc.snap.cal_phase == MISSION_CAL_PH_SPIN);
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H02", g_mc.snap.cal_phase == MISSION_CAL_PH_FINISH);
    ev(MISSION_EV_CAL_DONE, 3u, 1);
    ev(MISSION_EV_CAL_DONE, 4u, 0);                                          /* IMU failed */
    tick(100u);
    CHECK("S5-H02", g_mc.snap.state == MISSION_ST_IDLE && g_mc.snap.calibrated == MISSION_CAL_IR);
    calibrate();
    CHECK("S5-H02", g_mc.snap.calibrated == MISSION_CAL_ALL);

    /* S5-H03: MQTT map (not hard-coded): no map -> barcodes reported only */
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    CHECK("S5-H03", g_mc.snap.state == MISSION_ST_LINE_FOLLOW && g_mc.snap.auth == MOTION_SRC_LINE);
    CHECK("S5-H03", find(MISSION_ACT_SCAN, SCAN_REQ_MONITOR) >= 0 && find(MISSION_ACT_NEW_RUN, 0u) >= 0);
    ev(MISSION_EV_BARCODE, 0u, 'A');
    CHECK("S5-H03", g_mc.snap.pending_nav == MISSION_NAV_NONE && find(MISSION_ACT_LOG, MISSION_LOG_NAV_UNMAPPED) >= 0);
    memcpy(map, "AFZW", 4);
    ev(MISSION_EV_REMOTE_MAP, 0u, mission_map_pack(map));                   /* refused while running */
    CHECK("S5-H03", g_mc.snap.nav_map[0] == '\0');
    ev(MISSION_EV_REMOTE_STOP, 0u, 0);
    CHECK("S5-H03", g_mc.snap.state == MISSION_ST_MISSION_COMPLETE && find(MISSION_ACT_LOG, MISSION_LOG_SUMMARY) >= 0);
    memcpy(map, "AFAW", 4);
    ev(MISSION_EV_REMOTE_MAP, 0u, mission_map_pack(map));                   /* duplicate: refused */
    CHECK("S5-H03", g_mc.snap.nav_map[0] == '\0' && find(MISSION_ACT_LOG, MISSION_LOG_MAP_BAD) >= 0);
    memcpy(map, "AF*W", 4);
    CHECK("S5-H03", !mission_map_unpack(mission_map_pack(map), map));
    memcpy(map, "QFZW", 4);                     /* left Q, right F, straight Z, U-turn W */
    ev(MISSION_EV_REMOTE_MAP, 0u, mission_map_pack(map));
    CHECK("S5-H03", memcmp(g_mc.snap.nav_map, "QFZW", 4) == 0);

    /* S5-H04: barcode latched, executed at the NEXT junction (FR4) */
    ev(MISSION_EV_BTN_START_STOP, 0u, 0);
    CHECK("S5-H04", g_mc.snap.state == MISSION_ST_LINE_FOLLOW && g_mc.snap.runs == 2u);
    ev(MISSION_EV_BARCODE, 0u, 'F');
    CHECK("S5-H04", g_mc.snap.pending_nav == MISSION_NAV_RIGHT && g_mc.snap.pending_char == 'F');
    ev(MISSION_EV_BARCODE, 0u, 'X');            /* unmapped: does not clear the latch */
    CHECK("S5-H04", g_mc.snap.pending_nav == MISSION_NAV_RIGHT);
    ev(MISSION_EV_JUNCTION, 0u, 0);
    CHECK("S5-H04", g_mc.snap.state == MISSION_ST_EXECUTE_NAV && g_mc.snap.auth == MOTION_SRC_MISSION);
    CHECK("S5-H04", find(MISSION_ACT_MOTION, MOTION_CMD_MOVE) >= 0 && g_mc.snap.pending_nav == MISSION_NAV_NONE);
    seq = last_seq();
    ev(MISSION_EV_BARCODE, 0u, 'Q');            /* read during the manoeuvre: latched for later */
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04", find(MISSION_ACT_MOTION, MOTION_CMD_TURN) >= 0 &&
                    g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].p0 == -900);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04", g_mc.snap.state == MISSION_ST_LINE_FOLLOW && g_mc.snap.auth == MOTION_SRC_LINE);
    CHECK("S5-H04", g_mc.snap.pending_nav == MISSION_NAV_LEFT);
    ev(MISSION_EV_JUNCTION, 0u, 0);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04", g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].p0 == 900);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    ev(MISSION_EV_BARCODE, 0u, 'W');
    ev(MISSION_EV_JUNCTION, 0u, 0);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04", g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].p0 == 1800);   /* U-turn */
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    ev(MISSION_EV_BARCODE, 0u, 'Z');
    ev(MISSION_EV_JUNCTION, 0u, 0);
    CHECK("S5-H04", find(MISSION_ACT_MOTION, MOTION_CMD_MOVE) >= 0 &&
                    g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_MOVE)].p0 == MISSION_NAV_STRAIGHT_MM);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H04", g_mc.snap.state == MISSION_ST_LINE_FOLLOW);
    ev(MISSION_EV_JUNCTION, 0u, 0);             /* no latch: straight through */
    CHECK("S5-H04", g_mc.snap.state == MISSION_ST_LINE_FOLLOW);

    /* S5-H05: obstacle -> profile -> bypass -> recovery -> line follow */
    ev(MISSION_EV_SLOW_DOWN, 0u, 380);
    CHECK("S5-H05", g_mc.snap.speed_cap_mm_s == MISSION_SLOW_SPEED_MM_S);
    ev(MISSION_EV_OBSTACLE, 0u, 190);
    CHECK("S5-H05", g_mc.snap.state == MISSION_ST_OBSTACLE_HANDLING && g_mc.snap.auth == MOTION_SRC_AVOID);
    CHECK("S5-H05", find(MISSION_ACT_MOTION, MOTION_CMD_STOP) >= 0 && find(MISSION_ACT_SCAN, SCAN_REQ_PROFILE) >= 0);
    ev(MISSION_EV_PROFILE_DONE, OBST_PLAN_LEFT, 150);
    CHECK("S5-H05", g_mc.snap.state == MISSION_ST_AVOIDING && find(MISSION_ACT_SCAN, SCAN_REQ_BYPASS_LEFT) >= 0);
    ev(MISSION_EV_BYPASS_DONE, 0u, 0);
    CHECK("S5-H05", g_mc.snap.state == MISSION_ST_LINE_RECOVERY && find(MISSION_ACT_SCAN, SCAN_REQ_RECOVER) >= 0);
    ev(MISSION_EV_RECOVERY_DONE, 0u, 0);
    CHECK("S5-H05", g_mc.snap.state == MISSION_ST_LINE_FOLLOW && g_mc.snap.speed_cap_mm_s == MISSION_CRUISE_MM_S);
    ev(MISSION_EV_OBSTACLE, 0u, 190);
    ev(MISSION_EV_PROFILE_DONE, OBST_PLAN_CONTINUE, 0);
    CHECK("S5-H05", g_mc.snap.state == MISSION_ST_LINE_FOLLOW);
    ev(MISSION_EV_OBSTACLE, 0u, 190);
    ev(MISSION_EV_BLOCKED_TIMEOUT, 0u, 0);
    CHECK("S5-H05", g_mc.snap.state == MISSION_ST_FAULT && g_mc.snap.cause == MISSION_CAUSE_BLOCKED);

    /* S5-H06: FAULT needs an operator reset that SafetyTask accepts */
    ev(MISSION_EV_BTN_RESET, 0u, 0);
    CHECK("S5-H06", g_mc.snap.state == MISSION_ST_FAULT && find(MISSION_ACT_SAFETY_RESET, 0u) >= 0);
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    CHECK("S5-H06", g_mc.snap.state == MISSION_ST_FAULT);
    ev(MISSION_EV_SAFETY_OK, 0u, 0);
    CHECK("S5-H06", g_mc.snap.state == MISSION_ST_BOOT && find(MISSION_ACT_MOTION, MOTION_CMD_CLEAR_ESTOP) >= 0);
    tick(100u);
    CHECK("S5-H06", g_mc.snap.state == MISSION_ST_IDLE && g_mc.snap.calibrated == MISSION_CAL_ALL);
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    ev(MISSION_EV_FAULT, 0u, 0x0002);           /* TOO_CLOSE from SafetyTask */
    CHECK("S5-H06", g_mc.snap.state == MISSION_ST_FAULT && g_mc.snap.fault_bits == 0x0002u);

    /* S5-H07: line lost -> bounded +-60 deg search -> FAULT, or found */
    ev(MISSION_EV_SAFETY_OK, 0u, 0);
    tick(100u);
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    ev(MISSION_EV_LINE_LOST, 0u, 0);
    CHECK("S5-H07", g_mc.snap.state == MISSION_ST_LINE_SEARCH && g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].p0 == 600);
    seq = last_seq();
    ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H07", g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].p0 == -1200);
    g_in.b_on_line = true;
    tick(200u);
    CHECK("S5-H07", g_mc.snap.state == MISSION_ST_LINE_FOLLOW);
    g_in.b_on_line = false;
    ev(MISSION_EV_LINE_LOST, 0u, 0);
    seq = last_seq(); ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    seq = last_seq(); ev(MISSION_EV_MOTION_FAILED, seq, MOTION_SRC_MISSION);
    seq = last_seq(); ev(MISSION_EV_MOTION_DONE, seq, MOTION_SRC_MISSION);
    CHECK("S5-H07", g_mc.snap.state == MISSION_ST_FAULT && g_mc.snap.cause == MISSION_CAUSE_LINE_LOST);

    /* S5-H08: an mbf_mission drop while running is a fault; set only in IDLE */
    ev(MISSION_EV_SAFETY_OK, 0u, 0);
    tick(100u);
    ev(MISSION_EV_REMOTE_SET, 1u, 300);
    CHECK("S5-H08", g_mc.snap.cruise_mm_s == 300u && find(MISSION_ACT_LOG, MISSION_LOG_PARAM_SET) >= 0);
    ev(MISSION_EV_REMOTE_SET, 3u, 25);
    CHECK("S5-H08", g_mc.snap.stop_mm == 250u);
    ev(MISSION_EV_REMOTE_START, 0u, 0);
    ev(MISSION_EV_REMOTE_SET, 1u, 150);
    CHECK("S5-H08", g_mc.snap.cruise_mm_s == 300u);
    ev(MISSION_EV_HUMP_START, 0u, 0);
    CHECK("S5-H08", g_mc.snap.speed_cap_mm_s == MISSION_HUMP_SPEED_MM_S);
    ev(MISSION_EV_HUMP_END, 0u, 21);
    CHECK("S5-H08", g_mc.snap.speed_cap_mm_s == 300u);
    g_in.mbf_drops = 1u;
    tick(100u);
    CHECK("S5-H08", g_mc.snap.state == MISSION_ST_FAULT && g_mc.snap.cause == MISSION_CAUSE_MBF_OVERFLOW);
}

/* ---- obstacle geometry --------------------------------------------- */
typedef struct { double d, w, c; } box_t;       /* face distance, width, centre lateral (+ left) */

static uint16_t ping(box_t const * p_b, int n_b, int angle_deg)
{
    double best = 1e9;
    int k;

    for (k = 0; k < n_b; k++)
    {
        double lat;

        for (lat = p_b[k].c - p_b[k].w / 2.0; lat <= p_b[k].c + p_b[k].w / 2.0 + 1e-9; lat += 1.0)
        {
            double a = atan2(p_b[k].d, -lat) * 180.0 / M_PI;
            double r = sqrt(p_b[k].d * p_b[k].d + lat * lat);

            if ((fabs(a - angle_deg) <= 7.5) && (r < best)) { best = r; }
        }
    }
    return (best > 4000.0) ? OBST_RANGE_NONE : (uint16_t)lround(best);
}

static int g_pings;

static void scan_box(box_t const * p_b, int n_b, obst_profile_t * p_prof)
{
    obst_scan_t sc;

    obst_scan_start(&sc);
    g_pings = 0;
    while (!obst_scan_done(&sc) && (g_pings < 60))
    {
        obst_scan_feed(&sc, ping(p_b, n_b, obst_scan_angle(&sc)));
        g_pings++;
    }
    obst_scan_profile(&sc, p_prof);
}

static void test_obstacle(void)
{
    obst_profile_t prof;
    obst_step_t steps[OBST_BYPASS_MAX_STEPS];
    double worst = 0.0;
    int w, c;

    /* S5-H09: helpers */
    CHECK("S5-H09", obst_median3(300, 120, 200) == 200 && obst_median3(OBST_RANGE_NONE, 100, 90) == 100);
    CHECK("S5-H09", obst_echo_to_mm(1160u) == 200u && obst_echo_to_mm(23200u) == OBST_RANGE_NONE);
    CHECK("S5-H09", obst_echo_to_mm(0u) == OBST_RANGE_NONE);

    /* S5-H10: width of 10-20 cm boxes (instructor, 23 Oct) at 20 cm, centred
       and offset; model error target: within 10 % (grade A) */
    for (w = 100; w <= 200; w += 50)
    {
        for (c = -40; c <= 40; c += 40)
        {
            box_t b = { 200.0, (double)w, (double)c };
            double err;

            scan_box(&b, 1, &prof);
            err = fabs((double)prof.w_mm - w) / w;
            if (err > worst) { worst = err; }
            printf("box %3d mm at %+3d mm: width %3u mm  edges %+d/%+d  closest %u  plan %u  pings %d\n",
                   w, c, (unsigned)prof.w_mm, prof.left_lat_mm, prof.right_lat_mm,
                   (unsigned)prof.closest_mm, (unsigned)prof.plan, g_pings);
            CHECK("S5-H10", g_pings <= 30);       /* ~1.8 s of 60 ms pings */
            CHECK("S5-H10", prof.b_valid && prof.closest_mm >= 195u && prof.closest_mm <= 215u);
            CHECK("S5-H10", (prof.plan == OBST_PLAN_LEFT) || (prof.plan == OBST_PLAN_RIGHT));   /* free space around */
        }
    }
    printf("worst width error on the beam model: %.1f %%\n", worst * 100.0);
    CHECK("S5-H10", worst <= 0.10);

    /* S5-H11: plan: free both sides -> the side needing less deviation */
    {
        box_t b = { 200.0, 150.0, 30.0 };       /* centred a little to the left */
        scan_box(&b, 1, &prof);
        CHECK("S5-H11", prof.plan == OBST_PLAN_RIGHT && prof.deviation_mm < 0);
    }
    {
        box_t b[2] = { { 200.0, 150.0, 0.0 }, { 260.0, 120.0, -200.0 } };   /* wall on the right */
        scan_box(b, 2, &prof);
        CHECK("S5-H11", prof.plan == OBST_PLAN_LEFT && prof.clr_r_mm < (OBST_ROBOT_WIDTH_MM + 2 * OBST_MARGIN_MM));
    }
    {
        box_t b[3] = { { 200.0, 150.0, 0.0 }, { 250.0, 120.0, -200.0 }, { 250.0, 120.0, 200.0 } };
        scan_box(b, 3, &prof);
        CHECK("S5-H11", prof.plan == OBST_PLAN_BLOCKED);
    }
    {
        box_t b = { 200.0, 100.0, 260.0 };      /* beside the path */
        scan_box(&b, 1, &prof);
        CHECK("S5-H11", prof.plan == OBST_PLAN_CONTINUE);
    }
    {
        obst_point_t none[3] = { { 80, OBST_RANGE_NONE }, { 90, OBST_RANGE_NONE }, { 100, OBST_RANGE_NONE } };
        obst_profile(none, 3u, none, 0u, &prof);
        CHECK("S5-H11", !prof.b_valid && prof.plan == OBST_PLAN_CONTINUE);
    }

    /* S5-H12: bypass path returns to the original line, clears the margin */
    {
        box_t b = { 200.0, 150.0, 0.0 };
        uint8_t n, i;
        double x = 0.0, y = 0.0, h = 0.0, max_lat = 0.0;

        scan_box(&b, 1, &prof);
        n = obst_bypass_plan(&prof, prof.plan, steps);
        for (i = 0; i < n; i++)
        {
            double d = (steps[i].kind == OBST_STEP_TURN) ? 0.0 : (double)steps[i].value;
            if (steps[i].kind == OBST_STEP_TURN) { h += steps[i].value / 10.0; }
            x += d * cos(h * M_PI / 180.0);
            y += d * sin(h * M_PI / 180.0);
            if (fabs(y) > max_lat) { max_lat = fabs(y); }
        }
        printf("bypass: %u steps, end (%.0f, %.0f) heading %.1f, max lateral %.0f mm\n",
               (unsigned)n, x, y, h, max_lat);
        CHECK("S5-H12", fabs(y) < 2.0 && fabs(h) < 0.1 && x > 300.0);
        /* FR11: max deviation <= w/2 + robot/2 + 100 mm; clearance >= 50 mm */
        CHECK("S5-H12", max_lat <= 150.0 / 2 + OBST_ROBOT_WIDTH_MM / 2 + 100.0);
        CHECK("S5-H12", max_lat - OBST_ROBOT_WIDTH_MM / 2.0 - 75.0 >= OBST_MARGIN_MM);
    }

    /* S5-H13: seek-line acceptance window and the hump mask */
    CHECK("S5-H13", obst_line_accept(500, 40, 1000, 0, 0, 0));
    CHECK("S5-H13", !obst_line_accept(500, 80, 0, 0, 0, 0));
    CHECK("S5-H13", !obst_line_accept(500, 0, 2500, 0, 0, 0));
    CHECK("S5-H13", obst_cross_track_mm(0, 100, 0, 0, 9000) == 0 && obst_cross_track_mm(100, 0, 0, 0, 9000) == -100);
    {
        obst_mask_t m = { false, 0u, 0 };
        CHECK("S5-H13", !obst_hump_mask(&m, true, 200, 0, 1000u));
        CHECK("S5-H13", obst_hump_mask(&m, true, 700, 0, 1100u));
        CHECK("S5-H13", obst_hump_mask(&m, true, 700, 500, 2000u));
        CHECK("S5-H13", !obst_hump_mask(&m, true, 700, 650, 2100u));           /* > 600 mm: lapsed */
        CHECK("S5-H13", !obst_hump_mask(&m, true, 700, 700, 2200u));
        CHECK("S5-H13", !obst_hump_mask(&m, false, 700, 700, 2300u));          /* pitch not valid */
        CHECK("S5-H13", obst_hump_mask(&m, true, 700, 700, 2400u));            /* re-armed */
        CHECK("S5-H13", !obst_hump_mask(&m, true, 700, 700, 6500u));           /* > 4 s */
    }
}

static void test_bench(void)
{
#if APP_BENCH
    /* S5-H14: bench mode (APP_BENCH=1): set bench_* in IDLE drives the hardware */
    boot_to_idle();
    ev(MISSION_EV_REMOTE_SET, 101u, 500);
    CHECK("S5-H14", find(MISSION_ACT_MOTION, MOTION_CMD_MOVE) >= 0 &&
                    g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_MOVE)].p0 == 500);
    ev(MISSION_EV_REMOTE_SET, 102u, -900);
    CHECK("S5-H14", g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_TURN)].p0 == -900);
    ev(MISSION_EV_REMOTE_SET, 103u, 300);
    CHECK("S5-H14", g_out.act[find(MISSION_ACT_MOTION, MOTION_CMD_MOVE)].p1 == 300);
    ev(MISSION_EV_REMOTE_SET, 104u, 45);
    CHECK("S5-H14", find(MISSION_ACT_SCAN, SCAN_REQ_BENCH_SERVO) >= 0 &&
                    g_out.act[find(MISSION_ACT_SCAN, SCAN_REQ_BENCH_SERVO)].p0 == 45);
    CHECK("S5-H14", g_mc.snap.cruise_mm_s == MISSION_CRUISE_MM_S);     /* not a run parameter */
    printf("bench mode checks run\n");
#else
    /* without APP_BENCH the ids are not parameters at all: nothing happens */
    boot_to_idle();
    ev(MISSION_EV_REMOTE_SET, 101u, 500);
    CHECK("S5-H14", g_out.n == 0u);
#endif
}

int main(void)
{
    test_mission();
    test_bench();
    test_obstacle();
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
