/*
 *  test_s2.c
 *  Host unit tests for motion_core.c (subsystem 2) against a motor model:
 *  per wheel, steady speed = 0.5 mm/s per per-mille of duty above a 100
 *  per-mille deadband, first-order lag tau = 50 ms.  The model is only a
 *  stand-in for the real motors: these tests check the controller LOGIC
 *  (authority, profiles, timeouts, faults), not the real tuning, which
 *  must be measured on the robot (NFR4, NFR5).
 *  Build and run:  make -C testing/subsystem2/host
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motion_core.h"
#include "motion_cfg.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

typedef struct
{
    double v[2];            /* mm/s  */
    double pos_um[2];       /* wheel distance */
    double gain;            /* mm/s per per-mille above the deadband */
    int    locked[2];
    int32_t enc_offset;
} plant_t;

static motion_core_t g_mc;
static plant_t       g_pl;
static uint32_t      g_now;
static int           g_ev[8];       /* counts per motion_ev_code_t */
static uint8_t       g_last_seq;

static void plant_init(double gain)
{
    memset(&g_pl, 0, sizeof(g_pl));
    g_pl.gain = gain;
}

static void plant_step(int16_t duty_l, int16_t duty_r)
{
    int16_t const duty[2] = { duty_l, duty_r };
    int w;

    for (w = 0; w < 2; w++)
    {
        double d  = (double)duty[w];
        double ss = 0.0;

        if (d > 100.0)       { ss = (d - 100.0) * g_pl.gain; }
        else if (d < -100.0) { ss = (d + 100.0) * g_pl.gain; }
        if ((duty[w] == 0) || g_pl.locked[w]) { ss = 0.0; }     /* brake */
        g_pl.v[w] += (ss - g_pl.v[w]) * (10.0 / 50.0);
        if (g_pl.locked[w]) { g_pl.v[w] = 0.0; }
        g_pl.pos_um[w] += g_pl.v[w] * 10.0;                    /* mm/s * 10 ms = um */
    }
}

static int32_t enc_of(int w)
{
    double counts = g_pl.pos_um[w] * MOTION_COUNTS_PER_REV / MOTION_WHEEL_CIRC_UM;
    int32_t c = (int32_t)(counts >= 0 ? counts + 0.5 : counts - 0.5);
    int32_t sign = (w == 0) ? MOTION_ENC_SIGN_L : MOTION_ENC_SIGN_R;

    return (int32_t)((uint32_t)(c * sign) + (uint32_t)g_pl.enc_offset);
}

static void run_ms(uint32_t ms, int b_estop)
{
    uint32_t t;
    motion_event_t ev;

    for (t = 0; t < ms; t += 10u)
    {
        g_now += 10u;
        motion_core_step(&g_mc, enc_of(0), enc_of(1), b_estop != 0, g_now);
        while (motion_core_pop_event(&g_mc, &ev))
        {
            g_ev[ev.code]++;
            g_last_seq = ev.seq;
        }
        plant_step(g_mc.duty_l, g_mc.duty_r);
    }
}

static motion_result_t cmd(uint8_t src, uint8_t seq, uint8_t code, int16_t p0, int16_t p1)
{
    motion_cmd_t c = { src, seq, code, 0u, p0, p1 };
    motion_event_t ev;
    motion_result_t r = motion_core_command(&g_mc, &c, g_now);

    while (motion_core_pop_event(&g_mc, &ev))
    {
        g_ev[ev.code]++;
        g_last_seq = ev.seq;
    }
    return r;
}

static void reset_all(double gain)
{
    g_now = 1000u;
    plant_init(gain);
    motion_core_init(&g_mc, g_now);
    memset(g_ev, 0, sizeof(g_ev));
    run_ms(20u, 0);
}

static uint32_t run_until_event(int code, uint32_t max_ms)
{
    uint32_t t = 0u;
    int before = g_ev[code];

    while ((g_ev[code] == before) && (t < max_ms))
    {
        run_ms(10u, 0);
        t += 10u;
    }
    return t;
}

int main(void)
{
    motion_odom_t o;

    /* S2-H01: authority rules (Week 6 Section 6.1) */
    reset_all(0.5);
    CHECK("S2-H01", cmd(MOTION_SRC_LINE, 1, MOTION_CMD_DRIVE, 200, 0) == MOTION_RES_REJECTED_AUTH);
    CHECK("S2-H01", cmd(MOTION_SRC_LINE, 1, MOTION_CMD_SET_AUTH, MOTION_SRC_LINE, 0) == MOTION_RES_REJECTED_AUTH);
    CHECK("S2-H01", cmd(MOTION_SRC_MISSION, 1, MOTION_CMD_SET_AUTH, MOTION_SRC_LINE, 0) == MOTION_RES_ACCEPTED);
    CHECK("S2-H01", cmd(MOTION_SRC_LINE, 2, MOTION_CMD_DRIVE, 200, 0) == MOTION_RES_ACCEPTED);
    CHECK("S2-H01", cmd(MOTION_SRC_AVOID, 1, MOTION_CMD_STOP, 0, 0) == MOTION_RES_REJECTED_AUTH);
    CHECK("S2-H01", cmd(MOTION_SRC_MISSION, 2, MOTION_CMD_MOVE, 100, 200) == MOTION_RES_REJECTED_AUTH);
    CHECK("S2-H01", cmd(MOTION_SRC_MISSION, 3, MOTION_CMD_SET_AUTH, 7, 0) == MOTION_RES_INVALID);
    CHECK("S2-H01", g_mc.rejected == 5u);
    CHECK("S2-H01", cmd(MOTION_SRC_MISSION, 4, MOTION_CMD_STOP, 0, 0) == MOTION_RES_ACCEPTED);

    /* S2-H02: MOVE 1000 mm at 300 mm/s -> DONE, within 2 % (NFR4 logic) and straight */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 10, MOTION_CMD_MOVE, 1000, 300);
    {
        uint32_t t = run_until_event(MOTION_EV_DONE, 20000u);

        motion_core_snapshot(&g_mc, &o);
        CHECK("S2-H02", g_ev[MOTION_EV_DONE] == 1 && g_last_seq == 10u);
        CHECK("S2-H02", abs(o.dist_mm - 1000) <= 20);
        CHECK("S2-H02", abs(o.x_mm - 1000) <= 20 && abs(o.y_mm) <= 30);
        CHECK("S2-H02", t < 2u * motion_core_expected_ms(1000000, 300) + MOTION_TIMEOUT_MARGIN_MS);
        CHECK("S2-H02", o.last_done_seq == 10u && o.done_count == 1u && o.state == MOTION_ST_IDLE_BRAKED);
        printf("MOVE 1000 mm: %d mm in %u ms (x %d, y %d)\n", (int)o.dist_mm, (unsigned)t,
               (int)o.x_mm, (int)o.y_mm);
    }

    /* S2-H03: TURN +90.0 deg and -180.0 deg -> heading within 1 deg of the model */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 20, MOTION_CMD_TURN, 900, 180);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    motion_core_snapshot(&g_mc, &o);
    CHECK("S2-H03", g_ev[MOTION_EV_DONE] == 1 && abs(o.heading_cdeg - 9000) <= 100);
    CHECK("S2-H03", abs(o.dist_mm) <= 5);
    printf("TURN +90: heading %d cdeg\n", (int)o.heading_cdeg);
    (void)cmd(MOTION_SRC_MISSION, 21, MOTION_CMD_TURN, -1800, 180);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    motion_core_snapshot(&g_mc, &o);
    CHECK("S2-H03", g_ev[MOTION_EV_DONE] == 2 && abs(o.heading_cdeg - (-9000)) <= 100);
    printf("TURN -180: heading %d cdeg\n", (int)o.heading_cdeg);

    /* S2-H04: DRIVE watchdog: no new DRIVE for 100 ms -> stops on its own */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 1, MOTION_CMD_SET_AUTH, MOTION_SRC_LINE, 0);
    {
        uint32_t t;

        for (t = 0; t < 1000u; t += 10u)
        {
            (void)cmd(MOTION_SRC_LINE, (uint8_t)t, MOTION_CMD_DRIVE, 250, 0);
            run_ms(10u, 0);
        }
        motion_core_snapshot(&g_mc, &o);
        CHECK("S2-H09", abs(o.v_mm_s - 250) <= 25);      /* NFR5 logic: +-10 % */
        printf("DRIVE 250 mm/s: measured %d mm/s\n", (int)o.v_mm_s);
        CHECK("S2-H04", g_mc.state == MOTION_ST_VELOCITY_TRACK);
        run_ms(80u, 0);                 /* last DRIVE now 90 ms old */
        CHECK("S2-H04", g_mc.state == MOTION_ST_VELOCITY_TRACK);
        run_ms(20u, 0);                 /* 110 ms old */
        CHECK("S2-H04", g_mc.state == MOTION_ST_STOPPING);
        /* S2-H05: ramp at 1 m/s^2 from 250 mm/s: about 250 ms to braked */
        t = 0u;
        while ((g_mc.state != MOTION_ST_IDLE_BRAKED) && (t < 2000u)) { run_ms(10u, 0); t += 10u; }
        CHECK("S2-H05", (t >= 200u) && (t <= 300u));
        CHECK("S2-H05", g_mc.duty_l == 0 && g_mc.duty_r == 0);
        printf("STOP ramp from 250 mm/s: braked after %u ms\n", (unsigned)t);
    }

    /* S2-H06: stall: wheel locked while driven -> STALL after 300 ms */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 30, MOTION_CMD_MOVE, 2000, 300);
    run_ms(200u, 0);
    g_pl.locked[0] = 1;
    {
        uint32_t t = run_until_event(MOTION_EV_STALL, 3000u);

        CHECK("S2-H06", g_ev[MOTION_EV_STALL] == 1);
        CHECK("S2-H06", (t >= 300u) && (t <= 600u));
        printf("STALL reported %u ms after the wheel locked\n", (unsigned)t);
    }

    /* S2-H07: E-stop latch */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 40, MOTION_CMD_MOVE, 1000, 300);
    run_ms(200u, 0);
    run_ms(10u, 1);
    CHECK("S2-H07", g_mc.state == MOTION_ST_ESTOP_LATCHED && g_ev[MOTION_EV_FAILED] == 1);
    CHECK("S2-H07", g_mc.duty_l == 0 && g_mc.duty_r == 0);
    CHECK("S2-H07", cmd(MOTION_SRC_MISSION, 41, MOTION_CMD_STOP, 0, 0) == MOTION_RES_REJECTED_ESTOP);
    CHECK("S2-H07", cmd(MOTION_SRC_LINE, 41, MOTION_CMD_CLEAR_ESTOP, 0, 0) == MOTION_RES_REJECTED_AUTH);
    CHECK("S2-H07", cmd(MOTION_SRC_MISSION, 42, MOTION_CMD_CLEAR_ESTOP, 0, 0) == MOTION_RES_CLEAR_ESTOP);
    motion_core_estop_cleared(&g_mc);
    CHECK("S2-H07", g_mc.state == MOTION_ST_IDLE_BRAKED && g_mc.auth == MOTION_SRC_MISSION);

    /* S2-H08: manoeuvre timeout -> FAILED (weak motor cannot finish) */
    reset_all(0.03);
    (void)cmd(MOTION_SRC_MISSION, 50, MOTION_CMD_MOVE, 500, 300);
    {
        uint32_t limit = 2u * motion_core_expected_ms(500000, 300) + MOTION_TIMEOUT_MARGIN_MS;
        uint32_t t = run_until_event(MOTION_EV_FAILED, 20000u);

        CHECK("S2-H08", g_ev[MOTION_EV_FAILED] == 1 && g_ev[MOTION_EV_DONE] == 0);
        CHECK("S2-H08", (t >= limit - 20u) && (t <= limit + 20u));
        CHECK("S2-H08", g_mc.last_fail_seq == 50u && g_mc.state == MOTION_ST_IDLE_BRAKED);
    }

    /* S2-H10: pose: MOVE 500, TURN +90, MOVE 500 -> (500, 500) */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 60, MOTION_CMD_MOVE, 500, 300);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    (void)cmd(MOTION_SRC_MISSION, 61, MOTION_CMD_TURN, 900, 180);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    (void)cmd(MOTION_SRC_MISSION, 62, MOTION_CMD_MOVE, 500, 300);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    motion_core_snapshot(&g_mc, &o);
    CHECK("S2-H10", abs(o.x_mm - 500) <= 25 && abs(o.y_mm - 500) <= 25);
    printf("square leg: x %d y %d heading %d\n", (int)o.x_mm, (int)o.y_mm, (int)o.heading_cdeg);

    /* S2-H11: encoder counter wrap does not disturb odometry */
    reset_all(0.5);
    g_pl.enc_offset = 2147483647 - 2000;
    motion_core_init(&g_mc, g_now);
    run_ms(20u, 0);
    (void)cmd(MOTION_SRC_MISSION, 70, MOTION_CMD_MOVE, 600, 300);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    motion_core_snapshot(&g_mc, &o);
    CHECK("S2-H11", abs(o.dist_mm - 600) <= 12 && g_ev[MOTION_EV_ENC_FAULT] == 0);

    /* S2-H12: an impossible jump is reported as ENC_FAULT, then cleared */
    g_pl.pos_um[0] += 50000.0;          /* 50 mm in one cycle */
    run_ms(20u, 0);
    CHECK("S2-H12", g_ev[MOTION_EV_ENC_FAULT] == 1);
    run_ms(1100u, 0);
    CHECK("S2-H12", g_ev[MOTION_EV_ENC_OK] == 1);

    /* S2-H13: a new MOVE pre-empts the active one, which is reported FAILED */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 80, MOTION_CMD_MOVE, 1000, 300);
    run_ms(100u, 0);
    (void)cmd(MOTION_SRC_MISSION, 81, MOTION_CMD_MOVE, 100, 300);
    CHECK("S2-H13", g_ev[MOTION_EV_FAILED] == 1 && g_mc.last_fail_seq == 80u);
    (void)run_until_event(MOTION_EV_DONE, 5000u);
    CHECK("S2-H13", g_mc.last_done_seq == 81u);

    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
