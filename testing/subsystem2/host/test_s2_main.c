/** @file test_s2_main.c
 *
 *  @brief Host unit tests for motion_core.c (subsystem 2) against a motor
 * model: per wheel, steady speed = 0.5 mm/s per per-mille of duty above a 100
 *  per-mille deadband, first-order lag tau = 50 ms while driven and 20 ms
 *  while braked (both motor inputs low short the windings, so a braked
 *  wheel stops faster than it spins up; on the car the 360 deg
 *  calibration spins of 19 Nov ended at 360-361 deg by the encoders,
 *  which a 50 ms coast overshoots by about 2 deg).  The model is only a
 *  stand-in for the real motors: these tests check the controller LOGIC
 *  (authority, profiles, timeouts, faults), not the real tuning, which
 *  must be measured on the robot (NFR4, NFR5).
 *  Build and run:  make -C testing/subsystem2/host
 */

#include "test_s2_main.h"
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "app_types.h"
#include "app_pack.h"
#include "motion_core.h"
#include "motion_cfg.h"
#include "motion_api.h"

#define T_WHEELS      2 /* left, right */
#define T_MOVES       4 /* entries of move[] */
#define T_DRIVE_MS    1000u
#define T_STOP_MAX_MS 2000u /* ramp to braked well within */

/* NOTE: a macro rather than a function only so that a failure reports the
   caller's file and line (__FILE__, __LINE__). */
#define CHECK(id, cond) (check_result((id), (cond), __FILE__, __LINE__))

typedef struct
{
    float64_t vals[2];   /**< mm/s  */
    float64_t pos_um[2]; /**< wheel distance */
    float64_t gain;      /**< mm/s per per-mille above the deadband */
    int32_t   locked[2];
    int32_t   enc_offset;
} plant_t;

/** The last command passed to the subsystem2_post stand-in. */
typedef struct
{
    int32_t count;
    uint8_t src;
    uint8_t seq;
    uint8_t cmd;
    int16_t arg0;
    int16_t arg1;
} post_rec_t;

static int32_t g_pass;
static int32_t g_fail;

static motion_core_t g_mc;
static plant_t       g_pl;
static uint32_t      g_now;
static int32_t       g_ev[8]; /* counts per motion_core_ev_code_t */
static uint8_t       g_last_seq;

/** Stand-in for subsystem2.c's subsystem2_post (the real one posts to
   mbf_motion): records the last command for the motion_api tests. */
static post_rec_t g_post;

static bool                 posted(uint8_t cmd, int32_t arg0, int32_t arg1);
static void                 plant_init(float64_t gain);
static void                 plant_step(int16_t duty_l, int16_t duty_r);
static int32_t              enc_of(int32_t width);
static void                 run_ms(uint32_t delay_ms, bool b_estop);
static motion_core_result_t cmd(uint8_t src, uint8_t seq, uint8_t code,
                                int16_t arg0, int16_t arg1);
static void                 reset_all(float64_t gain);
static uint32_t             run_until_event(int32_t code, uint32_t max_ms);
static void                 test_pack_s2(void);
static void check_result(char const * p_id, bool b_ok, char const * p_file,
                         int32_t line);
static void test_h01(void);
static void test_h09(void);
static void test_h10(void);
static void test_pack_h16(void);
static void test_pack_h16_part2(void);

motion_api_err_t subsystem2_post (uint8_t src, uint8_t seq, uint8_t cmd,
                                  int16_t arg0, int16_t arg1)
{
    g_post.count++;
    g_post.src  = src;
    g_post.seq  = seq;
    g_post.cmd  = cmd;
    g_post.arg0 = arg0;
    g_post.arg1 = arg1;
    return 0;
}

/** Run subsystem 2's host tests against a simulated drive train.  Exit status
    0 if all passed. */
int main (void)
{
    test_h01();
    test_h09();
    test_h10();
    return (0 == g_fail) ? 0 : 1;
}

/** Checks S2-H01, S2-H02, S2-H03. */
static void test_h01 (void)
{
    motion_types_odom_t odom;

    /* S2-H01: authority rules (Week 6 Section 6.1) */
    reset_all(0.5);
    CHECK("S2-H01", MOTION_RES_REJECTED_AUTH
                        == cmd(MOTION_SRC_LINE, 1, MOTION_CMD_DRIVE, 200, 0));
    CHECK("S2-H01", MOTION_RES_REJECTED_AUTH
                        == cmd(MOTION_SRC_LINE, 1, MOTION_CMD_SET_AUTH,
                               MOTION_SRC_LINE, 0));
    CHECK("S2-H01", MOTION_RES_ACCEPTED
                        == cmd(MOTION_SRC_MISSION, 1, MOTION_CMD_SET_AUTH,
                               MOTION_SRC_LINE, 0));
    CHECK("S2-H01", MOTION_RES_ACCEPTED
                        == cmd(MOTION_SRC_LINE, 2, MOTION_CMD_DRIVE, 200, 0));
    CHECK("S2-H01", MOTION_RES_REJECTED_AUTH
                        == cmd(MOTION_SRC_AVOID, 1, MOTION_CMD_STOP, 0, 0));
    CHECK("S2-H01",
          MOTION_RES_REJECTED_AUTH
              == cmd(MOTION_SRC_MISSION, 2, MOTION_CMD_MOVE, 100, 200));
    CHECK("S2-H01",
          MOTION_RES_INVALID
              == cmd(MOTION_SRC_MISSION, 3, MOTION_CMD_SET_AUTH, 7, 0));
    CHECK("S2-H01", 5u == g_mc.rejected);
    CHECK("S2-H01", MOTION_RES_ACCEPTED
                        == cmd(MOTION_SRC_MISSION, 4, MOTION_CMD_STOP, 0, 0));

    /* S2-H02: MOVE 1000 mm at 300 mm/s -> DONE, within 2 % (NFR4 logic) and
     * straight */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 10, MOTION_CMD_MOVE, 1000, 300);
    {
        uint32_t elapsed = run_until_event(MOTION_EV_DONE, 20000u);

        motion_core_snapshot(&g_mc, &odom);
        CHECK("S2-H02", (1 == g_ev[MOTION_EV_DONE]) && (10u == g_last_seq));
        CHECK("S2-H02", abs(odom.dist_mm - 1000) <= 20);
        CHECK("S2-H02",
              (abs(odom.x_mm - 1000) <= 20) && (abs(odom.y_mm) <= 30));
        CHECK("S2-H02", elapsed < ((2u * motion_core_expected_ms(1000000, 300))
                                   + MOTION_TIMEOUT_MARGIN_MS));
        CHECK("S2-H02", (10u == odom.last_done_seq) && (1u == odom.done_count)
                            && (MOTION_ST_IDLE_BRAKED == odom.state));
        printf("MOVE 1000 mm: %d mm in %u ms (x %d, y %d)\n", odom.dist_mm,
               elapsed, odom.x_mm, odom.y_mm);
    }

    /* S2-H03: TURN +90.0 deg and -180.0 deg -> heading within 1 deg of the
     * model */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 20, MOTION_CMD_TURN, 900, 180);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    motion_core_snapshot(&g_mc, &odom);
    CHECK("S2-H03", (1 == g_ev[MOTION_EV_DONE])
                        && (abs(odom.heading_cdeg - 9000) <= 100));
    CHECK("S2-H03", abs(odom.dist_mm) <= 5);
    printf("TURN +90: heading %d cdeg\n", odom.heading_cdeg);
    (void)cmd(MOTION_SRC_MISSION, 21, MOTION_CMD_TURN, -1800, 180);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    motion_core_snapshot(&g_mc, &odom);
    CHECK("S2-H03", (2 == g_ev[MOTION_EV_DONE])
                        && (abs(odom.heading_cdeg - (-9000)) <= 100));
    printf("TURN -180: heading %d cdeg\n", odom.heading_cdeg);
}

/** Checks S2-H09, S2-H04, S2-H05, S2-H06, S2-H07, S2-H08. */
static void test_h09 (void)
{
    motion_types_odom_t odom;

    /* S2-H04: DRIVE watchdog: no new DRIVE for 100 ms -> stops on its own */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 1, MOTION_CMD_SET_AUTH, MOTION_SRC_LINE, 0);
    {
        uint32_t elapsed;

        for (elapsed = 0; elapsed < T_DRIVE_MS; elapsed += 10u)
        {
            /* Cast (uint8_t)elapsed: an 8-bit sequence number: the low 8 bits,
               wrapping on purpose. */
            (void)cmd(MOTION_SRC_LINE, (uint8_t)elapsed, MOTION_CMD_DRIVE, 250,
                      0);
            run_ms(10u, false);
        }

        /* Cast (int32_t)odom.v_mm_s: every int16_t value fits. */
        motion_core_snapshot(&g_mc, &odom);
        CHECK("S2-H09", abs(odom.v_mm_s - 250) <= 25); /* NFR5 logic: +-10 % */
        printf("DRIVE 250 mm/s: measured %d mm/s\n", (int32_t)odom.v_mm_s);
        CHECK("S2-H04", MOTION_ST_VELOCITY_TRACK == g_mc.state);
        run_ms(80u, false); /* last DRIVE now 90 ms old */
        CHECK("S2-H04", MOTION_ST_VELOCITY_TRACK == g_mc.state);
        run_ms(20u, false); /* 110 ms old */
        CHECK("S2-H04", MOTION_ST_STOPPING == g_mc.state);
        /* S2-H05: ramp at 1 m/s^2 from 250 mm/s: about 250 ms to braked */
        elapsed = 0u;

        while ((g_mc.state != MOTION_ST_IDLE_BRAKED)
               && (elapsed < T_STOP_MAX_MS))
        {
            run_ms(10u, false);
            elapsed += 10u;
        }

        CHECK("S2-H05", (elapsed >= 200u) && (elapsed <= 300u));
        CHECK("S2-H05", (0 == g_mc.duty_l) && (0 == g_mc.duty_r));
        printf("STOP ramp from 250 mm/s: braked after %u ms\n", elapsed);
    }

    /* S2-H06: stall: wheel locked while driven -> STALL after 300 ms */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 30, MOTION_CMD_MOVE, 2000, 300);
    run_ms(200u, false);
    g_pl.locked[0] = 1;
    {
        uint32_t elapsed = run_until_event(MOTION_EV_STALL, 3000u);

        CHECK("S2-H06", 1 == g_ev[MOTION_EV_STALL]);
        CHECK("S2-H06", (elapsed >= 300u) && (elapsed <= 600u));
        printf("STALL reported %u ms after the wheel locked\n", elapsed);
    }

    /* S2-H07: E-stop latch */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 40, MOTION_CMD_MOVE, 1000, 300);
    run_ms(200u, false);
    run_ms(10u, true);
    CHECK("S2-H07", (MOTION_ST_ESTOP_LATCHED == g_mc.state)
                        && (1 == g_ev[MOTION_EV_FAILED]));
    CHECK("S2-H07", (0 == g_mc.duty_l) && (0 == g_mc.duty_r));
    CHECK("S2-H07", MOTION_RES_REJECTED_ESTOP
                        == cmd(MOTION_SRC_MISSION, 41, MOTION_CMD_STOP, 0, 0));
    CHECK("S2-H07",
          MOTION_RES_REJECTED_AUTH
              == cmd(MOTION_SRC_LINE, 41, MOTION_CMD_CLEAR_ESTOP, 0, 0));
    CHECK("S2-H07",
          MOTION_RES_CLEAR_ESTOP
              == cmd(MOTION_SRC_MISSION, 42, MOTION_CMD_CLEAR_ESTOP, 0, 0));
    motion_core_estop_cleared(&g_mc);
    CHECK("S2-H07", (MOTION_ST_IDLE_BRAKED == g_mc.state)
                        && (MOTION_SRC_MISSION == g_mc.auth));

    /* S2-H08: manoeuvre timeout -> FAILED (weak motor cannot finish) */
    reset_all(0.03);
    (void)cmd(MOTION_SRC_MISSION, 50, MOTION_CMD_MOVE, 500, 300);
    {
        uint32_t limit   = (2u * motion_core_expected_ms(500000, 300))
                           + MOTION_TIMEOUT_MARGIN_MS;
        uint32_t elapsed = run_until_event(MOTION_EV_FAILED, 20000u);

        CHECK("S2-H08",
              (1 == g_ev[MOTION_EV_FAILED]) && (0 == g_ev[MOTION_EV_DONE]));
        CHECK("S2-H08",
              (elapsed >= (limit - 20u)) && (elapsed <= (limit + 20u)));
        CHECK("S2-H08", (50u == g_mc.last_fail_seq)
                            && (MOTION_ST_IDLE_BRAKED == g_mc.state));
    }
}

/** Checks S2-H10, S2-H11, S2-H12, S2-H13, S2-H14. */
static void test_h10 (void)
{
    motion_types_odom_t odom;

    /* S2-H10: pose: MOVE 500, TURN +90, MOVE 500 -> (500, 500) */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 60, MOTION_CMD_MOVE, 500, 300);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    (void)cmd(MOTION_SRC_MISSION, 61, MOTION_CMD_TURN, 900, 180);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    (void)cmd(MOTION_SRC_MISSION, 62, MOTION_CMD_MOVE, 500, 300);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    motion_core_snapshot(&g_mc, &odom);
    CHECK("S2-H10",
          (abs(odom.x_mm - 500) <= 25) && (abs(odom.y_mm - 500) <= 25));
    printf("square leg: x %d y %d heading %d\n", odom.x_mm, odom.y_mm,
           odom.heading_cdeg);

    /* S2-H11: encoder counter wrap does not disturb odometry */
    reset_all(0.5);
    g_pl.enc_offset = 2147483647 - 2000;
    motion_core_init(&g_mc, g_now);
    run_ms(20u, false);
    (void)cmd(MOTION_SRC_MISSION, 70, MOTION_CMD_MOVE, 600, 300);
    (void)run_until_event(MOTION_EV_DONE, 10000u);
    motion_core_snapshot(&g_mc, &odom);
    CHECK("S2-H11",
          (abs(odom.dist_mm - 600) <= 12) && (0 == g_ev[MOTION_EV_ENC_FAULT]));

    /* S2-H12: an impossible jump is reported as ENC_FAULT, then cleared */
    g_pl.pos_um[0] += 50000.0; /* 50 mm in one cycle */
    run_ms(20u, false);
    CHECK("S2-H12", 1 == g_ev[MOTION_EV_ENC_FAULT]);
    run_ms(1100u, false);
    CHECK("S2-H12", 1 == g_ev[MOTION_EV_ENC_OK]);

    /* S2-H13: a new MOVE pre-empts the active one, which is reported FAILED */
    reset_all(0.5);
    (void)cmd(MOTION_SRC_MISSION, 80, MOTION_CMD_MOVE, 1000, 300);
    run_ms(100u, false);
    (void)cmd(MOTION_SRC_MISSION, 81, MOTION_CMD_MOVE, 100, 300);
    CHECK("S2-H13",
          (1 == g_ev[MOTION_EV_FAILED]) && (80u == g_mc.last_fail_seq));
    (void)run_until_event(MOTION_EV_DONE, 5000u);
    CHECK("S2-H13", 81u == g_mc.last_done_seq);

    /* S2-H14: the named API (FR5) posts the right command and signs */
    memset(&g_post, 0, sizeof(g_post));
    CHECK("S2-H14",
          (0 == motion_api_move_forward(MOTION_SRC_MISSION, 7u, 500u, 250u))
              && posted(MOTION_CMD_MOVE, 500, 250)
              && (MOTION_SRC_MISSION == g_post.src) && (7u == g_post.seq));
    CHECK("S2-H14",
          (0 == motion_api_move_backward(MOTION_SRC_AVOID, 8u, 100u, 250u))
              && posted(MOTION_CMD_MOVE, -100, 250)
              && (MOTION_SRC_AVOID == g_post.src));
    CHECK("S2-H14",
          (0 == motion_api_turn_left(MOTION_SRC_MISSION, 9u, 900u, 120u))
              && posted(MOTION_CMD_TURN, 900, 120));
    CHECK("S2-H14",
          (0 == motion_api_turn_right(MOTION_SRC_MISSION, 10u, 1800u, 90u))
              && posted(MOTION_CMD_TURN, -1800, 90));
    CHECK("S2-H14", (0 == motion_api_stop(MOTION_SRC_MISSION, 11u))
                        && posted(MOTION_CMD_STOP, 0, 0));
    CHECK("S2-H14", (0 == motion_api_drive(MOTION_SRC_LINE, 12u, 250, -400))
                        && posted(MOTION_CMD_DRIVE, 250, -400)
                        && (MOTION_SRC_LINE == g_post.src));
    /* out of range: refused, nothing posted */
    {
        int32_t const count = g_post.count;

        CHECK("S2-H14", MOTION_API_E_PAR
                            == motion_api_move_forward(MOTION_SRC_MISSION, 1u,
                                                       40000u, 250u));
        CHECK("S2-H14",
              MOTION_API_E_PAR
                  == motion_api_turn_left(MOTION_SRC_MISSION, 1u, 900u, 0u));
        CHECK("S2-H14", g_post.count == count);
    }
    /* end to end: what the API posts, MotionTask executes (turn left 90) */
    reset_all(0.5);
    (void)motion_api_turn_left(MOTION_SRC_MISSION, 90u, 900u, 120u);
    (void)cmd(g_post.src, g_post.seq, g_post.cmd, g_post.arg0, g_post.arg1);
    (void)run_until_event(MOTION_EV_DONE, 5000u);
    CHECK("S2-H14", 90u == g_mc.last_done_seq);

    test_pack_s2();
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
}

/** True when the last posted command was cmd(arg0, arg1). */
static bool posted (uint8_t cmd, int32_t arg0, int32_t arg1)
{
    return (g_post.cmd == cmd) && (g_post.arg0 == arg0)
           && (g_post.arg1 == arg1);
}

/** Reset the simulated motors and wheels; gain = mm/s per per-mille of duty
    above the deadband. */
static void plant_init (float64_t gain)
{
    memset(&g_pl, 0, sizeof(g_pl));
    g_pl.gain = gain;
}

/** Advance the simulated wheels by 10 ms at these duties. */
static void plant_step (int16_t duty_l, int16_t duty_r)
{
    int16_t const duty[2] = {duty_l, duty_r};
    int32_t       width;

    for (width = 0; width < T_WHEELS; width++)
    {
        /* Cast (float64_t)duty[width]: exact: a 16-bit integer fits a
           double. */
        float64_t diff   = (float64_t)duty[width];
        float64_t sum_sq = 0.0;

        if (diff > 100.0)
        {
            sum_sq = (diff - 100.0) * g_pl.gain;
        }
        else if (diff < -100.0)
        {
            sum_sq = (diff + 100.0) * g_pl.gain;
        }
        else
        {
            /* No action required. */
        }

        if ((0 == duty[width]) || g_pl.locked[width])
        {
            sum_sq = 0.0;
        } /* brake */
        g_pl.vals[width] += (sum_sq - g_pl.vals[width])

                            * (10.0 / ((0 == duty[width]) ? 20.0 : 50.0));
        if (g_pl.locked[width])
        {
            g_pl.vals[width] = 0.0;
        }

        g_pl.pos_um[width] += g_pl.vals[width] * 10.0; /* mm/s * 10 ms = um */
    }
}

/** Encoder count of simulated wheel width (0 left, 1 right), with its sign and
    offset. */
static int32_t enc_of (int32_t width)
{
    /* Cast (int32_t)((counts >= 0) ? (counts ...)): 12 counts per mm and moves
       of at most 4095 mm: about 50000 counts. */
    float64_t counts =
        (g_pl.pos_um[width] * MOTION_COUNTS_PER_REV) / MOTION_WHEEL_CIRC_UM;
    int32_t cnt  = (int32_t)((counts >= 0) ? (counts + 0.5) : (counts - 0.5));
    int32_t sign = (0 == width) ? MOTION_ENC_SIGN_L : MOTION_ENC_SIGN_R;

    /* Casts (int32_t)((uint32_t)(cnt * sign) + ...): wrapping 32-bit counter,
       as the PIO counter does; (uint32_t)(cnt * sign),
       (uint32_t)g_pl.enc_offset: bit pattern for the wrapping sum. */
    return (int32_t)((uint32_t)(cnt * sign) + (uint32_t)g_pl.enc_offset);
}

/** Run the controller and the simulated wheels for delay_ms, counting the
    events. */
static void run_ms (uint32_t delay_ms, bool b_estop)
{
    uint32_t            elapsed;
    motion_core_event_t event;

    for (elapsed = 0; elapsed < delay_ms; elapsed += 10u)
    {
        g_now += 10u;
        motion_core_step(&g_mc, enc_of(0), enc_of(1), b_estop, g_now);

        while (motion_core_pop_event(&g_mc, &event))
        {
            g_ev[event.code]++;
            g_last_seq = event.seq;
        }

        plant_step(g_mc.duty_l, g_mc.duty_r);
    }
}

/** Send a command to the controller now and count the events it gives.
    Returns its result. */
static motion_core_result_t cmd (uint8_t src, uint8_t seq, uint8_t code,
                                 int16_t arg0, int16_t arg1)
{
    motion_types_cmd_t   cmd_msg = {src, seq, code, 0u, arg0, arg1};
    motion_core_event_t  event;
    motion_core_result_t result = motion_core_command(&g_mc, &cmd_msg, g_now);

    while (motion_core_pop_event(&g_mc, &event))
    {
        g_ev[event.code]++;
        g_last_seq = event.seq;
    }

    return result;
}

/** Fresh controller and simulated wheels with this gain, settled for 20 ms. */
static void reset_all (float64_t gain)
{
    g_now = 1000u;
    plant_init(gain);
    motion_core_init(&g_mc, g_now);
    memset(g_ev, 0, sizeof(g_ev));
    run_ms(20u, false);
}

/** Run until another event of this code arrives or max_ms pass.  Returns the
    time taken. */
static uint32_t run_until_event (int32_t code, uint32_t max_ms)
{
    uint32_t elapsed = 0u;
    int32_t  before  = g_ev[code];

    while ((g_ev[code] == before) && (elapsed < max_ms))
    {
        run_ms(10u, false);
        elapsed += 10u;
    }

    return elapsed;
}

/** S2-H16: Change Requirement 1, the packed mbf_motion word and odometry */
static void test_pack_s2 (void)
{
    test_pack_h16();
    test_pack_h16_part2();
}

/** Checks S2-H16. */
static void test_pack_h16 (void)
{
    motion_types_cmd_t   cmd;
    motion_types_cmd_t   unpacked;
    bool                 b_ok = true;
    int32_t              kdx;
    static int16_t const drive[][2] = {
        {0, 0}, {450, -2000}, {-511, 2047}, {300, 1500}};
    static int16_t const move[][2] = {
        {1000, 200}, {-4095, 511}, {4095, 1}, {0, 90}};

    memset(&cmd, 0, sizeof(cmd));

    for (kdx = 0; kdx < T_MOVES; kdx++)
    {
        /* Cast (uint8_t)(kdx * 9): kdx < T_MOVES (4). */
        cmd.src  = 1u;
        cmd.seq  = (uint8_t)(kdx * 9);
        cmd.cmd  = MOTION_CMD_DRIVE;
        cmd.arg0 = drive[kdx][0];
        cmd.arg1 = drive[kdx][1];
        motion_core_cmd_unpack(motion_core_cmd_pack(&cmd), &unpacked);

        if ((unpacked.src != cmd.src) || (unpacked.seq != cmd.seq)
            || (unpacked.cmd != cmd.cmd) || (unpacked.arg0 != cmd.arg0)
            || (unpacked.arg1 != cmd.arg1))
        {
            b_ok = false;
        }

        cmd.src  = 0u;
        cmd.cmd  = (0 != (kdx % 2)) ? MOTION_CMD_TURN : MOTION_CMD_MOVE;
        cmd.arg0 = move[kdx][0];
        cmd.arg1 = move[kdx][1];
        motion_core_cmd_unpack(motion_core_cmd_pack(&cmd), &unpacked);

        if ((unpacked.cmd != cmd.cmd) || (unpacked.arg0 != cmd.arg0)
            || (unpacked.arg1 != cmd.arg1))
        {
            b_ok = false;
        }
    }

    CHECK("S2-H16", b_ok);
    cmd.cmd  = MOTION_CMD_SET_AUTH;
    cmd.arg0 = 2;
    cmd.arg1 = 0;
    cmd.seq  = 31u;
    cmd.src  = 0u;
    motion_core_cmd_unpack(motion_core_cmd_pack(&cmd), &unpacked);
    CHECK("S2-H16", (MOTION_CMD_SET_AUTH == unpacked.cmd)
                        && (2 == unpacked.arg0) && (31u == unpacked.seq));
    cmd.cmd  = MOTION_CMD_DRIVE;
    cmd.arg0 = 600;
    cmd.arg1 = -6000;
    cmd.seq  = 33u; /* saturate, seq 5 bits */
    motion_core_cmd_unpack(motion_core_cmd_pack(&cmd), &unpacked);
    CHECK("S2-H16", (511 == unpacked.arg0) && (-2048 == unpacked.arg1)
                        && (1u == unpacked.seq));
    CHECK("S2-H16", (0u == motion_types_seq_next(31u))
                        && (6u == motion_types_seq_next(5u)));
    /* the named API refuses what the word cannot carry */
    CHECK(
        "S2-H16",
        (MOTION_API_E_PAR == motion_api_move_forward(0u, 1u, 4096u, 200u))
            && (MOTION_API_E_PAR == motion_api_turn_left(0u, 1u, 900u, 512u)));
}

/** Checks S2-H16. */
static void test_pack_h16_part2 (void)
{
    motion_types_odom_t odom;
    motion_types_odom_t ret;
    uint32_t            words[MOTION_ODOM_WORDS];

    /* odometry: the layout fills MOTION_ODOM_WORDS; values in range come back
     * exactly */
    CHECK("S2-H16", (MOTION_ODOM_BITS == motion_core_odom_layout_bits())
                        && (MOTION_ODOM_WORDS == PK_WORDS(MOTION_ODOM_BITS))
                        && (sizeof(words) < sizeof(odom)));
    memset(&odom, 0, sizeof(odom));
    odom.t_ms            = 0xDEADBEEFu;
    odom.enc_l           = -123456789;
    odom.enc_r           = 2000000000;
    odom.dist_mm         = -42;
    odom.x_mm            = -8388608;
    odom.y_mm            = 8388607;
    odom.heading_cdeg    = -720000;
    odom.v_l_mm_s        = -511;
    odom.v_r_mm_s        = 450;
    odom.v_mm_s          = -30;
    odom.yaw_rate_cdeg_s = -32768;
    odom.duty_l          = -1000;
    odom.duty_r          = 1000;
    odom.rejected        = 65535u;
    odom.state           = 4u;
    odom.phase           = 4u;
    odom.auth            = 2u;
    odom.last_done_seq   = 31u;
    odom.last_done_src   = 2u;
    odom.last_fail_seq   = 17u;
    odom.done_count      = 255u;
    odom.fail_count      = 1u;
    odom.hw_ok           = 1u;
    motion_core_odom_pack(&odom, words);
    memset(&ret, 0x5A, sizeof(ret)); /* every field must be written */
    motion_core_odom_unpack(words, &ret);
    {
        uint32_t words_out[MOTION_ODOM_WORDS];

        motion_core_odom_pack(&ret, words_out);
        CHECK("S2-H16", 0 == memcmp(words, words_out, sizeof(words)));
    }
    memset(&ret, 0, sizeof(ret)); /* same padding as o */
    motion_core_odom_unpack(words, &ret);
    CHECK("S2-H16", 0 == memcmp(&odom, &ret, sizeof(odom)));
    odom.v_l_mm_s = 32767;
    odom.duty_r   = -32768;
    odom.x_mm     = 100000000; /* out of range: saturate */
    motion_core_odom_pack(&odom, words);
    motion_core_odom_unpack(words, &ret);
    CHECK("S2-H16", (511 == ret.v_l_mm_s) && (-1024 == ret.duty_r)
                        && (8388607 == ret.x_mm));
}

/*!
 * @brief Count one check; print a FAIL line when it failed.
 * @param[in] p_id Test id, as in the subsystem's TEST_LOG.md.
 * @param[in] b_ok Whether the check passed.
 * @param[in] p_file Source file of the check.
 * @param[in] line Source line of the check.
 */
static void check_result (char const * p_id, bool b_ok, char const * p_file,
                          int32_t line)
{
    if (b_ok)
    {
        g_pass++;
    }
    else
    {
        g_fail++;
        printf("FAIL %s  (%s:%" PRId32 ")\n", p_id, p_file, line);
    }
}

/*** end of file ***/
