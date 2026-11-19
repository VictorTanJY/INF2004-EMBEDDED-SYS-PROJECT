/*
 *  test_s4.c
 *  Host unit tests for subsystem 4: imu_core.c (pitch, humps, tilt,
 *  lift, classification, calibration) and safety_core.c (heartbeats).
 *  Humps are simulated geometrically: the chassis rests on the drive axle
 *  at x and the caster at x + L, so pitch = atan((z(x+L) - z(x)) / L) for
 *  a cosine hump of known height.  This checks the method (A9, NFR7) on a
 *  model; the real error must still be measured on test humps.
 *  Build and run:  make -C testing/subsystem4/host
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_pack.h"
#include "imu_core.h"
#include "imu_cfg.h"
#include "safety_core.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

static imu_core_t g_ic;
static uint32_t   g_now;
static uint8_t    g_ev;

/* Accelerometer reading (mg, raw axes) for chassis pitch theta (deg),
   roll phi (deg) and a forward linear acceleration a_lin (mm/s^2). */
static void accel_for(double theta, double phi, double a_lin, int32_t out[3])
{
    double const time_s = theta * M_PI / 180.0;
    double const pos = phi * M_PI / 180.0;
    double fwd  = 1000.0 * sin(time_s) + a_lin / 9.807;
    double left = 1000.0 * cos(time_s) * sin(pos);
    double up_mg   = 1000.0 * cos(time_s) * cos(pos);
    double body[3] = { 0.0, 0.0, 0.0 };

    body[IMU_AXIS_FWD]  = fwd  * IMU_SIGN_FWD;
    body[IMU_AXIS_LEFT] = left * IMU_SIGN_LEFT;
    body[IMU_AXIS_UP]   = up_mg   * IMU_SIGN_UP;
    out[0] = (int32_t)lround(body[0]);
    out[1] = (int32_t)lround(body[1]);
    out[2] = (int32_t)lround(body[2]);
}

static void step(double theta, double phi, double a_lin, int32_t value, int32_t ds_um, int32_t yaw)
{
    int32_t accel[3];

    accel_for(theta, phi, a_lin, accel);
    g_now += 10u;
    g_ev |= imu_core_step(&g_ic, accel, NULL, ds_um, value, yaw, 10u, g_now);
}

/* Level chassis with an extra up acceleration (mg), e.g. a hand lift. */
static void step_up(int32_t extra_up_mg, int32_t value, int32_t ds_um)
{
    int32_t accel[3];

    accel_for(0.0, 0.0, 0.0, accel);
    accel[IMU_AXIS_UP] += extra_up_mg * IMU_SIGN_UP;
    g_now += 10u;
    g_ev |= imu_core_step(&g_ic, accel, NULL, ds_um, value, 0, 10u, g_now);
}

static double z_of(double x_val, double start, double len, double height)
{
    if ((x_val < start) || (x_val > start + len)) { return 0.0; }
    return 0.5 * height * (1.0 - cos(2.0 * M_PI * (x_val - start) / len));
}

/* Drive 1.5 m at 300 mm/s over a cosine hump; returns the reported peak. */
static int run_hump(double h_mm, double len_mm, int * p_starts, int * p_ends)
{
    double const len = IMU_WHEELBASE_MM;
    double x_val;

    imu_core_init(&g_ic);
    g_now = 0u;
    *p_starts = 0;
    *p_ends = 0;
    for (x_val = 0.0; x_val < 1500.0; x_val += 3.0)
    {
        double theta = atan((z_of(x_val + len, 300.0, len_mm, h_mm) - z_of(x_val, 300.0, len_mm, h_mm)) / len) * 180.0 / M_PI;

        g_ev = 0u;
        step(theta, 0.0, 0.0, 300, 3000, 0);
        if (g_ev & IMU_EV_HUMP_START) { (*p_starts)++; }
        if (g_ev & IMU_EV_HUMP_END) { (*p_ends)++; }
    }
    return (int)g_ic.last_hump_mm;
}

/* Drive 1.2 m at 100 mm/s with one wheel over a cosine hump of height
   h_mm (side = +1 right wheel, -1 left).  The raised wheel rolls the
   chassis by asin(z / track) and tips the nose down by atan(z / 2 / L)
   (drive axle behind the caster).  Returns the reported peak. */
static int run_side_hump(double h_mm, double len_mm, int side, int * p_starts, int * p_ends)
{
    double const period = IMU_SIDE_LEVER_MM;
    double const len = IMU_WHEELBASE_MM;
    double x_val;

    imu_core_init(&g_ic);
    g_now = 0u;
    *p_starts = 0;
    *p_ends = 0;
    for (x_val = 0.0; x_val < 1200.0; x_val += 1.0)
    {
        double z_val    = z_of(x_val, 300.0, len_mm, h_mm);
        double roll = side * asin(z_val / period) * 180.0 / M_PI;
        double theta   = -atan((0.5 * z_val) / len) * 180.0 / M_PI;

        g_ev = 0u;
        step(theta, roll, 0.0, 100, 1000, 0);
        if (g_ev & IMU_EV_HUMP_START) { (*p_starts)++; }
        if (g_ev & IMU_EV_HUMP_END) { (*p_ends)++; }
    }
    return (int)g_ic.last_hump_mm;
}

/* S4-H11: Change Requirement 1, the packed IMU snapshot */
static void test_pack_s4(void)
{
    imu_snap_t snap;
    imu_snap_t ret;
    uint32_t   words[IMU_SNAP_WORDS];

    CHECK("S4-H11", imu_snap_layout_bits() == IMU_SNAP_BITS && PK_WORDS(IMU_SNAP_BITS) == IMU_SNAP_WORDS &&
                    sizeof(words) < sizeof(snap));
    memset(&snap, 0, sizeof(snap));
    snap.t_ms = 99u; snap.pitch_cdeg = -9000; snap.roll_cdeg = 18000; snap.heading_cdeg = 35999; snap.turn_rate_cdeg_s = -32768;
    snap.accel_mg = -16000; snap.hump_max_mm = 31u; snap.last_hump_mm = 1023u; snap.humps = 3u; snap.i2c_errors = 65535u;
    snap.hump_state = 3u; snap.motion = 7u; snap.b_ok = 1u; snap.cal = 3u; snap.pitch_valid = 1u; snap.tilt = 1u;
    imu_snap_pack(&snap, words);
    memset(&ret, 0x5A, sizeof(ret));                /* every field must be written */
    imu_snap_unpack(words, &ret);
    {
        uint32_t words_out[IMU_SNAP_WORDS];

        imu_snap_pack(&ret, words_out);
        CHECK("S4-H11", memcmp(words, words_out, sizeof(words)) == 0);
    }
    memset(&ret, 0, sizeof(ret));                   /* same padding as s */
    imu_snap_unpack(words, &ret);
    CHECK("S4-H11", memcmp(&snap, &ret, sizeof(snap)) == 0);
    snap.humps = 300u; snap.hump_max_mm = 5000u;                                   /* saturate */
    imu_snap_pack(&snap, words);
    imu_snap_unpack(words, &ret);
    CHECK("S4-H11", ret.humps == 255u && ret.hump_max_mm == 1023u);
}

int main(void)
{
    /* S4-H01: humps of known height, 300 mm long: NFR7 (+-20 % or +-5 mm) */
    {
        double const heights[3] = { 15.0, 20.0, 30.0 };
        int idx;

        for (idx = 0; idx < 3; idx++)
        {
            int start_idx, end_idx;
            int peak = run_hump(heights[idx], 300.0, &start_idx, &end_idx);
            double tol = fmax(5.0, 0.2 * heights[idx]);

            printf("hump %.0f mm x 300 mm: reported %d mm\n", heights[idx], peak);
            CHECK("S4-H01", start_idx == 1 && end_idx == 1);
            CHECK("S4-H01", fabs(peak - heights[idx]) <= tol);
        }
        CHECK("S4-H01", g_ic.hump_max_mm == g_ic.last_hump_mm && g_ic.humps == 1u);
    }
    /* S4-H01b: known limit of the Week 6 threshold.  A 10 mm x 300 mm hump
       tilts this chassis (L = IMU_WHEELBASE_MM) to only ~4.8 deg, below the
       +5 deg climb threshold, so it is NOT detected.  If the course has humps
       this low, lower IMU_HUMP_ON_CDEG once A8 has measured the pitch noise. */
    {
        int start_idx, end_idx;
        (void)run_hump(10.0, 300.0, &start_idx, &end_idx);
        CHECK("S4-H01b", start_idx == 0 && end_idx == 0);
        printf("hump 10 mm x 300 mm: not detected (peak pitch below 5 deg) - known limit\n");
    }

    /* S4-H10: humps under ONE wheel (roll): 10 / 20 / 30 mm x 300 mm, both
       sides, reported within NFR7 (+-20 % or +-5 mm) */
    {
        double const heights[3] = { 10.0, 20.0, 30.0 };
        int idx, side;

        for (side = -1; side <= 1; side += 2)
        {
            for (idx = 0; idx < 3; idx++)
            {
                int start_idx, end_idx;
                int peak = run_side_hump(heights[idx], 300.0, side, &start_idx, &end_idx);
                double tol = fmax(5.0, 0.2 * heights[idx]);

                if (side > 0) { printf("one-wheel hump %.0f mm x 300 mm: reported %d mm\n", heights[idx], peak); }
                CHECK("S4-H10", start_idx == 1 && end_idx == 1 && g_ic.humps == 1u);
                CHECK("S4-H10", fabs(peak - heights[idx]) <= tol);
            }
        }
    }
    /* S4-H10b: robot lifted on one side by hand, wheels still: no hump */
    {
        int kdx;
        imu_core_init(&g_ic);
        g_now = 0u;
        g_ev = 0u;
        for (kdx = 0; kdx < 200; kdx++) { step(0.0, 10.0, 0.0, 0, 0, 0); }
        CHECK("S4-H10b", (g_ev & (IMU_EV_HUMP_START | IMU_EV_HUMP_END)) == 0u && g_ic.humps == 0u);
    }
    /* S4-H10c: a short sideways jolt while driving (6 deg for 10 mm): no hump */
    {
        int kdx;
        imu_core_init(&g_ic);
        g_now = 0u;
        g_ev = 0u;
        for (kdx = 0; kdx < 100; kdx++) { step(0.0, 0.0, 0.0, 100, 1000, 0); }
        for (kdx = 0; kdx < 10; kdx++)  { step(0.0, 6.0, 0.0, 100, 1000, 0); }
        for (kdx = 0; kdx < 100; kdx++) { step(0.0, 0.0, 0.0, 100, 1000, 0); }
        CHECK("S4-H10c", (g_ev & (IMU_EV_HUMP_START | IMU_EV_HUMP_END)) == 0u && g_ic.humps == 0u);
    }

    /* S4-H02: hard acceleration and braking on flat ground: no hump */
    {
        int kdx;
        int32_t value = 0;

        imu_core_init(&g_ic);
        g_now = 0u;
        g_ev = 0u;
        for (kdx = 0; kdx < 400; kdx++)
        {
            double val_a = ((kdx / 50) % 2 == 0) ? 1500.0 : -1500.0;  /* +-1.5 m/s^2 */
            value += (int32_t)(val_a / 100.0);
            if (value < 0) { value = 0; val_a = 0.0; }
            step(0.0, 0.0, val_a, value, value * 10, 0);
        }
        CHECK("S4-H02", (g_ev & (IMU_EV_HUMP_START | IMU_EV_HUMP_END)) == 0u);
        CHECK("S4-H02", abs(g_ic.pitch_cdeg) < 300);
    }

    /* S4-H03: a long 8 deg ramp is a ramp, not a hump */
    {
        int kdx;

        imu_core_init(&g_ic);
        g_now = 0u;
        g_ev = 0u;
        for (kdx = 0; kdx < 300; kdx++) { step(8.0, 0.0, 0.0, 300, 3000, 0); }
        CHECK("S4-H03", (g_ev & IMU_EV_RAMP) != 0u && (g_ev & IMU_EV_HUMP_END) == 0u);
    }

    /* S4-H04: tilt on above 30 deg held for IMU_TILT_HOLD_MS, off below
       25 deg; roll too */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    { int kdx; for (kdx = 0; kdx < 10; kdx++) { step(35.0, 0.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H04", (g_ev & IMU_EV_TILT_ON) == 0u && !g_ic.b_tilt);     /* 100 ms: not yet */
    { int kdx; for (kdx = 0; kdx < 20; kdx++) { step(35.0, 0.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H04", (g_ev & IMU_EV_TILT_ON) != 0u && g_ic.b_tilt);
    g_ev = 0u;
    { int kdx; for (kdx = 0; kdx < 10; kdx++) { step(20.0, 0.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H04", (g_ev & IMU_EV_TILT_OFF) != 0u && !g_ic.b_tilt);
    g_ev = 0u;
    { int kdx; for (kdx = 0; kdx < 30; kdx++) { step(0.0, -40.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H04", (g_ev & IMU_EV_TILT_ON) != 0u && g_ic.roll_cdeg < -3000);

    /* S4-H04b: a wheel start on the bench (encoders report 0 -> 400 mm/s
       in 20 ms while the chassis stays level) and a 150 ms 40-deg spike
       must not raise TILT. */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    { int kdx; for (kdx = 0; kdx < 20; kdx++) { step(0.0, 0.0, 0.0, 0, 0, 0); } }
    { int kdx; for (kdx = 0; kdx < 30; kdx++) { step(0.0, 0.0, 0.0, (kdx < 2) ? 200 * (kdx + 1) : 400, 4000, 0); } }
    { int kdx; for (kdx = 0; kdx < 15; kdx++) { step(40.0, 0.0, 0.0, 400, 4000, 0); } }
    { int kdx; for (kdx = 0; kdx < 30; kdx++) { step(0.0, 0.0, 0.0, 400, 4000, 0); } }
    CHECK("S4-H04b", (g_ev & IMU_EV_TILT_ON) == 0u && !g_ic.b_tilt);

    /* S4-H05 (14 Nov): lift.  IMU collision detection is removed; a lift
       is the up axis IMU_LIFT_RISE_MG above level (filtered) for
       IMU_LIFT_HOLD_MS on flat ground. */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    { int kdx; for (kdx = 0; kdx < 20; kdx++) { step_up(0, 200, 2000); } }
    CHECK("S4-H05", (g_ev & IMU_EV_LIFT) == 0u);
    { int kdx; for (kdx = 0; kdx < 15; kdx++) { step_up(400, 0, 0); } }     /* hand lifts it: +0.4 g, 150 ms */
    CHECK("S4-H05", (g_ev & IMU_EV_LIFT) != 0u && g_ic.lift_rise_mg > IMU_LIFT_RISE_MG);
    printf("lift: rise %d mg\n", (int)g_ic.lift_rise_mg);
    /* held still in the air: no second event; set down and lifted again: a new one */
    g_ev = 0u;
    { int kdx; for (kdx = 0; kdx < 30; kdx++) { step_up(0, 0, 0); } }
    CHECK("S4-H05", (g_ev & IMU_EV_LIFT) == 0u && !g_ic.b_lift);
    { int kdx; for (kdx = 0; kdx < 15; kdx++) { step_up(400, 0, 0); } }
    CHECK("S4-H05", (g_ev & IMU_EV_LIFT) != 0u);

    /* S4-H05b: the robot's own jolts do not lift it: one 10 ms sample of
       +1.5 g, two of +1 g, and +0.2 g for 200 ms while driving */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    { int kdx; for (kdx = 0; kdx < 20; kdx++) { step_up(0, 200, 2000); } }
    step_up(1500, 200, 2000);
    { int kdx; for (kdx = 0; kdx < 10; kdx++) { step_up(0, 200, 2000); } }
    step_up(1000, 200, 2000); step_up(1000, 200, 2000);
    { int kdx; for (kdx = 0; kdx < 10; kdx++) { step_up(0, 200, 2000); } }
    { int kdx; for (kdx = 0; kdx < 20; kdx++) { step_up(200, 200, 2000); } }
    { int kdx; for (kdx = 0; kdx < 10; kdx++) { step_up(0, 200, 2000); } }
    CHECK("S4-H05b", (g_ev & IMU_EV_LIFT) == 0u);
    /* ...and the largest rise while driving (filtered) is reported for tuning (LOG 78) */
    CHECK("S4-H05b", (g_ev & IMU_EV_RISE_MAX) != 0u && g_ic.rise_max_mg >= 400);   /* the two +1 g samples */
    printf("largest rise while driving: %d mg\n", (int)g_ic.rise_max_mg);

    /* S4-H05c: on a hump the up push is not a lift */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    { int kdx; for (kdx = 0; kdx < 10; kdx++) { step_up(0, 200, 2000); } }
    g_ic.hump_state = (uint8_t)IMU_HUMP_CLIMBING;
    { int kdx; for (kdx = 0; kdx < 15; kdx++) { step_up(400, 200, 2000); } }
    CHECK("S4-H05c", (g_ev & IMU_EV_LIFT) == 0u);

    /* S4-H05d: the base is the level calibration's up-axis mean (the
       robot read ~890 mg level on 8-9 Nov, not 1000) */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    g_ic.lvl_up_mg = 890;
    { int kdx; for (kdx = 0; kdx < 15; kdx++) { step_up(-110 + 300, 0, 0); } }   /* 1190 mg: +300 over level */
    CHECK("S4-H05d", (g_ev & IMU_EV_LIFT) != 0u);

    /* S4-H05e (14 Nov run 02:44): held in the air, nose up 10 deg with
       the wheels spinning, the hump check read humps of 12-41 mm.  Saved
       when all IR went black, restored when the lift was confirmed: the
       run keeps its real humps only; a real hump while all black briefly
       (a crossing line) is kept. */
    {
        int step0, elem0;
        uint16_t real_max;

        (void)run_hump(20.0, 150.0, &step0, &elem0);              /* one real hump */
        real_max = g_ic.hump_max_mm;
        CHECK("S4-H05e", real_max >= 10u && g_ic.humps == 1u);
        imu_core_hump_save(&g_ic);                           /* all IR black */
        g_ev = 0u;
        { int kdx; for (kdx = 0; kdx < 60; kdx++) { step(10.0, 0.0, 0.0, 300, 3000, 0); } }   /* nose up in the air */
        { int kdx; for (kdx = 0; kdx < 60; kdx++) { step(-10.0, 0.0, 0.0, 300, 3000, 0); } }
        { int kdx; for (kdx = 0; kdx < 30; kdx++) { step(0.0, 0.0, 0.0, 300, 3000, 0); } }
        printf("in the air: %u fake hump(s), max %u mm (real %u)\n", (unsigned)(g_ic.humps - 1u),
               (unsigned)g_ic.hump_max_mm, (unsigned)real_max);
        CHECK("S4-H05e", (g_ev & IMU_EV_HUMP_END) != 0u && g_ic.hump_max_mm > real_max);   /* the bug */
        imu_core_hump_restore(&g_ic);                        /* lift confirmed */
        CHECK("S4-H05e", g_ic.hump_max_mm == real_max && g_ic.humps == 1u &&
                         g_ic.hump_state == (uint8_t)IMU_HUMP_FLAT);

        imu_core_hump_save(&g_ic);                           /* crossing line */
        g_ic.hump_max_mm = (uint16_t)(real_max + 5u);        /* a hump measured meanwhile */
        imu_core_hump_drop(&g_ic);                           /* floor again */
        imu_core_hump_restore(&g_ic);                        /* nothing saved: keeps it */
        CHECK("S4-H05e", g_ic.hump_max_mm == real_max + 5u);
    }

    /* S4-H06: level calibration removes a 3 deg mounting offset */
    imu_core_init(&g_ic);
    g_now = 0u;
    imu_cal_level_start(&g_ic);
    {
        int kdx;
        int32_t accel[3];

        accel_for(3.0, -2.0, 0.0, accel);
        for (kdx = 0; kdx < 100; kdx++) { imu_cal_level_sample(&g_ic, accel); }
        CHECK("S4-H06", imu_cal_level_finish(&g_ic));
        for (kdx = 0; kdx < 20; kdx++) { step(3.0, -2.0, 0.0, 0, 0, 0); }
        CHECK("S4-H06", abs(g_ic.pitch_cdeg) <= 10 && abs(g_ic.roll_cdeg) <= 10);
        /* moving robot: calibration refused */
        imu_cal_level_start(&g_ic);
        for (kdx = 0; kdx < 100; kdx++)
        {
            accel_for((kdx % 2) ? 5.0 : -5.0, 0.0, 0.0, accel);
            imu_cal_level_sample(&g_ic, accel);
        }
        CHECK("S4-H06", !imu_cal_level_finish(&g_ic));
        CHECK("S4-H06", g_ic.lvl_n == 100u && g_ic.lvl_spread_mg > IMU_LEVEL_MAX_SPREAD_MG);   /* LOG 57/58 */
        /* stuck accelerometer (2 Nov: full scale, no noise): refused */
        imu_cal_level_start(&g_ic);
        accel[0] = -3997;  accel[1] = 0;  accel[2] = -3997;
        for (kdx = 0; kdx < 100; kdx++) { imu_cal_level_sample(&g_ic, accel); }
        CHECK("S4-H06", !imu_cal_level_finish(&g_ic));
        CHECK("S4-H06", g_ic.lvl_spread_mg == 0 && g_ic.lvl_up_mg > IMU_LEVEL_UP_MAX_MG);   /* LOG 58/59 */
    }

    /* S4-H07: hard-iron offsets and heading after a 360 deg spin */
    imu_core_init(&g_ic);
    imu_cal_mag_start(&g_ic);
    {
        int diff;
        for (diff = 0; diff < 360; diff += 5)
        {
            double radius = diff * M_PI / 180.0;
            int16_t mag[3] = { 0, 0, 0 };
            /* field of 400 counts; hard-iron offset (+120, -80, +30) */
            mag[IMU_AXIS_FWD]  = (int16_t)((400.0 * cos(radius)) * IMU_SIGN_FWD + 120);
            mag[IMU_AXIS_LEFT] = (int16_t)((400.0 * sin(radius)) * IMU_SIGN_LEFT - 80);
            mag[IMU_AXIS_UP]   = 30;
            imu_cal_mag_sample(&g_ic, mag);
        }
        CHECK("S4-H07", imu_cal_mag_finish(&g_ic));
        {
            int16_t mag[3] = { 0, 0, 0 };
            int32_t accel[3];
            /* facing east: north is to the left */
            mag[IMU_AXIS_FWD]  = (int16_t)(0 * IMU_SIGN_FWD + 120);
            mag[IMU_AXIS_LEFT] = (int16_t)(400 * IMU_SIGN_LEFT - 80);
            mag[IMU_AXIS_UP]   = 30;
            accel_for(0.0, 0.0, 0.0, accel);
            (void)imu_core_step(&g_ic, accel, mag, 0, 0, 0, 10u, 10u);
            CHECK("S4-H07", abs(g_ic.heading_cdeg - 9000) <= 200);
        }
    }

    /* S4-H08: motion classification */
    imu_core_init(&g_ic);
    g_now = 0u;
    { int kdx; for (kdx = 0; kdx < 30; kdx++) { step(0.0, 0.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_STATIONARY);
    { int kdx; for (kdx = 0; kdx < 30; kdx++) { step(0.0, 0.0, 0.0, 100, 1000, 6000); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_TURNING);
    { int kdx; for (kdx = 0; kdx < 30; kdx++) { step(0.0, 0.0, 0.0, 250, 2500, 0); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_CRUISING);
    /* speed up then slow down, forward: 1000 mm/s^2 each way */
    { int kdx; for (kdx = 0; kdx < 20; kdx++) { step(0.0, 0.0, 0.0, 250 + (10 * kdx), 2600, 0); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_ACCELERATING);
    { int kdx; for (kdx = 0; kdx < 20; kdx++) { step(0.0, 0.0, 0.0, 440 - (10 * kdx), 2600, 0); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_DECELERATING);
    /* the same in reverse: speeding up backwards is accelerating (the
       class may change at most every IMU_CLASS_HOLD_MS, so run > 200 ms) */
    imu_core_init(&g_ic);
    { int kdx; for (kdx = 0; kdx < 40; kdx++) { step(0.0, 0.0, 0.0, -20 - (10 * kdx), -1000, 0); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_ACCELERATING);
    { int kdx; for (kdx = 0; kdx < 30; kdx++) { step(0.0, 0.0, 0.0, -410 + (10 * kdx), -1000, 0); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_DECELERATING);

    /* S4-H09: heartbeats: each task times out after its own limit */
    {
        safety_hb_state_t hb_state;
        uint32_t coefs[SAFETY_HB_COUNT] = { 0u, 0u, 0u, 0u, 0u };
        uint32_t elapsed;
        uint32_t mid = 0u;

        memset(&hb_state, 0, sizeof(hb_state));
        CHECK("S4-H09", safety_hb_check(&hb_state, coefs, 1000u) == 0u);     /* starts supervision */
        for (elapsed = 1000u; elapsed <= 1320u; elapsed += 20u)
        {
            coefs[SAFETY_HB_MISSION]++;     /* only Mission keeps running */
            mid = safety_hb_check(&hb_state, coefs, elapsed);
            if (elapsed == 1040u) { CHECK("S4-H09", mid == 0u); }
            if (elapsed == 1060u) { CHECK("S4-H09", mid == ((1u << SAFETY_HB_MOTION) | (1u << SAFETY_HB_LINE))); }
            if (elapsed == 1120u) { CHECK("S4-H09", (mid & (1u << SAFETY_HB_IMU)) != 0u); }
        }
        CHECK("S4-H09", (mid & (1u << SAFETY_HB_MISSION)) == 0u);
        CHECK("S4-H09", (mid & (1u << SAFETY_HB_OBSTACLE)) != 0u);    /* > 300 ms */
        safety_hb_start(&hb_state, coefs, 1320u);
        CHECK("S4-H09", safety_hb_check(&hb_state, coefs, 1360u) == 0u);
        CHECK("S4-H09", safety_hb_timeout_ms(SAFETY_HB_IMU) == 100u);
        /* the 32-bit millisecond wrap does not trigger a timeout */
        safety_hb_start(&hb_state, coefs, 0xFFFFFFF0u);
        CHECK("S4-H09", safety_hb_check(&hb_state, coefs, 0x00000010u) == 0u);
    }

    test_pack_s4();
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
