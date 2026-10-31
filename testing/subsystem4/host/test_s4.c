/*
 *  test_s4.c
 *  Host unit tests for subsystem 4: imu_core.c (pitch, humps, tilt,
 *  impact, classification, calibration) and safety_core.c (heartbeats).
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
    double const t = theta * M_PI / 180.0;
    double const p = phi * M_PI / 180.0;
    double fwd  = 1000.0 * sin(t) + a_lin / 9.807;
    double left = 1000.0 * cos(t) * sin(p);
    double up   = 1000.0 * cos(t) * cos(p);
    double body[3] = { 0.0, 0.0, 0.0 };

    body[IMU_AXIS_FWD]  = fwd  * IMU_SIGN_FWD;
    body[IMU_AXIS_LEFT] = left * IMU_SIGN_LEFT;
    body[IMU_AXIS_UP]   = up   * IMU_SIGN_UP;
    out[0] = (int32_t)lround(body[0]);
    out[1] = (int32_t)lround(body[1]);
    out[2] = (int32_t)lround(body[2]);
}

static void step(double theta, double phi, double a_lin, int32_t v, int32_t ds_um, int32_t yaw)
{
    int32_t a[3];

    accel_for(theta, phi, a_lin, a);
    g_now += 10u;
    g_ev |= imu_core_step(&g_ic, a, NULL, ds_um, v, yaw, 10u, g_now);
}

static double z_of(double x, double start, double len, double h)
{
    if ((x < start) || (x > start + len)) { return 0.0; }
    return 0.5 * h * (1.0 - cos(2.0 * M_PI * (x - start) / len));
}

/* Drive 1.5 m at 300 mm/s over a cosine hump; returns the reported peak. */
static int run_hump(double h_mm, double len_mm, int * p_starts, int * p_ends)
{
    double const L = IMU_WHEELBASE_MM;
    double x;

    imu_core_init(&g_ic);
    g_now = 0u;
    *p_starts = 0;
    *p_ends = 0;
    for (x = 0.0; x < 1500.0; x += 3.0)
    {
        double th = atan((z_of(x + L, 300.0, len_mm, h_mm) - z_of(x, 300.0, len_mm, h_mm)) / L) * 180.0 / M_PI;

        g_ev = 0u;
        step(th, 0.0, 0.0, 300, 3000, 0);
        if (g_ev & IMU_EV_HUMP_START) { (*p_starts)++; }
        if (g_ev & IMU_EV_HUMP_END) { (*p_ends)++; }
    }
    return (int)g_ic.last_hump_mm;
}

int main(void)
{
    /* S4-H01: humps of known height, 300 mm long: NFR7 (+-20 % or +-5 mm) */
    {
        double const heights[3] = { 15.0, 20.0, 30.0 };
        int i;

        for (i = 0; i < 3; i++)
        {
            int s, e;
            int peak = run_hump(heights[i], 300.0, &s, &e);
            double tol = fmax(5.0, 0.2 * heights[i]);

            printf("hump %.0f mm x 300 mm: reported %d mm\n", heights[i], peak);
            CHECK("S4-H01", s == 1 && e == 1);
            CHECK("S4-H01", fabs(peak - heights[i]) <= tol);
        }
        CHECK("S4-H01", g_ic.hump_max_mm == g_ic.last_hump_mm && g_ic.humps == 1u);
    }
    /* S4-H01b: known limit of the Week 6 threshold.  A 10 mm x 300 mm hump
       tilts this chassis (L = IMU_WHEELBASE_MM) to only ~4.8 deg, below the
       +5 deg climb threshold, so it is NOT detected.  If the course has humps
       this low, lower IMU_HUMP_ON_CDEG once A8 has measured the pitch noise. */
    {
        int s, e;
        (void)run_hump(10.0, 300.0, &s, &e);
        CHECK("S4-H01b", s == 0 && e == 0);
        printf("hump 10 mm x 300 mm: not detected (peak pitch below 5 deg) - known limit\n");
    }

    /* S4-H02: hard acceleration and braking on flat ground: no hump */
    {
        int k;
        int32_t v = 0;

        imu_core_init(&g_ic);
        g_now = 0u;
        g_ev = 0u;
        for (k = 0; k < 400; k++)
        {
            double a = ((k / 50) % 2 == 0) ? 1500.0 : -1500.0;  /* +-1.5 m/s^2 */
            v += (int32_t)(a / 100.0);
            if (v < 0) { v = 0; a = 0.0; }
            step(0.0, 0.0, a, v, v * 10, 0);
        }
        CHECK("S4-H02", (g_ev & (IMU_EV_HUMP_START | IMU_EV_HUMP_END)) == 0u);
        CHECK("S4-H02", abs(g_ic.pitch_cdeg) < 300);
    }

    /* S4-H03: a long 8 deg ramp is a ramp, not a hump */
    {
        int k;

        imu_core_init(&g_ic);
        g_now = 0u;
        g_ev = 0u;
        for (k = 0; k < 300; k++) { step(8.0, 0.0, 0.0, 300, 3000, 0); }
        CHECK("S4-H03", (g_ev & IMU_EV_RAMP) != 0u && (g_ev & IMU_EV_HUMP_END) == 0u);
    }

    /* S4-H04: tilt on above 30 deg held for IMU_TILT_HOLD_MS, off below
       25 deg; roll too */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    { int k; for (k = 0; k < 10; k++) { step(35.0, 0.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H04", (g_ev & IMU_EV_TILT_ON) == 0u && !g_ic.b_tilt);     /* 100 ms: not yet */
    { int k; for (k = 0; k < 20; k++) { step(35.0, 0.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H04", (g_ev & IMU_EV_TILT_ON) != 0u && g_ic.b_tilt);
    g_ev = 0u;
    { int k; for (k = 0; k < 10; k++) { step(20.0, 0.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H04", (g_ev & IMU_EV_TILT_OFF) != 0u && !g_ic.b_tilt);
    g_ev = 0u;
    { int k; for (k = 0; k < 30; k++) { step(0.0, -40.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H04", (g_ev & IMU_EV_TILT_ON) != 0u && g_ic.roll_cdeg < -3000);

    /* S4-H04b: a wheel start on the bench (encoders report 0 -> 400 mm/s
       in 20 ms while the chassis stays level) and a 150 ms 40-deg spike
       must not raise TILT. */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    { int k; for (k = 0; k < 20; k++) { step(0.0, 0.0, 0.0, 0, 0, 0); } }
    { int k; for (k = 0; k < 30; k++) { step(0.0, 0.0, 0.0, (k < 2) ? 200 * (k + 1) : 400, 4000, 0); } }
    { int k; for (k = 0; k < 15; k++) { step(40.0, 0.0, 0.0, 400, 4000, 0); } }
    { int k; for (k = 0; k < 30; k++) { step(0.0, 0.0, 0.0, 400, 4000, 0); } }
    CHECK("S4-H04b", (g_ev & IMU_EV_TILT_ON) == 0u && !g_ic.b_tilt);

    /* S4-H05: impact = a jump of more than 1.5 g between samples */
    imu_core_init(&g_ic);
    g_now = 0u;
    g_ev = 0u;
    { int k; for (k = 0; k < 10; k++) { step(0.0, 0.0, 0.0, 200, 2000, 0); } }
    CHECK("S4-H05", (g_ev & IMU_EV_IMPACT) == 0u);
    {
        int32_t a[3] = { 0, 0, 0 };
        a[IMU_AXIS_FWD] = -2600 * IMU_SIGN_FWD;     /* front collision */
        a[IMU_AXIS_UP]  = 1000 * IMU_SIGN_UP;
        g_now += 10u;
        g_ev = imu_core_step(&g_ic, a, NULL, 2000, 200, 0, 10u, g_now);
    }
    CHECK("S4-H05", (g_ev & IMU_EV_IMPACT) != 0u && g_ic.motion == IMU_MOTION_IMPACT);

    /* S4-H06: level calibration removes a 3 deg mounting offset */
    imu_core_init(&g_ic);
    g_now = 0u;
    imu_cal_level_start(&g_ic);
    {
        int k;
        int32_t a[3];

        accel_for(3.0, -2.0, 0.0, a);
        for (k = 0; k < 100; k++) { imu_cal_level_sample(&g_ic, a); }
        CHECK("S4-H06", imu_cal_level_finish(&g_ic));
        for (k = 0; k < 20; k++) { step(3.0, -2.0, 0.0, 0, 0, 0); }
        CHECK("S4-H06", abs(g_ic.pitch_cdeg) <= 10 && abs(g_ic.roll_cdeg) <= 10);
        /* moving robot: calibration refused */
        imu_cal_level_start(&g_ic);
        for (k = 0; k < 100; k++)
        {
            accel_for((k % 2) ? 5.0 : -5.0, 0.0, 0.0, a);
            imu_cal_level_sample(&g_ic, a);
        }
        CHECK("S4-H06", !imu_cal_level_finish(&g_ic));
    }

    /* S4-H07: hard-iron offsets and heading after a 360 deg spin */
    imu_core_init(&g_ic);
    imu_cal_mag_start(&g_ic);
    {
        int d;
        for (d = 0; d < 360; d += 5)
        {
            double r = d * M_PI / 180.0;
            int16_t m[3] = { 0, 0, 0 };
            /* field of 400 counts; hard-iron offset (+120, -80, +30) */
            m[IMU_AXIS_FWD]  = (int16_t)((400.0 * cos(r)) * IMU_SIGN_FWD + 120);
            m[IMU_AXIS_LEFT] = (int16_t)((400.0 * sin(r)) * IMU_SIGN_LEFT - 80);
            m[IMU_AXIS_UP]   = 30;
            imu_cal_mag_sample(&g_ic, m);
        }
        CHECK("S4-H07", imu_cal_mag_finish(&g_ic));
        {
            int16_t m[3] = { 0, 0, 0 };
            int32_t a[3];
            /* facing east: north is to the left */
            m[IMU_AXIS_FWD]  = (int16_t)(0 * IMU_SIGN_FWD + 120);
            m[IMU_AXIS_LEFT] = (int16_t)(400 * IMU_SIGN_LEFT - 80);
            m[IMU_AXIS_UP]   = 30;
            accel_for(0.0, 0.0, 0.0, a);
            (void)imu_core_step(&g_ic, a, m, 0, 0, 0, 10u, 10u);
            CHECK("S4-H07", abs(g_ic.heading_cdeg - 9000) <= 200);
        }
    }

    /* S4-H08: motion classification */
    imu_core_init(&g_ic);
    g_now = 0u;
    { int k; for (k = 0; k < 30; k++) { step(0.0, 0.0, 0.0, 0, 0, 0); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_STATIONARY);
    { int k; for (k = 0; k < 30; k++) { step(0.0, 0.0, 0.0, 100, 1000, 6000); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_TURNING);
    { int k; for (k = 0; k < 30; k++) { step(0.0, 0.0, 0.0, 250, 2500, 0); } }
    CHECK("S4-H08", g_ic.motion == IMU_MOTION_CRUISING);

    /* S4-H09: heartbeats: each task times out after its own limit */
    {
        safety_hb_state_t st;
        uint32_t c[SAFETY_HB_COUNT] = { 0u, 0u, 0u, 0u, 0u };
        uint32_t t;
        uint32_t m = 0u;

        memset(&st, 0, sizeof(st));
        CHECK("S4-H09", safety_hb_check(&st, c, 1000u) == 0u);     /* starts supervision */
        for (t = 1000u; t <= 1320u; t += 20u)
        {
            c[SAFETY_HB_MISSION]++;     /* only Mission keeps running */
            m = safety_hb_check(&st, c, t);
            if (t == 1040u) { CHECK("S4-H09", m == 0u); }
            if (t == 1060u) { CHECK("S4-H09", m == ((1u << SAFETY_HB_MOTION) | (1u << SAFETY_HB_LINE))); }
            if (t == 1120u) { CHECK("S4-H09", (m & (1u << SAFETY_HB_IMU)) != 0u); }
        }
        CHECK("S4-H09", (m & (1u << SAFETY_HB_MISSION)) == 0u);
        CHECK("S4-H09", (m & (1u << SAFETY_HB_OBSTACLE)) != 0u);    /* > 300 ms */
        safety_hb_start(&st, c, 1320u);
        CHECK("S4-H09", safety_hb_check(&st, c, 1360u) == 0u);
        CHECK("S4-H09", safety_hb_timeout_ms(SAFETY_HB_IMU) == 100u);
        /* the 32-bit millisecond wrap does not trigger a timeout */
        safety_hb_start(&st, c, 0xFFFFFFF0u);
        CHECK("S4-H09", safety_hb_check(&st, c, 0x00000010u) == 0u);
    }

    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
