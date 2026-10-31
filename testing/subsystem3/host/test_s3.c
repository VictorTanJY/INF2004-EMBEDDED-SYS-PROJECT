/*
 *  test_s3.c
 *  Host unit tests for subsystem 3: Code 39 decoding (code39.c), barcode
 *  capture from a 1 kHz sample stream (line_barcode.c) and line tracking
 *  (line_track.c).  The barcode tests use the element widths of the
 *  instructor's sample sheets (vectors_slides.h): A, F and Z at all three
 *  printed sizes, plus W, X and Y.
 *  Build and run:  make -C testing/subsystem3/host
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "code39.h"
#include "line_barcode.h"
#include "line_track.h"
#include "vectors_slides.h"

static int g_pass;
static int g_fail;

#define CHECK(id, cond) \
    do { if (cond) { g_pass++; } else { g_fail++; \
        printf("FAIL %s  (%s:%d)  %s\n", id, __FILE__, __LINE__, #cond); } } while (0)

#define N_SLIDES ((int)(sizeof(k_slides) / sizeof(k_slides[0])))

static char const k_data_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

static uint32_t g_rng = 12345u;
static uint32_t rnd(void) { g_rng = g_rng * 1103515245u + 12345u; return (g_rng >> 8) & 0xFFFFu; }
/* uniform in [-1, 1] */
static double rndu(void) { return ((double)rnd() / 32767.5) - 1.0; }

/* Widths of *c* with the given narrow width, ratio and +-jitter. */
static uint16_t make_code(char c, uint32_t narrow, double ratio, double jitter, uint32_t * p_w)
{
    char const seq[3] = { '*', c, '*' };
    uint16_t n = 0u;
    int k;
    int e;

    for (k = 0; k < 3; k++)
    {
        uint16_t pat = code39_pattern(seq[k]);

        for (e = 0; e < 9; e++)
        {
            double w = (double)narrow * (((pat >> (8 - e)) & 1u) ? ratio : 1.0);
            p_w[n++] = (uint32_t)(w * (1.0 + jitter * rndu()));
        }
        if (k < 2)
        {
            p_w[n++] = (uint32_t)((double)narrow * (1.0 + jitter * rndu()));   /* gap */
        }
    }
    return n;
}

static void reverse(uint32_t * p_w, uint16_t n)
{
    uint16_t i;

    for (i = 0u; i < n / 2u; i++)
    {
        uint32_t t = p_w[i];
        p_w[i] = p_w[n - 1u - i];
        p_w[n - 1u - i] = t;
    }
}

/* ---- sample-stream simulation ---------------------------------------- */
#define TRACK_LEN_UM    1200000u        /* 1.2 m of track per pass */

/* Reflectance (0..1000, 1000 = black) at position x along a track with
   60 mm of white, then the code, then white.  Sensor spot: 1.5 mm box. */
static uint16_t profile_at(uint32_t const * p_w, uint16_t n, uint32_t x_um)
{
    uint32_t const lead = 60000u;
    uint32_t       spot;
    uint32_t       black = 0u;

    for (spot = 0u; spot < 1500u; spot += 100u)
    {
        uint32_t x = x_um + spot;
        uint32_t pos = lead;
        uint16_t e;

        for (e = 0u; e < n; e++)
        {
            if ((x >= pos) && (x < pos + p_w[e]))
            {
                black += ((e % 2u) == 0u) ? 1u : 0u;
                break;
            }
            pos += p_w[e];
        }
    }
    return (uint16_t)(80u + (840u * black) / 15u);     /* white 80, black 920 */
}

/* Run one pass at v_mm_s; returns the result and fills *p_out. */
static bc_result_t run_pass(uint32_t const * p_w, uint16_t n, uint32_t v_mm_s, int noise, bc_out_t * p_out)
{
    bc_capture_t bc;
    uint32_t     x;
    uint32_t     step = v_mm_s;     /* um per 1 ms sample = mm/s */
    bc_result_t  found = BC_NONE;

    bc_init(&bc);
    for (x = 0u; x < TRACK_LEN_UM; x += step)
    {
        int32_t  s = (int32_t)profile_at(p_w, n, x) + (int32_t)(noise * rndu());
        bc_out_t out;
        bc_result_t r;

        if (s < 0) { s = 0; }
        if (s > 1000) { s = 1000; }
        memset(&out, 0, sizeof(out));
        r = bc_sample(&bc, (uint16_t)s, step, &out);
        if ((r != BC_NONE) && (found == BC_NONE))
        {
            found = r;
            *p_out = out;
        }
        else if (r != BC_NONE)
        {
            return BC_REJECT;           /* more than one result per code */
        }
    }
    return found;
}

static void test_table(void)
{
    size_t i;
    size_t j;
    int    ok = 1;

    for (i = 0u; i < sizeof(k_data_chars) - 1u; i++)
    {
        uint16_t p = code39_pattern(k_data_chars[i]);
        int wide = 0, wide_sp = 0, b;

        for (b = 0; b < 9; b++)
        {
            if ((p >> (8 - b)) & 1u) { wide++; if (b % 2) { wide_sp++; } }
        }
        if ((p == 0u) || (wide != 3) || ((wide_sp != 1) && (wide_sp != 3))) { ok = 0; }
        if (code39_char(p) != k_data_chars[i]) { ok = 0; }
        for (j = 0u; j < i; j++)
        {
            if (code39_pattern(k_data_chars[j]) == p) { ok = 0; }
        }
    }
    CHECK("S3-H01", ok);
    CHECK("S3-H01", code39_pattern('*') == 0x094u && code39_char(0x1FFu) == '\0');
}

static void test_slides(void)
{
    int s;

    for (s = 0; s < N_SLIDES; s++)
    {
        uint32_t w[29];
        char     ch;
        bool     b_rev;
        int      k;
        double   scale;

        for (scale = 1.0; scale > 0.5; scale -= (1.0 - 297.0 / 338.67))   /* screen, A4 */
        {
            for (k = 0; k < 29; k++) { w[k] = (uint32_t)(k_slides[s].p_w[k] * scale); }
            CHECK("S3-H02", code39_decode_one(w, 29, 15, &ch, &b_rev) == CODE39_OK &&
                            ch == k_slides[s].ch && !b_rev);
            reverse(w, 29);
            CHECK("S3-H02", code39_decode_one(w, 29, 15, &ch, &b_rev) == CODE39_OK &&
                            ch == k_slides[s].ch && b_rev);
            if (scale < 1.0) { break; }
        }
    }
}

static void test_synthetic(void)
{
    uint32_t const narrows[3] = { 3000u, 5100u, 6900u };
    int ok = 1, wrong = 0;
    size_t i;
    int a, r, d;

    for (i = 0u; i < sizeof(k_data_chars) - 1u; i++)
    {
        for (a = 0; a < 3; a++)
        {
            for (r = 0; r < 3; r++)
            {
                double const ratio = (r == 0) ? 2.2 : ((r == 1) ? 2.5 : 3.0);

                for (d = 0; d < 2; d++)
                {
                    uint32_t w[29];
                    uint16_t n = make_code(k_data_chars[i], narrows[a], ratio, 0.15, w);
                    char ch;
                    bool b_rev;
                    code39_result_t res;

                    if (d) { reverse(w, n); }
                    res = code39_decode_one(w, n, 15, &ch, &b_rev);
                    if (res != CODE39_OK) { ok = 0; }
                    else if (ch != k_data_chars[i]) { wrong++; }
                }
            }
        }
    }
    CHECK("S3-H03", ok && (wrong == 0));
}

static void test_rejects(void)
{
    uint32_t w[40];
    uint16_t n;
    char ch;
    bool b_rev;
    int  i, wrong = 0;

    /* a wide bar printed narrow: rejected, never a different character */
    for (i = 0; i < (int)(sizeof(k_data_chars) - 1u); i++)
    {
        int e;
        for (e = 10; e < 19; e++)
        {
            uint16_t pat = code39_pattern(k_data_chars[i]);

            n = make_code(k_data_chars[i], 3000u, 3.0, 0.0, w);
            if ((pat >> (8 - (e - 10))) & 1u)
            {
                w[e] = 3000u;           /* wide -> narrow */
                if (code39_decode_one(w, n, 15, &ch, &b_rev) == CODE39_OK) { wrong++; }
            }
        }
    }
    CHECK("S3-H04", wrong == 0);
    n = make_code('A', 3000u, 3.0, 0.0, w);
    CHECK("S3-H04", code39_decode_one(w, 28, 15, &ch, &b_rev) == CODE39_ERR_COUNT);
    n = make_code('A', 3000u, 1.3, 0.0, w);
    CHECK("S3-H04", code39_decode_one(w, n, 15, &ch, &b_rev) == CODE39_ERR_RATIO);
    CHECK("S3-H04", code39_decode_one(w, 5, 15, &ch, &b_rev) == CODE39_NOT_BARCODE);
    {
        /* "*AB*": valid Code 39 but two data characters */
        uint32_t w2[40];
        char const seq[4] = { '*', 'A', 'B', '*' };
        int k, e;
        n = 0u;
        for (k = 0; k < 4; k++)
        {
            uint16_t pat = code39_pattern(seq[k]);
            for (e = 0; e < 9; e++) { w2[n++] = ((pat >> (8 - e)) & 1u) ? 9000u : 3000u; }
            if (k < 3) { w2[n++] = 3000u; }
        }
        CHECK("S3-H04", code39_decode_one(w2, n, 15, &ch, &b_rev) == CODE39_ERR_LENGTH);
    }
}

static void test_capture(void)
{
    uint32_t const speeds[2] = { 150u, 300u };      /* NFR3 speeds, mm/s */
    int s, v, d, pass;
    int good = 0, total = 0, wrong = 0;

    /* every slide, both speeds, both directions, clean */
    for (s = 0; s < N_SLIDES; s++)
    {
        for (v = 0; v < 2; v++)
        {
            for (d = 0; d < 2; d++)
            {
                uint32_t w[29];
                bc_out_t out;
                int k;

                for (k = 0; k < 29; k++) { w[k] = k_slides[s].p_w[k] * 297u / 339u; }
                if (d) { reverse(w, 29); }
                memset(&out, 0, sizeof(out));
                CHECK("S3-H05", run_pass(w, 29, speeds[v], 0, &out) == BC_VALID && out.ch == k_slides[s].ch);
            }
        }
    }

    /* NFR3-style: 20 noisy passes per character per speed and direction */
    for (s = 0; s < N_SLIDES; s += 3)
    {
        for (v = 0; v < 2; v++)
        {
            for (pass = 0; pass < 20; pass++)
            {
                uint32_t w[29];
                bc_out_t out;
                bc_result_t r;
                int k;

                for (k = 0; k < 29; k++)
                {
                    w[k] = (uint32_t)((double)k_slides[s].p_w[k] * 297.0 / 338.67 * (1.0 + 0.10 * rndu()));
                }
                if (pass % 2) { reverse(w, 29); }
                memset(&out, 0, sizeof(out));
                r = run_pass(w, 29, speeds[v], 120, &out);
                total++;
                if ((r == BC_VALID) && (out.ch == k_slides[s].ch)) { good++; }
                else if (r == BC_VALID) { wrong++; }
            }
        }
    }
    printf("noisy passes: %d / %d decoded, %d wrong\n", good, total, wrong);
    CHECK("S3-H05", wrong == 0 && (good * 100) >= (total * 95));

    /* S3-H06: a crossing line (18 mm black) is not a barcode and not an error */
    {
        uint32_t w[1] = { 18000u };
        bc_out_t out;
        CHECK("S3-H06", run_pass(w, 1, 300u, 0, &out) == BC_NONE);
    }
    /* S3-H07: a long dark patch is rejected, never decoded */
    {
        uint32_t w[1] = { 500000u };
        bc_out_t out;
        memset(&out, 0, sizeof(out));
        CHECK("S3-H07", run_pass(w, 1, 300u, 0, &out) == BC_REJECT && out.reason == CODE39_ERR_TOO_LONG);
    }
}

/* ---- line tracking ---------------------------------------------------- */
static line_trk_cfg_t const k_cfg = { 20000u, 80000u, 300 };

/* Feed blocks of 3 mm (0.3 m/s x 10 ms) for dist_um; returns OR of events. */
static uint8_t feed(line_track_t * p_t, uint16_t l, uint16_t r, uint32_t dist_um)
{
    uint8_t ev = 0u;
    uint32_t d;

    for (d = 0u; d < dist_um; d += 3000u) { ev |= line_track_step(p_t, &k_cfg, l, r, 3000u); }
    return ev;
}

static void test_track(void)
{
    line_track_t t;
    uint8_t ev;

    /* S3-H10: steering sign: line under the left sensor -> turn left */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    CHECK("S3-H10", t.err == 0 && t.omega == 0 && t.v == 300);
    (void)line_track_step(&t, &k_cfg, 800u, 200u, 3000u);
    CHECK("S3-H10", t.err == 600 && t.omega > 0 && t.v < 300);
    (void)feed(&t, 200u, 800u, 9000u);
    CHECK("S3-H10", t.omega < 0);

    /* S3-H08: junction on a straight approach, once; not on a tight curve */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    ev = feed(&t, 1000u, 1000u, 15000u);
    CHECK("S3-H08", (ev & LINE_TRK_EV_JUNCTION) == 0u);
    ev = feed(&t, 1000u, 1000u, 15000u);
    CHECK("S3-H08", (ev & LINE_TRK_EV_JUNCTION) != 0u && t.omega == 0);
    ev = feed(&t, 1000u, 1000u, 30000u);
    CHECK("S3-H08", (ev & LINE_TRK_EV_JUNCTION) == 0u);
    line_track_init(&t);
    (void)feed(&t, 950u, 300u, 30000u);        /* hard steering on a curve */
    ev = feed(&t, 1000u, 1000u, 30000u);
    CHECK("S3-H08", (ev & LINE_TRK_EV_JUNCTION) == 0u);

    /* S3-H09: 40 mm gap bridged (heading held), 90 mm -> LOST once, then FOUND */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    ev = feed(&t, 50u, 50u, 40000u);
    CHECK("S3-H09", (ev & LINE_TRK_EV_LOST) == 0u && t.omega == 0 && t.v == 300);
    (void)feed(&t, 500u, 500u, 30000u);
    ev = feed(&t, 50u, 50u, 90000u);
    CHECK("S3-H09", (ev & LINE_TRK_EV_LOST) != 0u && t.b_lost && t.v == 0);
    ev = feed(&t, 50u, 50u, 90000u);
    CHECK("S3-H09", (ev & LINE_TRK_EV_LOST) == 0u);
    ev = feed(&t, 600u, 400u, 3000u);
    CHECK("S3-H09", (ev & LINE_TRK_EV_FOUND) != 0u && !t.b_lost);

    /* S3-H11: drifting off the right of the line -> keeps turning back left */
    line_track_init(&t);
    (void)feed(&t, 500u, 500u, 60000u);
    (void)feed(&t, 950u, 80u, 6000u);          /* line under the left sensor only */
    (void)feed(&t, 60u, 60u, 9000u);           /* ... then past it */
    CHECK("S3-H11", t.omega > 0 && t.v < 300);

    /* S3-H12: calibration: contrast check and normalisation */
    {
        line_cal_t cal;
        uint16_t raw[3];
        int i;

        memset(&cal, 0, sizeof(cal));
        line_cal_start(&cal);
        for (i = 0; i < 100; i++)
        {
            raw[0] = (uint16_t)((i < 50) ? 300 : 3000);  /* white 300, black 3000 */
            raw[1] = (uint16_t)((i < 50) ? 350 : 2900);
            raw[2] = (uint16_t)((i < 50) ? 250 : 2500);
            line_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", line_cal_finish(&cal) && cal.contrast_x10 >= 30);
        CHECK("S3-H12", line_cal_norm(&cal, 0, 300) == 0 && line_cal_norm(&cal, 0, 3000) == 1000);
        CHECK("S3-H12", line_cal_norm(&cal, 0, 1650) == 500 && line_cal_norm(&cal, 0, 4095) == 1000);
        line_cal_start(&cal);
        for (i = 0; i < 100; i++)
        {
            raw[0] = (uint16_t)((i < 50) ? 1000 : 2000);  /* 2:1 only */
            raw[1] = 1000; raw[2] = 1000;
            line_cal_sample(&cal, raw);
        }
        CHECK("S3-H12", !line_cal_finish(&cal) && cal.b_valid && line_cal_norm(&cal, 0, 3000) == 1000);
    }
}

int main(void)
{
    test_table();
    test_slides();
    test_synthetic();
    test_rejects();
    test_capture();
    test_track();
    printf("host tests: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
